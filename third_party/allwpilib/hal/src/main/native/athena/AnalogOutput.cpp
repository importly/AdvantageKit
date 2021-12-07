// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/AnalogOutput.h"

#include "AnalogInternal.h"
#include "HALInitializer.h"
#include "PortsInternal.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"
#include "hal/handles/IndexedHandleResource.h"

using namespace hal;

namespace {

struct AnalogOutput {
  uint8_t channel;
};

}  // namespace

static IndexedHandleResource<HAL_AnalogOutputHandle, AnalogOutput,
                             kNumAnalogOutputs, HAL_HandleEnum::AnalogOutput>*
    analogOutputHandles;

namespace hal::init {
void InitializeAnalogOutput() {
  static IndexedHandleResource<HAL_AnalogOutputHandle, AnalogOutput,
                               kNumAnalogOutputs, HAL_HandleEnum::AnalogOutput>
      aoH;
  analogOutputHandles = &aoH;
}
}  // namespace hal::init

extern "C" {

HAL_AnalogOutputHandle HAL_InitializeAnalogOutputPort(HAL_PortHandle portHandle,
                                                      int32_t* status) {
  hal::init::CheckInit();
  initializeAnalog(status);

  if (*status != 0) {
    return HAL_kInvalidHandle;
  }

  int16_t channel = getPortHandleChannel(portHandle);
  if (channel == InvalidHandleIndex) {
    *status = PARAMETER_OUT_OF_RANGE;
    return HAL_kInvalidHandle;
  }

  HAL_AnalogOutputHandle handle =
      analogOutputHandles->Allocate(channel, status);

  if (*status != 0) {
    return HAL_kInvalidHandle;  // failed to allocate. Pass error back.
  }

  auto port = analogOutputHandles->Get(handle);
  if (port == nullptr) {  // would only error on thread issue
    *status = HAL_HANDLE_ERROR;
    return HAL_kInvalidHandle;
  }

  port->channel = static_cast<uint8_t>(channel);
  return handle;
}

void HAL_FreeAnalogOutputPort(HAL_AnalogOutputHandle analogOutputHandle) {
  // no status, so no need to check for a proper free.
  analogOutputHandles->Free(analogOutputHandle);
}

void HAL_SetAnalogOutput(HAL_AnalogOutputHandle analogOutputHandle,
                         double voltage, int32_t* status) {
  auto port = analogOutputHandles->Get(analogOutputHandle);
  if (port == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  uint16_t rawValue = static_cast<uint16_t>(voltage / 5.0 * 0x1000);

  if (voltage < 0.0) {
    rawValue = 0;
  } else if (voltage > 5.0) {
    rawValue = 0x1000;
  }

  analogOutputSystem->writeMXP(port->channel, rawValue, status);
}

double HAL_GetAnalogOutput(HAL_AnalogOutputHandle analogOutputHandle,
                           int32_t* status) {
  auto port = analogOutputHandles->Get(analogOutputHandle);
  if (port == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return 0.0;
  }

  uint16_t rawValue = analogOutputSystem->readMXP(port->channel, status);

  return rawValue * 5.0 / 0x1000;
}

HAL_Bool HAL_CheckAnalogOutputChannel(int32_t channel) {
  return channel < kNumAnalogOutputs && channel >= 0;
}

}  // extern "C"