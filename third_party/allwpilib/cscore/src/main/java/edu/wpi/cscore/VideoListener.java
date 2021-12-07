// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.cscore;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

/**
 * An event listener. This calls back to a desigated callback function when an event matching the
 * specified mask is generated by the library.
 */
public class VideoListener implements AutoCloseable {
  /**
   * Create an event listener.
   *
   * @param listener Listener function
   * @param eventMask Bitmask of VideoEvent.Type values
   * @param immediateNotify Whether callback should be immediately called with a representative set
   *     of events for the current library state.
   */
  public VideoListener(Consumer<VideoEvent> listener, int eventMask, boolean immediateNotify) {
    s_lock.lock();
    try {
      if (s_poller == 0) {
        s_poller = CameraServerJNI.createListenerPoller();
        startThread();
      }
      m_handle = CameraServerJNI.addPolledListener(s_poller, eventMask, immediateNotify);
      s_listeners.put(m_handle, listener);
    } finally {
      s_lock.unlock();
    }
  }

  @Override
  public synchronized void close() {
    if (m_handle != 0) {
      s_lock.lock();
      try {
        s_listeners.remove(m_handle);
      } finally {
        s_lock.unlock();
      }
      CameraServerJNI.removeListener(m_handle);
      m_handle = 0;
    }
  }

  public boolean isValid() {
    return m_handle != 0;
  }

  private int m_handle;

  private static final ReentrantLock s_lock = new ReentrantLock();
  private static final Map<Integer, Consumer<VideoEvent>> s_listeners = new HashMap<>();
  private static Thread s_thread;
  private static int s_poller;
  private static boolean s_waitQueue;
  private static final Condition s_waitQueueCond = s_lock.newCondition();

  @SuppressWarnings("PMD.AvoidCatchingThrowable")
  private static void startThread() {
    s_thread =
        new Thread(
            () -> {
              boolean wasInterrupted = false;
              while (!Thread.interrupted()) {
                VideoEvent[] events;
                try {
                  events = CameraServerJNI.pollListener(s_poller);
                } catch (InterruptedException ex) {
                  s_lock.lock();
                  try {
                    if (s_waitQueue) {
                      s_waitQueue = false;
                      s_waitQueueCond.signalAll();
                      continue;
                    }
                  } finally {
                    s_lock.unlock();
                  }
                  Thread.currentThread().interrupt();
                  // don't try to destroy poller, as its handle is likely no longer valid
                  wasInterrupted = true;
                  break;
                }
                for (VideoEvent event : events) {
                  Consumer<VideoEvent> listener;
                  s_lock.lock();
                  try {
                    listener = s_listeners.get(event.listener);
                  } finally {
                    s_lock.unlock();
                  }
                  if (listener != null) {
                    try {
                      listener.accept(event);
                    } catch (Throwable throwable) {
                      System.err.println(
                          "Unhandled exception during listener callback: " + throwable.toString());
                      throwable.printStackTrace();
                    }
                  }
                }
              }
              s_lock.lock();
              try {
                if (!wasInterrupted) {
                  CameraServerJNI.destroyListenerPoller(s_poller);
                }
                s_poller = 0;
              } finally {
                s_lock.unlock();
              }
            },
            "VideoListener");
    s_thread.setDaemon(true);
    s_thread.start();
  }
}