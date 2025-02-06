#ifndef __BASE_TASKSCHEDULER_H__
#define __BASE_TASKSCHEDULER_H__
#include <thread>

#include "base_msg.h"
#include "base_toolkit.h"

namespace tdttoolkit {
class TaskScheduler {
 private:
  std::thread thread;
  bool is_running = true;

 public:
  TaskScheduler();

  /**************************************************************************
   * @name async_run
   * @brief: Run a task in a new thread
   * @param: time_per_frame: time per frame in mileseconds
   * @param: task: task to run
   * @param: obj: object to run the task
   */
  template <class T>
  void async_run(float time_per_frame, void (T::*task)(), T *obj) {
    thread =
        std::thread(&TaskScheduler::run<T>, this, time_per_frame, task, obj);
    thread.detach();
  }

  /**************************************************************************
   * @name run
   * @brief: Run a task
   * @param: time_per_frame: time per frame in mileseconds
   * @param: task: task to run
   * @param: obj: object to run the task
   */
  template <class T>
  void run(float time_per_frame, void (T::*task)(), T *obj) {
    while (is_running) {
      clock_t start = tdttoolkit::Time::GetTimeNow();
      (obj->*task)();
      clock_t end = tdttoolkit::Time::GetTimeNow();
      float time = (end - start) / 1e3;
      if (time < time_per_frame) {
        std::this_thread::sleep_for(
            std::chrono::nanoseconds((int)((time_per_frame - time) * 1e6)));
      }
    }
  }

  /**************************************************************************
   * @name async_stop
   * @brief: Stop a task thread
   */
  void async_stop();
};
}  // namespace tdttoolkit

#endif