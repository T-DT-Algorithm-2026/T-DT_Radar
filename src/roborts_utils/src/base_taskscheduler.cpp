#include "base_taskscheduler.h"

namespace tdttoolkit {
TaskScheduler::TaskScheduler() {}

void TaskScheduler::async_stop() { is_running = false; }
}  // namespace tdttoolkit