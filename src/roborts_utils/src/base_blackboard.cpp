#include "base_blackboard.h"

namespace tdttoolkit {

std::map<std::string, std::shared_ptr<BaseBlackboard::shmem>>
    BaseBlackboard::blackboard_map_;

BaseBlackboard::shmem::shmem(std::string name, int size, int max_subscriber,
                             bool &is_new) {
  this->name = name;
  is_new = false;
  try {
    mem = boost::interprocess::shared_memory_object(
        boost::interprocess::open_only, name.c_str(),
        boost::interprocess::read_write);
    region = boost::interprocess::mapped_region(
        mem, boost::interprocess::read_write);  // map the whole shared memory
                                                // in this process
  } catch (boost::interprocess::interprocess_exception &e) {
    mem = boost::interprocess::shared_memory_object(
        boost::interprocess::open_or_create, name.c_str(),
        boost::interprocess::read_write);
    mem.truncate(size + sizeof(int) * max_subscriber + 2 * sizeof(int) +
                 sizeof(boost::interprocess::interprocess_mutex));
    region = boost::interprocess::mapped_region(
        mem, boost::interprocess::read_write);  // map the whole shared memory
                                                // in this process
    auto addr = region.get_address();
    auto mutex = new (addr) boost::interprocess::interprocess_mutex;
    int *subscriber_num = new (
        static_cast<char *>(addr) +
        sizeof(boost::interprocess::interprocess_mutex)) int(max_subscriber);
    is_new = true;
  }
  int pid;
#ifdef _WIN32
  pid = _getpid();
#else
  pid = getpid();
#endif

  auto addr = region.get_address();
  auto mutex = (boost::interprocess::interprocess_mutex *)(addr);
  int *subscriber_num =
      (int *)(static_cast<char *>(addr) +
              sizeof(boost::interprocess::interprocess_mutex));
  std::vector<int> available_subscriber_pids;
  for (int i = 0; i < max_subscriber; i++) {
    int *subscriber_pid =
        (int *)(static_cast<char *>(addr) +
                sizeof(boost::interprocess::interprocess_mutex) + sizeof(int) +
                i * sizeof(int));
    // 检查当前订阅者是否在线
#ifdef _WIN32
    if (*subscriber_pid > 0 && _getpgid(*subscriber_pid) >= 0 &&
        *subscriber_pid != pid) {
      available_subscriber_pids.push_back(*subscriber_pid);
    }
#else
    if (*subscriber_pid > 0 && getpgid(*subscriber_pid) >= 0 &&
        *subscriber_pid != pid) {
      available_subscriber_pids.push_back(*subscriber_pid);
    }
#endif
  }
  if (available_subscriber_pids.empty() && *subscriber_num > 0) {
    boost::interprocess::shared_memory_object::remove(name.c_str());
    mem = boost::interprocess::shared_memory_object(
        boost::interprocess::open_or_create, name.c_str(),
        boost::interprocess::read_write);
    mem.truncate(size + sizeof(int) * max_subscriber + 2 * sizeof(int) +
                 sizeof(boost::interprocess::interprocess_mutex));
    region = boost::interprocess::mapped_region(
        mem, boost::interprocess::read_write);  // map the whole shared
                                                // memory in this process
    addr = region.get_address();
    mutex = new (addr) boost::interprocess::interprocess_mutex;
    subscriber_num = new (
        static_cast<char *>(addr) +
        sizeof(boost::interprocess::interprocess_mutex)) int(max_subscriber);
  }
  available_subscriber_pids.push_back(pid);

  // 更新订阅者数量
  mutex->lock();
  for (int i = 0; i < available_subscriber_pids.size(); i++) {
    int *subscriber_pid =
        (int *)(static_cast<char *>(addr) +
                sizeof(boost::interprocess::interprocess_mutex) + sizeof(int) +
                i * sizeof(int));
    *subscriber_pid = available_subscriber_pids[i];
  }
  for (int i = available_subscriber_pids.size(); i < max_subscriber; i++) {
    int *subscriber_pid =
        (int *)(static_cast<char *>(addr) +
                sizeof(boost::interprocess::interprocess_mutex) + sizeof(int) +
                i * sizeof(int));
    *subscriber_pid = 0;
  }
  *subscriber_num = available_subscriber_pids.size();
  mutex->unlock();
}

BaseBlackboard::shmem::~shmem() {
#ifdef _WIN32
  auto pid = _getpid();
#else
  auto pid = getpid();
#endif
  auto addr = region.get_address();
  auto mutex = (boost::interprocess::interprocess_mutex *)(addr);
  int *subscriber_num =
      (int *)(static_cast<char *>(addr) +
              sizeof(boost::interprocess::interprocess_mutex));
  std::vector<int> available_subscriber_pids;
  for (int i = 0; i < *subscriber_num; i++) {
    int *subscriber_pid =
        (int *)(static_cast<char *>(addr) +
                sizeof(boost::interprocess::interprocess_mutex) + sizeof(int) +
                i * sizeof(int));
    if (*subscriber_pid > 0 && *subscriber_pid != pid)
      available_subscriber_pids.push_back(*subscriber_pid);
  }
  if (available_subscriber_pids.empty()) {
    boost::interprocess::shared_memory_object::remove(name.c_str());
  } else {
    mutex->lock();
    for (int i = 0; i < available_subscriber_pids.size(); i++) {
      int *subscriber_pid =
          (int *)(static_cast<char *>(addr) +
                  sizeof(boost::interprocess::interprocess_mutex) +
                  sizeof(int) + i * sizeof(int));
      *subscriber_pid = available_subscriber_pids[i];
    }
    for (int i = available_subscriber_pids.size(); i < *subscriber_num; i++) {
      int *subscriber_pid =
          (int *)(static_cast<char *>(addr) +
                  sizeof(boost::interprocess::interprocess_mutex) +
                  sizeof(int) + i * sizeof(int));
      *subscriber_pid = 0;
    }
    *subscriber_num = available_subscriber_pids.size();
    mutex->unlock();
  }
}

BaseBlackboard::shmem_node::shmem_node(std::string name) {
  if (BaseBlackboard::blackboard_map_.find(name) ==
      BaseBlackboard::blackboard_map_.end()) {
    TDT_FATAL("BaseBlackboard::Get(): Blackboard %s not found", name.c_str());
  }
  auto &region = BaseBlackboard::blackboard_map_[name]->region;
  addr = region.get_address();
  mutex = (boost::interprocess::interprocess_mutex *)(addr);
}

BaseBlackboard::shmem_node::~shmem_node() { if(locked)mutex->unlock(); }

uint8_t BaseBlackboard::Init(std::string name, int size) {
  bool is_new = false;
  try {
    auto shmem_ =
        std::make_shared<shmem>(name, size, BLACKBOARD_MAX_SUBSCRIBER, is_new);
    blackboard_map_[name] = shmem_;
  } catch (...) {
    return 2;
  }
  if (is_new)
    return 1;
  else
    return 0;
}

void BaseBlackboard::shmem_node::publish() {
  if (!locked) mutex->lock();
  int *addr =
      new (static_cast<char *>(this->addr) +
           sizeof(boost::interprocess::interprocess_mutex) + sizeof(int) +
           BLACKBOARD_MAX_SUBSCRIBER * sizeof(int)) int(1);
  if (!locked) mutex->unlock();
}

void BaseBlackboard::shmem_node::reset_publish() {
  if (!locked) mutex->lock();
  int *addr =
      new (static_cast<char *>(this->addr) +
           sizeof(boost::interprocess::interprocess_mutex) + sizeof(int) +
           BLACKBOARD_MAX_SUBSCRIBER * sizeof(int)) int(0);
  if (!locked) mutex->unlock();
}

bool BaseBlackboard::shmem_node::is_published() {
  bool ret = false;
  if (!locked) mutex->lock();
  ret = *(int *)(static_cast<char *>(this->addr) +
                 sizeof(boost::interprocess::interprocess_mutex) + sizeof(int) +
                 BLACKBOARD_MAX_SUBSCRIBER * sizeof(int));
  if (!locked) mutex->unlock();
  return ret;
}

}  // namespace tdttoolkit