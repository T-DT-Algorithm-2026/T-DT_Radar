/**
 * @file base_blackboard.h
 * @author 20严俊涵(359360997@qq.com)
 * @brief 黑板类,用于全局数据共享
 * @version 0.1
 * @date 2023-04-05
 *
 * @copyright T-DT Copyright (c) 2023
 *
 */

#ifndef __BASE_BLACKBOARD_H
#define __BASE_BLACKBOARD_H

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/process.hpp>
#include <iostream>
#include <map>
#include <memory>
#include <thread>

#ifdef _WIN32
#include <process.h>  // for _getpid()
#else
#include <unistd.h>  // for getpid()
#endif

#include "base_msg.h"

#define BLACKBOARD_MAX_SUBSCRIBER 20

namespace tdttoolkit {
class BaseBlackboard {
 private:
  class shmem {
    // 对于一个共享内存区域，定义前sizeof(boost::interprocess::interprocess_mutex)个字节为进程锁，
    // 之后sizeof(int)个字节为订阅者数量，之后max_subscriber *
    // sizeof(int)个字节为订阅者pid， 之后sizeof(int)个字节为消息发布指示区，
    // 再之后的字节为读写内存区域
   public:
    shmem(std::string name, int size, int max_subscriber, bool& is_new);

    ~shmem();

    std::string name;
    boost::interprocess::shared_memory_object mem;
    boost::interprocess::mapped_region region;
  };

  class shmem_node {
   public:
    shmem_node(std::string name);

    ~shmem_node();

    template <typename T>
    shmem_node& operator<<(const T& dat) {
      if (!locked) mutex->lock();
      T* addr = (T*)(static_cast<char*>(this->addr) +
                     sizeof(boost::interprocess::interprocess_mutex) +
                     2 * sizeof(int) + BLACKBOARD_MAX_SUBSCRIBER * sizeof(int) +
                     write_cnt);
      *addr = dat;
      if (!locked) mutex->unlock();
      write_cnt += sizeof(T);
      return *this;
    }

    template <typename T>
    shmem_node& operator>>(T& dat) {
      if (!locked) mutex->lock();
      T* addr = (T*)(static_cast<char*>(this->addr) +
                     sizeof(boost::interprocess::interprocess_mutex) +
                     2 * sizeof(int) + BLACKBOARD_MAX_SUBSCRIBER * sizeof(int) +
                     read_cnt);
      dat = *addr;
      if (!locked) mutex->unlock();
      read_cnt += sizeof(T);
      return *this;
    }

    void publish();

    void reset_publish();

    bool is_published();

    inline void reset_read_iter() { read_cnt = 0; }

    inline void reset_write_iter() { write_cnt = 0; }

    inline void lock() {
      mutex->lock();
      locked = true;
    }

    inline void unlock() {
      mutex->unlock();
      locked = false;
    }

   private:
    void* addr;
    boost::interprocess::interprocess_mutex* mutex;
    int read_cnt = 0, write_cnt = 0;
    bool locked = false;
  };

  struct subscriber {
    ;
    ;
  };

 private:
  static std::map<std::string, std::shared_ptr<shmem>> blackboard_map_;

 public:
  /*****
   * @brief 初始化黑板
   * @param name 黑板名
   * @param size 黑板大小
   * @return 0 打开并初始化成功 1 创建并初始化成功 2 初始化失败
   */
  static uint8_t Init(std::string name, int size);

  inline static shmem_node On(std::string name) { return shmem_node(name); }

  template <typename T>
  inline static void Get(std::string name, T& dat) {
    shmem_node node = shmem_node(name);
    node >> dat;
  }

  template <typename T>
  inline static T Get(std::string name) {
    T dat;
    shmem_node node = shmem_node(name);
    node >> dat;
    return dat;
  }

  template <typename T>
  inline static void Set(std::string name, const T& dat) {
    shmem_node node = shmem_node(name);
    node << dat;
  }

  template <typename T>
  inline static void Publish(std::string name, T dat) {
    shmem_node node = shmem_node(name);
    node.lock();
    node << dat;
    node.publish();
    node.unlock();
  }

  template <typename T>
  static void Subcribe(std::string name, void (*callback)(T&)) {
    shmem_node node = shmem_node(name);
    while (true) {
      if (node.is_published()) {
        T dat;
        node.lock();
        node >> dat;
        node.reset_publish();
        node.reset_read_iter();
        node.unlock();
        callback(dat);
      }
      std::this_thread::sleep_for(std::chrono::nanoseconds(50000));
    }
  }

  template <typename T>
  inline static bool SubcribeSome(std::string name, void (*callback)(T&)) {
    shmem_node node = shmem_node(name);
    if (node.is_published()) {
      T dat;
      node.lock();
      node >> dat;
      node.reset_publish();
      node.reset_read_iter();
      node.unlock();
      callback(dat);
      return true;
    }
    return false;
  }
};

}  // namespace tdttoolkit

#endif  // __BASE_BLACKBOARD_H
