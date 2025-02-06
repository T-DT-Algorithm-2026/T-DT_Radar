#ifndef __BASE_MSG_H
#define __BASE_MSG_H
#include <fcntl.h>
#include <linux/kd.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

///
/// \brief 日志文件的类型
///
enum Log_Rank { DEBUG = 0, INFO = 1, WARNING = 2, ERROR = 3, FATAL = 4 };

/// \brief 日志系统类
///
// TODO: 和tdt_log结合
class Logger {
 public:
  struct MyException : public std::exception {
    const char *what() const throw() {
      return "\033[0;32;31m程序调用了TDT_FATAL,"
             "通常由于程序某处发生了致命的错误,若未进行异常捕获程序将会结束,"
             "你可以阅读上方的报错信息和报错代码位置\033[m";
    }
  };
  // 构造函数
  Logger(Log_Rank log_rank) : log_rank_(log_rank){};

  ~Logger();
  ///
  /// \brief 写入日志信息之前先写入的源代码文件名, 行号, 函数名
  /// \param log_rank 日志的等级
  /// \param line 日志发生的行号
  /// \param function 日志发生的函数
  // static void taken(Log_Rank log_rank,
  //                   std::string massage,
  //                   const int line,
  //                   const std::string& function);

  void taken(const int line, const std::string &function,
             char const *msg_format, ...);
  class StreamerNode {
   private:
    int log_rank_ = 0;
    char buffer_[1024];

   public:
    StreamerNode(const int line, const std::string &file, int log_rank) {
      this->log_rank_ = log_rank;
      std::string rank;
      std::string time = GetTime();
      switch (log_rank_) {
        case 0:
          rank = "\033[0;32;32mT-DT LOG | ";
          rank += time;
          rank += " | DEBUG   |";
          break;
        case 1:
          rank = "\033[0;32;32mT-DT LOG | ";
          rank += time;
          rank += " | INFO    |";
          break;
        case 2:
          rank = "\033[1;33mT-DT LOG | ";
          rank += time;
          rank += " | WARNING |";
          break;
        case 3:
          rank = "\033[0;32;31mT-DT LOG | ";
          rank += time;
          rank += " | ERROR   |";
          break;
        case 4:
          rank = "\033[0;32;31m";
          rank += time;
          rank += " | FATAL   |";
          break;
      }

      if (log_rank_ == INFO || log_rank_ == DEBUG) {
        std::cout << rank << " " << file << ":" << line << " | ";
      } else {
        std::cerr << rank << " " << file << ":" << line << " | ";
      }
    }

    ~StreamerNode() {
      if (log_rank_ == INFO || log_rank_ == DEBUG) {
        std::cout << "\033[m" << std::endl << std::flush;
      } else {
        std::cerr << "\033[m" << std::endl << std::flush;
      }
      if (log_rank_ == FATAL) {
        std::terminate();
      }
    }

    template <typename T>
    StreamerNode &operator<<(const T &t) {
      if (log_rank_ == INFO || log_rank_ == DEBUG) {
        std::cout << t << std::flush;
      } else {
        std::cerr << t << std::flush;
      }
      return *this;
    }
  };

 private:
  static char *GetTime();

  Log_Rank log_rank_;  ///< 日志的信息的等级
  char buffer_[1024];
};  /// \brief 写入日志信息之前先写入的源代码文件名, 行号, 函数名

///
/// \brief 根据不同等级进行用不同的输出流进行读写
///

#define TDT_DEBUG(message, ...) \
  Logger(DEBUG).taken(__LINE__, __FILE__, message, ##__VA_ARGS__)
#define TDT_INFO(message, ...) \
  Logger(INFO).taken(__LINE__, __FILE__, message, ##__VA_ARGS__)
#define TDT_WARNING(message, ...) \
  Logger(WARNING).taken(__LINE__, __FILE__, message, ##__VA_ARGS__)
#define TDT_ERROR(message, ...) \
  Logger(ERROR).taken(__LINE__, __FILE__, message, ##__VA_ARGS__)
#define TDT_FATAL(message, ...) \
  Logger(FATAL).taken(__LINE__, __FILE__, message, ##__VA_ARGS__)

#define TDT_DEBUG_() Logger::StreamerNode(__LINE__, __FILE__, DEBUG)
#define TDT_INFO_() Logger::StreamerNode(__LINE__, __FILE__, INFO)
#define TDT_WARNING_() Logger::StreamerNode(__LINE__, __FILE__, WARNING)
#define TDT_ERROR_() Logger::StreamerNode(__LINE__, __FILE__, ERROR)
#define TDT_FATAL_() Logger::StreamerNode(__LINE__, __FILE__, FATAL)

#endif  // LIB_TDTCOMMON_TDTMSG_H