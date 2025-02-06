#include "base_msg.h"

#include <string.h>
#include <zconf.h>

#include <cstdlib>
#include <ctime>

using namespace std;

char *Logger::GetTime() {
  time_t tm;
  time(&tm);
  static char time_string[128];
  ctime_r(&tm, time_string);
  time_string[strlen(time_string) - 1] = '\0';
  return time_string;
}

void Logger::taken(const int line, const std::string &file,
                   char const *msg_format, ...) {
  string rank;
  string time = GetTime();
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

  va_list aptr;
  int ret;

  va_start(aptr, msg_format);
  ret = vsprintf(buffer_, msg_format, aptr);
  va_end(aptr);

  if (log_rank_ == INFO || log_rank_ == DEBUG) {
    cout << rank << " " << file << ":" << line << " | " << buffer_ << "\033[m"
         << endl
         << flush;
  } else {
    cerr << rank << " " << file << ":" << line << " | " << buffer_ << "\033[m"
         << endl
         << flush;
  }
  if (log_rank_ == FATAL) {
    throw MyException();
  }
}

Logger::~Logger() {
  if (FATAL == log_rank_) {
    // abort();
    ;
  }
}