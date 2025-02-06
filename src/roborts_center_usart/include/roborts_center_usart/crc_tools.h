#ifndef __CRC_TOOLS_H
#define __CRC_TOOLS_H
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <deque>
#include <mutex>
#include <vector>
namespace tdtusart {
class CRC {
 private:
  /**带CRC为校验所用，非特殊情况禁止修改**/
  static uint16_t CRC_INIT;
  static uint16_t kCRCTable[256];

 public:
  static uint16_t GetCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength,
                                   uint16_t wCRC);
  static uint32_t VerifyCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);
  static void AppendCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);
};
}  // namespace tdtusart

#endif