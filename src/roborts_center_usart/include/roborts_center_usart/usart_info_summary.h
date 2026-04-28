#ifndef __USART_INFO_SUMMARY_H
#define __USART_INFO_SUMMARY_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <boost/asio.hpp>
#include <deque>
#include <mutex>
#include <vector>

#include "recver/match_info_usart_recver.h"
#include "recver/sentry_usart_recver.h"
#include "recver/gimbal_usart_recver.h"
#include "sender/decision_usart_sender.h"
#include "sender/message_usart_sender.h"
#include "sender/radar_usart_sender.h"
#include "sender/gimbal_usart_sender.h"
#include "sender/radio_usart_sender.h"
#include "sender/key_usart_sender.h"

namespace tdtusart {
namespace shared_data {
static const int DataRecverNum = 3;  //   command , vision, matchInfo
static BaseUsartRecver *DataRecver[DataRecverNum + 1] = {
    nullptr, 
    (BaseUsartRecver *)(new MatchInfoUsartRecver()),
    (BaseUsartRecver *)(new SentryUsartRecver()),
    (BaseUsartRecver *)(new GimbalUsartRecver()),
    // (BaseUsartRecver *)(new (VisionUsartRecver))
    };
    
static const int DataSenderNum = 6;  // vision, commandReply
static BaseUsartSender *DataSender[DataSenderNum + 1] = {
    nullptr, 
    (BaseUsartSender *)(new DecisionUsartSender()),
    (BaseUsartSender *)(new MessageUsartSender()),
    (BaseUsartSender *)(new RadarUsartSender()),
    (BaseUsartSender *)(new GimbalUsartSender()),
    (BaseUsartSender *)(new KeyUsartSender()),
    (BaseUsartSender *)(new RadioUsartSender())};

}  // namespace shared_data
}  // namespace tdtusart
#endif