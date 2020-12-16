#ifndef SUBSCRIBER_CALLBACKS_H
#define SUBSCRIBER_CALLBACKS_H

#include "HardwareMessages.h"
#include "Publisher_Indexer.h"
#include "Indexer.h"
#include "Interface.h"
#include <map>

extern std::map < Interface_Indexer_t, Interface * > interfaces;
extern std::map < Device_Indexer_t, Device * > devices;

namespace callbacks {
  /**
   * These functions are used for ROS subsciber callbacks. Their purpose is to pass along the data
   * to a primary callback manager. The only reason why there are seperate is for the seperate
   * index numbers
   */
  void pwm0(board_interface::pwm msg);

  void pwm1(board_interface::pwm msg);

  void pwmCallback(board_interface::pwm &msg, uint8_t index);
}

#endif /* end of include guard: SUBSCRIBER_CALLBACKS_H */
