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
 * to a primary callback manager. The reason there are seperate functions is for the seperate
 * index numbers
 * These are ONLY wrappers for the callbacks below.
 */
void pwm0(board_interface::pwm msg);

void pwm1(board_interface::pwm msg);

void gpio0(board_interface::gpio msg);

void gpio1(board_interface::gpio msg);

void gpio2(board_interface::gpio msg);

/**
 * These function ACTUALLY handle the callback data.
 */
void pwmCallback(board_interface::pwm &msg, uint8_t index);

void gpioCallback(board_interface::gpio &msg, uint8_t index);
} // namespace callbacks

#endif /* end of include guard: SUBSCRIBER_CALLBACKS_H */
