/**
 * @Author: Nick Steele <nichlock>
 * @Date:   10:54 Aug 15 2020
 * @Last modified by:   nichlock
 * @Last modified time: 19:19 Sep 19 2020
 */

#ifndef BIT_TESTING_H
#define BIT_TESTING_H

#include <stdint.h>
#include <stdio.h>
#include <ros/ros.h>
#include "Logger.h"
#include "HardwareMessages.h"
#include "ConsoleColors.h"

namespace test {
bool pwm(uint8_t index, bool direction);

bool gpio(uint8_t index);

bool power(uint8_t index);

bool leak(uint8_t index);

bool emergencyIo(uint8_t index);

bool led(uint8_t index);

bool adc(uint8_t index);
} // namespace test

#endif /* end of include guard: BIT_TESTING_H */
