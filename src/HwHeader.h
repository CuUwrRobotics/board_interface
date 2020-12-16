/**
 * @Author: Nick Steele <nichlock>
 * @Date:   11:31 Aug 16 2020
 * @Last modified by:   Nick Steele
 * @Last modified time: 12:42 Dec 16 2020
 */

#ifndef HW_HEADER
#define HW_HEADER

using namespace std;

#include <stdint.h>
#include <stdio.h>
#include <signal.h>
#include <ros/ros.h>
#include "HardwareDescription.h"
#include "HardwareData.h"
#include "Logger.h"
#include "ConsoleColors.h"
#include "AllDevicesInterfaces.h"
#include "PinBus.h"
#include "BitTesting.h"
// #include "watchdog/pet_dog_msg.h"
#include <map>
#include <utility> // TODO: do we need this?
#include "Indexer.h"
#include "HardwareMessages.h"
#include "Publisher_Indexer.h"
#include "config.h"
#include "Subscriber_Callbacks.h"

// If incompatible enums are compared, an error will be thrown instead of a warning.
#pragma GCC diagnostic error "-Wenum-compare"

#define ROS_MESSAGE_DELAY_TIME 0.5

#endif /* ifndef HW_HEADER */
