/**
 * @Author: Nick Steele <nichlock>
 * @Date:   9:08 Aug 15 2020
 * @Last modified by:   nichlock
 * @Last modified time: 19:23 Sep 19 2020
 */

#ifndef BIT_TESTING_H
#define BIT_TESTING_H

#include <stdint.h>
#include <stdio.h>
#include <ros/ros.h>
#include "HardwareDescription.h"
#include "PinBus.h"
#include "HardwareData.h"
#include "Device.h"
#include "Interface.h"
#include "ConsoleColors.h"
#include "Indexer.h"
#include <map> // Mot used here, but necesary for the maps
#include <utility> // TODO: do we need this?

namespace bit_testing {
/**
 * @param intf TODO
 * @param dev TODO
 * @param direction TODO
 * @return TODO
 */

bool testPwm(Interface *intf, Device *dev, bool direction);

/**
 * @param intf TODO
 * @param dev TODO
 * @param direction TODO
 * @return TODO
 */

bool testGpio(Interface *intf, Device *dev);

/**
 * @param intf TODO
 * @param dev TODO
 * @param direction TODO
 * @return TODO
 */

bool testPower(Interface *intf, Device *dev);

/**
 * @param intf TODO
 * @param dev TODO
 * @param direction TODO
 * @return TODO
 */

bool testLeak(Interface *intf, Device *dev);

/**
 * @param intf TODO
 * @param dev TODO
 * @return TODO
 */

bool testEmergencyIo(Interface *intf, Device *dev);

/**
 * @param intf TODO
 * @param dev TODO
 * @return TODO
 */

bool testLed(Interface *intf, Device *dev);

/**
 * @param intf TODO
 * @param dev TODO
 * @return TODO
 */

bool testAdc(Interface *intf, Device *dev);

/**
 * Fancy display of all devices, interfaces, and pins.
 * TODO: At end, cycle through and see if any interfaces are unconnected.
 */

// void dumpConfiguration(bool shrinkRepeatedPins,
//                        std::map <Interface_Indexer_t, Interface *> *interfaces,
//                        std::map <Device_Indexer_t, Device *> *devices);
} // namespace bit_testing

#endif /* ifndef BIT_TESTING_H */
