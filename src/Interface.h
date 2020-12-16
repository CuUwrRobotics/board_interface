/**
 * @Author: Nick Steele <nichlock>
 * @Date:   9:08 Aug 15 2020
 * @Last modified by:   Nick Steele
 * @Last modified time: 12:56 Dec 16 2020
 */

/* Interface.cpp
 * Abstract class which is used to implement any interface. Interfaces are not directly controlled
 * by Raspberry
 * Pi's digital lines, while devices are. For example, the Power Mosfets are driven by a GPIO chip,
 * which is then
 * driven by the RPi, therfore indirect control.
 */

#ifndef INTERFACE_H
#define INTERFACE_H

#include <stdint.h>
#include <stdio.h>
#include <ros/ros.h>
#include "HardwareDescription.h"
#include "PinBus.h"
#include "HardwareData.h"
#include "Device.h"
#include "Indexer.h"

/**
 * Interface
 * @author Nicholas Steele
 */
class Interface {
private:
public:

Device *commDevice; // The device to actually communicate using
PinBus pinBus;

bool initerrorVal = false;
bool commDeviceExists = false;
Interface_Indexer_t interfaceIndex; // Index of the interface object

// Must be overwritten by subclasses
// ****************************************************************************

/**
 * @return DataError_t error usable with errorCharArray.
 */
virtual DataError_t writePin(PinValue_t *valueIn) = 0;

/**
 * @return DataError_t error usable with errorCharArray.
 */
virtual DataError_t readPin(PinValue_t *valueIn) = 0;

/**
 * @return DataError_t error usable with errorCharArray.
 */
virtual DataError_t writeConfig(InterfaceConfig_t *cfg) = 0;

/**
 * @return DataError_t error usable with errorCharArray.
 */
virtual DataError_t readConfig(InterfaceConfig_t *cfg) = 0;

/**
 * @return DataError_t error usable with errorCharArray.
 */
virtual DataError_t writeDeviceConfig(DeviceConfig_t *cfg) = 0;

/**
 * @return DataError_t error usable with errorCharArray.
 */
virtual DataError_t readDeviceConfig(DeviceConfig_t *cfg) = 0;

virtual void prepareInterface() = 0;

virtual uint8_t getPinCount() = 0;

virtual Interface_t getInterfaceTypeId() = 0;

virtual Device_t getParentTypeId() = 0;

// Regular fuctions
// ****************************************************************************

/** Initialize the interface.
 * @param device Pointer for Device object that the interface communicates with. MUST already be
 * initialized
 * @param devicePins An array of the pin numbers to be used on the given device.
 * @param hd The full hardware descriptor.
 * @return true of sucessful
 */
bool start(Device *device, PinBus pb, Interface_Indexer_t ifaceIndex); /* start */

/*
 * Some interfaces cannot have pin modes changed. They need to override this.
 * @param pinNumber The pin nunmber to try to drive
 * @param hd The full hardware descriptor.
 * @return 1 if there are no errors.
 */

virtual inline uint8_t setPinMode(uint8_t pinNumber, PinMode_t pinMode){
  // Try to drive a pin in commDevice
  return commDevice->setPinMode(pinBus.getPin(pinNumber), pinMode);
}; /* setPinMode */

/*
 * @return The parent device's index.
 */

inline Device_Indexer_t getParentDeviceIndex(){
  if (!commDeviceExists)
    return Device_Indexer_t(DEVICE_INVALID_, 0);
  return commDevice->getDeviceIndex();
} // getParentDeviceIndex

/*
 * @return This devices index.
 */

inline Interface_Indexer_t getInterfaceIndex(){
  return interfaceIndex;
} // getParentDeviceIndex

/**
 * @return TODO
 */

inline PinMode_t getPinMode(uint8_t pin){
  return commDevice->getPinMode(pinBus.getPin(pin));
} // getPinMode

/**
 * @return TODO
 */

inline bool ready(){
  return initerrorVal;
} // ready

/**
 * @return TODO
 */

inline uint8_t getMappedDevPin(uint8_t pin){
  return pinBus.getPin(pin);
} // getMappedPin

/**
 * @return TODO
 */

inline PinBus getPinBus() {
  return pinBus;
} // getPinBus

/**
 * @return The subclass's interface name.
 */

inline const char *getInterfaceName() {
  return interfaceIdToCharArray(getInterfaceTypeId());
} // getInterfaceName
}

;

#endif /* end of include guard: INTERFACE_H */
