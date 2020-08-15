#ifndef INTERFACE_PWM_H
#define INTERFACE_PWM_H

// Generic headers
#include <stdint.h>
#include <stdio.h>
#include <ros/ros.h>
#include "HardwareDescription.h"
#include "PinBus.h"
#include "HardwareData.h"
// #include "Device.h"
#include "Interface.h"
#include "Logger.h"

#include "Device_Pwm_Pca9685.h"

/**
 * Interface_Pwm
 * @author Nicholas Steele
 */
class Interface_Pwm : public Interface {
private:
// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
const static uint8_t PIN_COUNT = 16; // number of pins
const static Interface_t interfaceTypeId = INTF_PWM; // The ID for this intf
const static Device_t parentDeviceTypeId = DEVICE_PWM; // The IF for the device
// ----------------------------------------------------------------------------

// uint8_t pinMapToDevice[PIN_COUNT]; // Contains a map to the devices pins.
// uint8_t pinModes[PIN_COUNT]; // Contains a map to the devices pins.
// interface based on HD
// class values
public:

/* Don't change these; they allow the base class to access locally assigned
 * variables.
 *****************************************************************************/

//
inline Interface_t getInterfaceTypeId(){
	return interfaceTypeId;
} // getInterfaceTypeId

//
inline Device_t getParentTypeId(){
	return parentDeviceTypeId;
} // getParentTypeId

//
inline uint8_t getPinCount(){
	return PIN_COUNT;
} // getPinCount

/* These must be changed per interface to ensure operability.
 *****************************************************************************/

/** Called at init. Should assign default modes to the pinBus object.
 * updateData() will be called after this, so there's no needto call it here.
 */

void prepareInterface();

DataError_t readPin(PinValue_t *valueIn);

DataError_t writePin(PinValue_t *valueIn);

DataError_t writeConfig(InterfaceConfig_t *cfg);

DataError_t readConfig(InterfaceConfig_t *cfg);

DataError_t writeDeviceConfig(DeviceConfig_t *cfg);

DataError_t readDeviceConfig(DeviceConfig_t *cfg);

// **** OVERRIDES BASE CLASS ****
uint8_t setPinMode(uint8_t pinNumber, PinMode_t pinMode);
}

;

#endif /* end of include guard: INTERFACE_PWM_H */
