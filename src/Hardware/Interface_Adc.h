/**
 * @Author: Nick Steele <nichlock>
 * @Date:   9:08 Aug 15 2020
 * @Last modified by:   nichlock
 * @Last modified time: 19:25 Sep 19 2020
 */

#ifndef INTERFACE_ADC_H
#define INTERFACE_ADC_H

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

/**
 * Interface_Adc
 * @author
 */
class Interface_Adc : public Interface {
private:
// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
const static uint8_t PIN_COUNT = 8;
// IDs which indicate what this is and what it should be connected to
const static Interface_t interfaceTypeId = INTF_ADC;
const static Device_t parentDeviceTypeId = DEVICE_ADC;

// Calibration ratios, preset to a default offset and a tolerance +- 100%
float avccOffsetRatio = 1; // multiply by measured to get actual.
float avccOffsetToleranceRatio = 1; // multiply by actual to get ± tolerance value
// For data conversions
float adcSteps = 1024; // number of steps the ADC uses to save data (ie reading at AVCC)
float avccTheoretical = 5.00; // Theoretical AVCC

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

DataError_t writePin(PinValue_t *value);

DataError_t writeConfig(InterfaceConfig_t *cfg);

DataError_t readConfig(InterfaceConfig_t *cfg);

DataError_t writeDeviceConfig(DeviceConfig_t *cfg);

DataError_t readDeviceConfig(DeviceConfig_t *cfg);

/**
 * @param pinNumber TODO
 * @param pinMode TODO
 * @param hd TODO
 * @return TODO
 */

uint8_t setPinMode(uint8_t pinNumber, PinMode_t pinMode);
}

;

#endif /* end of include guard: INTERFACE_ADC_H */
