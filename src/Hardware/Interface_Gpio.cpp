/**
 * @Author: Nick Steele <nichlock>
 * @Date:   16:38 Aug 12 2020
 * @Last modified by:   Nick Steele
 * @Last modified time: 17:59 Feb 13 2021
 */

#include "Interface_Gpio.h"

// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
// const static uint8_t PIN_COUNT = 16; // number of pins
// const static Interface_t interfaceTypeId = INTF_GPIO; // The ID for this intf
// const static Device_t parentDeviceTypeId = DEVICE_GPIO; // The IF for the device
// ----------------------------------------------------------------------------

/* These must be changed per interface to ensure operability.
 *****************************************************************************/

/** Called at init. Should assign default modes to the pinBus object.
 * updateData() will be called after this, so there's no needto call it here.
 */

void Interface_Gpio::prepareInterface(){
  pinBus.setAllPins(MODE_INPUT);
  commDevice->setPinModes(pinBus);
} // prepareInterface

DataError_t Interface_Gpio::readPin(PinValue_t *valueIn) {
  if (!(valueIn->pin >= 0 && valueIn->pin < PIN_COUNT))
    return ERROR_INTF_PIN_INVALID;

  PinValue_t val;
  DataError_t errorVal;
  // Reads the pin on the device. Formatting/scaling/other data changes happen below.
  val.fmt = VALUE_GPIO_STATE; // Set format
  val.pin = pinBus.getPin(valueIn->pin); // Go from local pin to the device pin
  val.data = valueIn->data; // Uses input data to store data
  errorVal = commDevice->getPinValue(&val); // Get the data

  // Format data and return with the error/success code from device
  switch (valueIn->fmt) {
  case VALUE_GPIO_STATE:
    return errorVal;
    break;
  case VALUE_ROS_DATA_: // Data format for dumping data over ROS messages
    return commDevice->getPinValue(&val);
    break;
  default:
    return ERROR_NOT_AVAIL;
  } // switch
} // readPin

DataError_t Interface_Gpio::writePin(PinValue_t *valueIn) {
  if (!(valueIn->pin >= 0 && valueIn->pin < PIN_COUNT))
    return ERROR_INTF_PIN_INVALID;

  PinValue_t val;
  DataError_t errorVal; // not used
  // Reads the pin on the device. Formatting/scaling/other data changes happen below.
  val.pin = pinBus.getPin(valueIn->pin); // Go from local pin to the device pin
  val.data = valueIn->data;

  if (!(commDevice->ready()))
    return ERROR_DEV_N_READY;

  switch (valueIn->fmt) {
  case VALUE_GPIO_STATE:
    val.fmt = VALUE_GPIO_STATE; // Set format
    return commDevice->setPinValue(&val); // Set the data
    break;
  case VALUE_GPIO_PUPD:
    val.fmt = VALUE_GPIO_PUPD;
    return commDevice->setPinValue(&val);
    break;
  case VALUE_GPIO_MODE:
    val.fmt = VALUE_GPIO_MODE;
    commDevice->setPinMode(val.pin,
                           (val.data[0] > 0.5)?MODE_OUTPUT:MODE_INPUT);
    return ERROR_SUCCESS;
    break;
  default:
    return ERROR_NOT_AVAIL;
    break;
  } // switch
} // writePin

DataError_t Interface_Gpio::writeConfig(InterfaceConfig_t *cfg) {
  return ERROR_NOT_AVAIL;
} // writeConfig

DataError_t Interface_Gpio::readConfig(InterfaceConfig_t *cfg) {
  return ERROR_NOT_AVAIL;
} // readConfig

DataError_t Interface_Gpio::writeDeviceConfig(DeviceConfig_t *cfg) {
  return ERROR_NOT_AVAIL;
} // writeDeviceConfig

DataError_t Interface_Gpio::readDeviceConfig(DeviceConfig_t *cfg) {
  return ERROR_NOT_AVAIL;
} // readDeviceConfig
