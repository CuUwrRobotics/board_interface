/**
 * @Author: Nick Steele <nichlock>
 * @Date:   16:38 Aug 12 2020
 * @Last modified by:   nichlock
 * @Last modified time: 19:24 Sep 19 2020
 */

#include "Interface_Leak.h"
// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
// const static uint8_t PIN_COUNT = 8; // number of pins
// const static Interface_t interfaceTypeId = INTF_LEAK; // The ID for this intf
// const static Device_t parentDeviceTypeId = DEVICE_GPIO; // The IF for the device
// ----------------------------------------------------------------------------

/* These must be changed per interface to ensure operability.
 *****************************************************************************/

/** Called at init. Should assign default modes to the pinBus object.
 * updateData() will be called after this, so there's no needto call it here.
 */

void Interface_Leak::prepareInterface(){
  pinBus.setAllPins(MODE_INPUT);
  commDevice->setPinModes(pinBus);
} // prepareInterface

DataError_t Interface_Leak::readPin(PinValue_t *valueIn) {
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
  case VALUE_ROS_DATA_: // Data format for dumping data over ROS messages
  { // For scoping of variables create in here
    errorVal = commDevice->getPinValue(&val); // TODO: this should only have limited pins!
    uint16_t pinValues = (uint16_t)valueIn->data[0]; // Get pin values
    uint16_t interfacePinValues; // Will have pin values written into it
    // Changes data for return to only have this interfaces pins
    for (uint8_t pin = 0; pin < PIN_COUNT; pin++) {
      // If GPIO pin is high, set the bit. If low, clear the bit
      if (((pinValues << pinBus.getPin(pin)) & 0x01)) {
        interfacePinValues |= (0x01 << pin); // set bit
      } else {
        interfacePinValues &= ~(0x01 << pin); // clear bit
      }
    }
    return errorVal;
    break;
  }
  case VALUE_GPIO_STATE:
    return errorVal;
    break;
  default:
    return ERROR_NOT_AVAIL;
  } // switch
} // readPin

DataError_t Interface_Leak::writePin(PinValue_t *valueIn) {
  return ERROR_NOT_AVAIL;
} // writePin

DataError_t Interface_Leak::writeConfig(InterfaceConfig_t *cfg) {
  return ERROR_NOT_AVAIL;
} // writeConfig

DataError_t Interface_Leak::readConfig(InterfaceConfig_t *cfg) {
  return ERROR_NOT_AVAIL;
} // readConfig

DataError_t Interface_Leak::writeDeviceConfig(DeviceConfig_t *cfg) {
  return ERROR_NOT_AVAIL;
} // writeDeviceConfig

DataError_t Interface_Leak::readDeviceConfig(DeviceConfig_t *cfg) {
  return ERROR_NOT_AVAIL;
} // readDeviceConfig

uint8_t Interface_Leak::setPinMode(uint8_t pinNumber, PinMode_t pinMode){
  ROS_INFO("setPinMode: Data cannot be written to the %s interface!",
           interfaceIdToCharArray(interfaceTypeId));
  return 0;
} /* setPinMode */
