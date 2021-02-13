/**
 * @Author: Nick Steele <nichlock>
 * @Date:   9:08 Aug 15 2020
 * @Last modified by:   Nick Steele
 * @Last modified time: 18:22 Feb 13 2021
 */

/* This class is a tempalte for copy/pasting to actual class files. It is
 * intened tospeed up making new devices.
 */
#include "Device_Gpio_Mcp23017.h"

// THESE VALUES MUST BE REVIEWED ON CREATION OF EACH NEW DEVICE SUBCLASS
// ***************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
// char HARDWARE_NAME[9] = "MCP23017";
//
// // Informating About The Chip Used
// // ===============================.
// const static uint8_t PIN_COUNT = 16;
// const static Device_t deviceTypeId = DEVICE_GPIO;
//
// // Pin Modes That This Chip can Accept
// // ==============================================
// const static uint8_t VALID_PIN_MODE_COUNT = 2;
// const PinMode_t validPinModes[VALID_PIN_MODE_COUNT] = {MODE_INPUT,
//                                                       MODE_OUTPUT};
//
// // Other Variables (Don't change these)
// // ====================================
// Interface_t reservedPins[PIN_COUNT];
// // No pin values for GPIO, only HIGH/LOW, so each bit is one pin.
// uint16_t currentPinValues;
// uint16_t requestedPinValues;

/* These actually drive the chip, and must be different for each device subclass.
 ******************************************************************************/

/**
 * deviceInit is called by Device::init() just before completion. It should at
 * least init the chip and run updateData() once. Note that the uint8_t var
 * 'address' already has the address set by the time this is called.
 * @return whether the init worked.
 */

bool Device_Gpio_Mcp23017::deviceInit(){
  Wire.begin();

  writeTwoRegisters(0x0000, MCP23017_IODIRA); // Output
  writeTwoRegisters(0x0000, MCP23017_GPIOA); // all off
  writeTwoRegisters(0xFFFF, MCP23017_IODIRA); // Input
  writeTwoRegisters(0x0000, MCP23017_GPPUA); // no pull up

  pinModeChangePending = true;
  writeDataPending = true;
  updateData();
  return true;
} /* deviceInit */

DataError_t Device_Gpio_Mcp23017::getPinValue(PinValue_t *value){
  if (!(value->pin >= 0 && value->pin < PIN_COUNT))
    return ERROR_DEV_PIN_INVALID;
  switch (value->fmt) {
  case VALUE_GPIO_STATE:
    // Gets the on/off bit and returns as a 1 or 0.
    value->data[0] = ((currentPinValues >> value->pin) & 0x01);
    return ERROR_SUCCESS;
    break;
  case VALUE_ROS_DATA_: // Data format for dumping data over ROS messages
    // Because data tranfrers happen with floats, and floats cannot be manipulated
    // with hbit-shifting, we need to store bits into uinsigned integers, then
    // convert those to floats. Thos floats must be converted back into unsinged
    // 16-bit integers after. This relies on the hope that floats will be precise
    // enough for these values to keep at least the ones place intact! If they are
    // not, doubles will be needed, but floats should be enough as we are only
    // dealing with 'small' values (up to 2^17 - 1)
  { // Needed to scope variables created in this case statement
    PinMode_t mode = MODE_INVALID; // For packing modes into data
    uint16_t modes; // Stores mode bits temporarily before storing into a float
    // Put all data into value
    value->data[0] = (float)currentPinValues;
// Get modes and pack into bit array
    for (uint8_t pin = 0; pin < PIN_COUNT; pin++) {
      mode = currentPinBus.getPinMode(pin);
      if (mode == MODE_INPUT) {
        modes |= 0x01 << pin; // Set bit
      } else modes &= ~(0x01 << pin); // Clear bit
    }
    value->data[1] = (float)modes; // Store modes into the float
    return ERROR_SUCCESS;
    break;
  }
  default:
    return ERROR_NOT_AVAIL;
    break;
  } // switch
} // getPinValue

DataError_t Device_Gpio_Mcp23017::setPinValue(PinValue_t *value) {
  // If pin is not writable, don't set it.
  // if (pinIsReadable(value->pin))
  //   return ERROR_WROTE_INPUT;
  if (!(value->pin >= 0 && value->pin < PIN_COUNT))
    return ERROR_DEV_PIN_INVALID;

  if (value->fmt == VALUE_GPIO_STATE) {
    if (value->data[0]) // set bit
      requestedPinValues |= (0x0001 << value->pin);
    else // clear bit
      requestedPinValues &= ~(0x0001 << value->pin);
    writeDataPending = true;
    return ERROR_SUCCESS;
  } else if (value->fmt == VALUE_GPIO_PUPD) {
    if (value->data[0] == 0) // No PU or PD
      pupd |= (0x0001 << value->pin);
    else if (value->data[0] == 1) // Pull up
      pupd &= ~(0x0001 << value->pin);
    else
      log_error("Invalid pull-up/pull-down value recieved: %d\n",
                (uint8_t)value->data[0]);
    pinModeChangePending = true;
    return ERROR_SUCCESS;
  }
  return ERROR_NOT_AVAIL;
} // setPinValue

DataError_t Device_Gpio_Mcp23017::writeDeviceConfig(DeviceConfig_t *cfg) {
  return ERROR_NOT_AVAIL;
} // writeDeviceConfig

DataError_t Device_Gpio_Mcp23017::readDeviceConfig(DeviceConfig_t *cfg) {
  return ERROR_NOT_AVAIL;
} // readDeviceConfig

void Device_Gpio_Mcp23017::writeTwoRegisters(uint16_t two_bytes, uint8_t
                                             first_register){
  Wire.beginTransmission(this->address);
  Wire.write(first_register);
  Wire.write((uint8_t)(two_bytes & 0xFF));
  Wire.write((uint8_t)(two_bytes >> 8));
  Wire.endTransmission();
} // Device_Gpio_Mcp23017::writeTwoRegisters

bool Device_Gpio_Mcp23017::updateData(){
  if (!ready())
    return false;

  if (!simulate_io) {
    if (pinModeChangePending) {
      uint16_t modes = 0;
      for (uint8_t pin = 0; pin < PIN_COUNT; pin++) {
        modes |= (requestedPinBus.getPinMode(pin) == MODE_INPUT) << pin;
        currentPinBus.setPinMode(pin, requestedPinBus.getPinMode(pin));
      }
      writeTwoRegisters(modes, MCP23017_IODIRA);
      writeTwoRegisters(pupd, MCP23017_GPPUA);
      pinModeChangePending = false;
    }

    if (writeDataPending) {
      writeTwoRegisters(requestedPinValues, MCP23017_GPIOA);
      writeDataPending = false;
      // currentPinValues will be updated in the read later
    }

    { // Read data
      uint16_t ba = 0;
      uint8_t a;
      // read the current GPIO output latches
      Wire.beginTransmission(this->address);
      Wire.write((uint8_t)MCP23017_GPIOA);
      Wire.endTransmission();
      Wire.requestFrom(this->address, 2);
      a = Wire.read();
      ba = Wire.read();
      ba <<= 8;
      ba |= a;
      currentPinValues = ba; // Accounts for currently high output pins
    }
    return true;
  } else {
    // Check if any pins need their modes changed
    if (pinModeChangePending) {
      for (uint8_t pin = 0; pin < PIN_COUNT; pin++) {
        currentPinBus.setPinMode(pin, requestedPinBus.getPinMode(pin));
      }
      pinModeChangePending = false;
    }
    // Check if any data is readable on any pins. If so, read it.
    if (readableDataAvailable()) {
      for (uint8_t pin = 0; pin < PIN_COUNT; pin++) {
        if (pinIsReadable(pin)) { // If pin should be read
          // currentPinValues[pin] = 0; // Just set to zero
        }
      }
    }
    // Check if any data needs to be written. If so, write it.
    if (writeDataPending) {
      currentPinValues = requestedPinValues; // Just pushes data over for now
      writeDataPending = false;
    }
    return true;
  }
} // updateData
