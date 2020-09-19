/**
 * @Author: Nick Steele <nichlock>
 * @Date:   9:08 Aug 15 2020
 * @Last modified by:   nichlock
 * @Last modified time: 19:25 Sep 19 2020
 */

#include "Hardware/Device_Adc_Mcp3008.h"

// THESE VALUES MUST BE REVIEWED ON CREATION OF EACH NEW DEVICE SUBCLASS
// ***************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
// char HARDWARE_NAME[8] = "MCP3008";
//
// // Informating About The Chip Used
// // ===============================.
// const static uint8_t PIN_COUNT = 8;
// const static Device_t deviceTypeId = DEVICE_ADC;
//
// // Pin Modes That This Chip can Accept
// // ==============================================
// const static uint8_t VALID_PIN_MODE_COUNT = 1;
// const PinMode_t validPinModes[VALID_PIN_MODE_COUNT] = {MODE_INPUT};
//
// // Other Variables (Don't change these)
// // ====================================
// Interface_t reservedPins[PIN_COUNT];
// // For storing tick rate (duty cycle)
// float pinValues[PIN_COUNT];

// These are specific to the MCP3008 on the interfacing board REV A
// const float ADC_STEPS = 1024;
// const float AVCC_THEORETICAL_VALUE = 5.00;

/* These actually drive the chip, and must be different for each device subclass.
 ******************************************************************************/

/**
 * deviceInit is called by Device::init() just before completion. It should at
 * least init the chip and run updateData() once. Note that the uint8_t var
 * 'address' already has the address set by the time this is called.
 * @return whether the init worked.
 */

bool Device_Adc_Mcp3008::deviceInit(){
  // Init chip here

  // Default modes and states assigned here

  // Pin modes never change, and are not in update data function, so they are
  // set here and final
  for (uint8_t i = 0; i < getPinCount(); i++) {
    requestedPinBus.setPinMode(i, MODE_INPUT);
    currentPinBus.setPinMode(i, MODE_INPUT);
  }
  updateData();
  return true;
} /* deviceInit */

/**
 */

DataError_t Device_Adc_Mcp3008::getPinValue(PinValue_t *value){
  if (!(value->pin >= 0 && value->pin < PIN_COUNT))
    return ERROR_DEV_PIN_INVALID;
  if (value->fmt == VALUE_ADC_DIRECT) { // Data from a pin
    value->data[0] = pinValues[value->pin];
    return ERROR_SUCCESS;
  }
  return ERROR_NOT_AVAIL;
} // getPinValue

/**
 * Can't set the pin value on an ADC
 */

DataError_t Device_Adc_Mcp3008::setPinValue(PinValue_t *value) {
  return ERROR_NOT_AVAIL;
} // setPinValue

DataError_t Device_Adc_Mcp3008::writeDeviceConfig(DeviceConfig_t *cfg) {
  return ERROR_NOT_AVAIL;
} // writeDeviceConfig

DataError_t Device_Adc_Mcp3008::readDeviceConfig(DeviceConfig_t *cfg) {
  if (cfg->fmt == DCFG_ADC_STEPS) { // How many steps there are in the ADC measurements
    cfg->data[0] = ADC_STEPS;
    return ERROR_SUCCESS;
  }
  if (cfg->fmt == DCFG_ADC_AVCC_VOLTAGE) { // Voltage of the ADC
    cfg->data[0] = AVCC_THEORETICAL_VALUE;
    return ERROR_SUCCESS;
  }
  return ERROR_NOT_AVAIL;
} // readDeviceConfig

/**
 * Reads data
 */

bool Device_Adc_Mcp3008::updateData(){
  if (!ready())
    return false;
  // Real hardware should be used
  if (!simulate_io) {
    log_info("%s updating (TODO).", HARDWARE_NAME);
    return true;
  }	else { // Only uses simulated hardware, no connections
    for (uint8_t pin = 0; pin < PIN_COUNT; pin++) {
      pinValues[pin] = (0.625 * ADC_STEPS / AVCC_THEORETICAL_VALUE) * (pin + 1);
    }
    // Hacky way of getting correct telemetry data
    if (deviceIndex.index == 2) { // CURRENT 2
      pinValues[0] = 389; // -10 amps
      pinValues[1] = 635; // +10 amps
      pinValues[2] = 614; // 3v
      // pinValues[2] = 635;    // 3.1v
      pinValues[3] = 178; // 25'C
      pinValues[4] = 676; // 3.3v
      pinValues[5] = 1024; // 5v
      pinValues[6] = 768; // 12v ==> 3.75v with resitor divider
      pinValues[7] = 819; // 48v ==> 4v with resitor divider
    }
    return true;
  }
} // updateData
