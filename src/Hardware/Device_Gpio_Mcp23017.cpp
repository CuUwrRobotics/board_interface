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
// 	                                                     MODE_OUTPUT};
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
	// Init chip here

	// Default modes and states assigned here

	pinModeChangePending = true;
	writeDataPending = true;
	updateData();
	return true;
} /* deviceInit */

DataError_t Device_Gpio_Mcp23017::getPinValue(PinValue_t *value){
	if (!(value->pin >= 0 && value->pin < PIN_COUNT))
		return ERROR_DEV_PIN_INVALID;
	if (value->fmt == VALUE_GPIO_STATE) {
		// Gets the on/off bit and returns as a 1 or 0.
		// value->data[0] = ((uint16_t)((currentPinValues >> pin) & 0x01) !=
		//                   (uint16_t)0) ? 1 : 0;
		value->data[0] = ((currentPinValues >> value->pin) & 0x01);
		return ERROR_SUCCESS;
	}
	return ERROR_NOT_AVAIL;
} // getPinValue

DataError_t Device_Gpio_Mcp23017::setPinValue(PinValue_t *value) {
	// If pin is not writable, don't set it.
	if (pinIsReadable(value->pin))
		return ERROR_WROTE_INPUT;
	if (!(value->pin >= 0 && value->pin < PIN_COUNT))
		return ERROR_DEV_PIN_INVALID;

	if (value->fmt == VALUE_GPIO_STATE) {
		if (value->data[0]) // set bit
			requestedPinValues |= (0x0001 << value->pin);
		else // clear bit
			requestedPinValues &= ~(0x0001 << value->pin);
		writeDataPending = true;
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

bool Device_Gpio_Mcp23017::updateData(){
	if (!ready())
		return false;

	if (!simulate_io) {
		log_info("%s updating (TODO).", HARDWARE_NAME);
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
