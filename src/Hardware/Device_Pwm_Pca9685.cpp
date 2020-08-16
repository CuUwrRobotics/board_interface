#include "Device_Pwm_Pca9685.h"

// THESE VALUES MUST BE REVIEWED ON CREATION OF EACH NEW DEVICE SUBCLASS
// ***************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
// char HARDWARE_NAME[8] = "PCA9685";
//
// // Informating About The Chip Used
// // ===============================.
// const static uint8_t PIN_COUNT = 16;
// const static Device_t deviceTypeId = DEVICE_PWM;
//
// // Pin Modes That This Chip can Accept
// // ==============================================
// const static uint8_t VALID_PIN_MODE_COUNT = 1;
// const PinMode_t validPinModes[VALID_PIN_MODE_COUNT] = {MODE_OUTPUT};
//
// // Other Variables (Don't change these)
// // ====================================
// Interface_t reservedPins[PIN_COUNT];
// // For storing tick rate (duty cycle)
// float currentPinTicks[PIN_COUNT];
// float requestedPinTicks[PIN_COUNT];
// // For storing device frequncy
// float currentFrequencyValue;
// float requestedFrequencyValue;

/* These actually drive the chip, and must be different for each device subclass.
 ******************************************************************************/

/**
 * deviceInit is called by Device::init() just before completion. It should at
 * least init the chip and run updateData() once. Note that the uint8_t var
 * 'address' already has the address set by the time this is called.
 * @return whether the init worked.
 */

bool Device_Pwm_Pca9685::deviceInit(){
	// Init chip here

	// Default modes and states assigned here

	pinModeChangePending = true;
	writeDataPending = true;
	updateData();
	return true;
} /* deviceInit */

DataError_t Device_Pwm_Pca9685::getPinValue(PinValue_t *value){
	switch (value->fmt) {
	case VALUE_ROS_DATA_: // Data format for dumping data over ROS messages
		// TODO: priority
		// Stores all pins and base frequency into the array. This requires an array
		// of length PIN_COUNT + 1.
		// This is the only time when it is appropriate to have a conversion in the
		// device! It's only here for efficiency
		for (uint8_t pin = 0; pin < PIN_COUNT; pin++) {
			value->data[pin] = (currentPinTicks[value->pin] / (MAX_PWM_TICKS / 100));
		}
		value->data[16] = currentFrequencyValue;
		return ERROR_SUCCESS;
		break;
	case VALUE_PWM_FREQ:
		value->data[0] = currentFrequencyValue;
		return ERROR_SUCCESS;
		break;
	case VALUE_PWM_ON_TICKS:
		value->data[0] = currentPinTicks[value->pin];
		return ERROR_SUCCESS;
		break;
	default:
		return ERROR_NOT_AVAIL;
		break;
	} // switch
} // getPinValue

/**
 * Note that all data handling (min/max vals) happens in interfaces
 * @param pin TODO
 * @param struct PinDataPin data which MUST contain the PWM on time ticks.
 * @param interfaceId TODO
 * @return TODO
 */

DataError_t Device_Pwm_Pca9685::setPinValue(PinValue_t *value) {
	if (!(value->pin >= 0 && value->pin < PIN_COUNT))
		return ERROR_DEV_PIN_INVALID;
	// Don't flag for a data write if no changes are made.
	if (value->fmt == VALUE_PWM_FREQ) {
		if (value->data[0] == currentFrequencyValue) {
			return ERROR_SUCCESS;
		}	else {
			requestedFrequencyValue = value->data[0];
			writeDataPending = true;
			return ERROR_SUCCESS;
		}
	} else if (value->fmt == VALUE_PWM_ON_TICKS) {
		if (value->data[0] == requestedPinTicks[value->pin]) {
			return ERROR_SUCCESS;
		}	else {
			requestedPinTicks[value->pin] = value->data[0];
			writeDataPending = true;
			return ERROR_SUCCESS;
		}
	}
	return ERROR_NOT_AVAIL;
} // setPinValue

DataError_t Device_Pwm_Pca9685::writeDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // writeDeviceConfig

DataError_t Device_Pwm_Pca9685::readDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // readDeviceConfig

/**
 * Reads/writes any data related to the chip. This function will interface
 * directly with the chip's hardware interface, and is the ONLY way that any code
 * can read from or write to the chip at 'address'.
 * Note that the order this happens in is important.
 * @return if update succeeded
 */

bool Device_Pwm_Pca9685::updateData(){
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
		// Check if any data needs to be written. If so, write it.
		if (writeDataPending) {
			float pinValuesToSend[PIN_COUNT] = {0};
			// For each pin, write the value over.
			for (uint8_t pin = 0; pin < PIN_COUNT; pin++) { // For each pin
				pinValuesToSend[pin] = requestedPinTicks[pin]; // Pin is set up for write, so
				currentPinTicks[pin] = requestedPinTicks[pin];
				currentFrequencyValue = requestedFrequencyValue;
			}
			writeDataPending = false;
		}
		return true;
	}
} // updateData
