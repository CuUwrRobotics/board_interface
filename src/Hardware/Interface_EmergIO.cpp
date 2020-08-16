#include "Interface_EmergIO.h"

// SET THESE FOR ANY NEW INTERFACE
// ****************************************************************************
// Information for Interacting with Other Code
// ===========================================
// Number of pins to be assigned to the parent device. Max = parent device max pins
// const static uint8_t PIN_COUNT = 3; // number of pins
// const static Interface_t interfaceTypeId = INTF_EMERGENCY_IO;
// const static Device_t parentDeviceTypeId = DEVICE_GPIO;
// ----------------------------------------------------------------------------

/* Don't change these; they allow the base class to access locally assigned
 * variables.
 *****************************************************************************/

/* These must be changed per interface to ensure operability.
 *****************************************************************************/

/** Called at init. Should assign default modes to the pinBus object.
 * updateData() will be called after this, so there's no needto call it here.
 */

void Interface_EmergIO::prepareInterface(){
	pinBus.setAllPins(MODE_INPUT);
	commDevice->setPinModes(pinBus);
} // prepareInterface

DataError_t Interface_EmergIO::readPin(PinValue_t *valueIn) {
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
	case VALUE_DATA_DUMP: // Data format for dumping data over ROS messages
		// TODO: Prioritise these modes with attribute
		return commDevice->getPinValue(&val);
		break;
	case VALUE_GPIO_STATE:
		return errorVal;
		break;
	default:
		return ERROR_NOT_AVAIL;
	} // switch
} // readPin

DataError_t Interface_EmergIO::writePin(PinValue_t *valueIn) {
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
	default:
		return ERROR_NOT_AVAIL;
		break;
	} // switch
} // writePin

DataError_t Interface_EmergIO::writeConfig(InterfaceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // writeConfig

DataError_t Interface_EmergIO::readConfig(InterfaceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // readConfig

DataError_t Interface_EmergIO::writeDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // writeDeviceConfig

DataError_t Interface_EmergIO::readDeviceConfig(DeviceConfig_t *cfg) {
	return ERROR_NOT_AVAIL;
} // readDeviceConfig
