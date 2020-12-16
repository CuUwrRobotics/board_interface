/**
 * @Author: Nick Steele <nichlock>
 * @Date:   16:38 Aug 12 2020
 * @Last modified by:   Nick Steele
 * @Last modified time: 15:28 Dec 16 2020
 */

#ifndef DEVICE_PWM_PCA9685_H
#define DEVICE_PWM_PCA9685_H
// Generic headers
#include <stdint.h>
#include <stdio.h>
#include <ros/ros.h>
#include "HardwareDescription.h"
#include "PinBus.h"
#include "HardwareData.h"
#include "Device.h"
// #include "Interface.h"
#include "Logger.h"

#include "arduino_port_lib/ArduinoPort.h" // Generic functions like delay()
#include "arduino_port_lib/Ardu_Wire.h" // Arduino Wire (I2C)

// TODO: make this a config for interfaces to get value at their start
const float MAX_PWM_TICKS = 4095;
const float MIN_PWM_TICKS = 0;
const float MAX_VALUE_PWM_FREQUENCY_VALUE = 3500;
const float MIN_VALUE_PWM_FREQUENCY_VALUE = 0;

// Header file custom to this specific chip
// #include "Device_Gpio_Mcp23017.h"

class Device_Pwm_Pca9685 : public Device {
private:
// THESE VALUES MUST BE REVIEWED ON CREATION OF EACH NEW DEVICE SUBCLASS
// ***************************************************************************
// Metadata for Troubleshooting
// ============================
// Name specific to the product this device subclass will interface with.
char HARDWARE_NAME[8] = "PCA9685";

// Informating About The Chip Used
// ===============================.
const static uint8_t PIN_COUNT = 16;
const static Device_t deviceTypeId = DEVICE_PWM;

// Pin Modes That This Chip can Accept
// ==============================================
const static uint8_t VALID_PIN_MODE_COUNT = 1;
const PinMode_t validPinModes[VALID_PIN_MODE_COUNT] = {MODE_OUTPUT};

// Other Variables (Don't change these)
// ====================================
Interface_t reservedPins[PIN_COUNT];
// For storing tick rate (duty cycle)
float currentPinTicks[PIN_COUNT];
float requestedPinTicks[PIN_COUNT];
// For storing device frequncy
float currentFrequencyValue;
float requestedFrequencyValue;

uint32_t _oscillator_freq;

// REGISTER ADDRESSES
const uint8_t PCA9685_MODE1 = 0x00; /**< Mode Register 1 */
const uint8_t PCA9685_MODE2 = 0x01; /**< Mode Register 2 */
const uint8_t PCA9685_SUBADR1 = 0x02; /**< I2C-bus subaddress 1 */
const uint8_t PCA9685_SUBADR2 = 0x03; /**< I2C-bus subaddress 2 */
const uint8_t PCA9685_SUBADR3 = 0x04; /**< I2C-bus subaddress 3 */
const uint8_t PCA9685_ALLCALLADR = 0x05; /**< LED All Call I2C-bus address */
const uint8_t PCA9685_LED0_ON_L = 0x06; /**< LED0 on tick, low byte*/
const uint8_t PCA9685_LED0_ON_H = 0x07; /**< LED0 on tick, high byte*/
const uint8_t PCA9685_LED0_OFF_L = 0x08; /**< LED0 off tick, low byte */
const uint8_t PCA9685_LED0_OFF_H = 0x09; /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
const uint8_t PCA9685_ALLLED_ON_L = 0xFA; /**< load all the LEDn_ON registers, low */
const uint8_t PCA9685_ALLLED_ON_H = 0xFB; /**< load all the LEDn_ON registers, high */
const uint8_t PCA9685_ALLLED_OFF_L = 0xFC; /**< load all the LEDn_OFF registers, low */
const uint8_t PCA9685_ALLLED_OFF_H = 0xFD; /**< load all the LEDn_OFF registers,high */
const uint8_t PCA9685_PRESCALE = 0xFE; /**< Prescaler for PWM output frequency */
const uint8_t PCA9685_TESTMODE = 0xFF; /**< defines the test mode to be entered */

// MODE1 bits
const uint8_t MODE1_ALLCAL = 0x01; /**< respond to LED All Call I2C-bus address */
const uint8_t MODE1_SUB3 = 0x02; /**< respond to I2C-bus subaddress 3 */
const uint8_t MODE1_SUB2 = 0x04; /**< respond to I2C-bus subaddress 2 */
const uint8_t MODE1_SUB1 = 0x08; /**< respond to I2C-bus subaddress 1 */
const uint8_t MODE1_SLEEP = 0x10; /**< Low power mode. Oscillator off */
const uint8_t MODE1_AI = 0x20; /**< Auto-Increment enabled */
const uint8_t MODE1_EXTCLK = 0x40; /**< Use EXTCLK pin clock */
const uint8_t MODE1_RESTART = 0x80; /**< Restart enabled */
// MODE2 bits
const uint8_t MODE2_OUTNE_0 = 0x01; /**< Active LOW output enable input */
const uint8_t MODE2_OUTNE_1 = 0x02; /**< Active LOW output enable input - high impedience */
const uint8_t MODE2_OUTDRV = 0x04; /**< totem pole structure vs open-drain */
const uint8_t MODE2_OCH = 0x08; /**< Outputs change on ACK vs STOP */
const uint8_t MODE2_INVRT = 0x10; /**< Output logic state inverted */

const uint8_t PCA9685_I2C_ADDRESS = 0x40; /**< Default PCA9685 I2C Slave Address */
const uint32_t FREQUENCY_OSCILLATOR = 25000000; /**< Int. osc. frequency in datasheet */

const uint8_t PCA9685_PRESCALE_MIN = 3; /**< minimum prescale value */
const uint8_t PCA9685_PRESCALE_MAX = 255; /**< maximum prescale value */

/* These give the base Device class access to the above local variables. They
 * don't need any modification. See more info about each function in the Device
 * class.
 ******************************************************************************/

//
inline uint8_t getPinCount() {
  return PIN_COUNT;
} // getPinCount

//
inline Device_t getDeviceTypeId() {
  return deviceTypeId;
} // getDeviceTypeId

//
inline PinMode_t getValidPinModes(uint8_t i){
  return validPinModes[i];
} // getValidPinModes

//
inline uint8_t getValidPinModesCount(){
  return VALID_PIN_MODE_COUNT;
} // getValidPinModesCount

//
inline Interface_t &getReservedPins(uint8_t i){
  return reservedPins[i];
} // getReservedPins

//
inline char *getHardwareName(){
  return HARDWARE_NAME;
} // getHardwareName

/* These actually drive the chip, and must be different for each device subclass.
 ******************************************************************************/

/**
 * deviceInit is called by Device::init() just before completion. It should at
 * least init the chip and run updateData() once. Note that the uint8_t var
 * 'address' already has the address set by the time this is called.
 * @return whether the init worked.
 */

bool deviceInit();

/**
 * Internal hardware interfacing functions
 */
void _setPWMFreq(float freq);

void _reset();

uint8_t _read8(uint8_t addr);

void _write8(uint8_t addr, uint8_t d);

/*!
 *  @brief  Gets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @return requested PWM output value
 */
uint8_t _getPWM(uint8_t num);

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 */
void _setPWM(uint8_t num, uint16_t on, uint16_t off);

public:

DataError_t getPinValue(PinValue_t *value);

/**
 * Note that all data handling (min/max vals) happens in interfaces
 */

DataError_t setPinValue(PinValue_t *value);

DataError_t writeDeviceConfig(DeviceConfig_t *cfg);

DataError_t readDeviceConfig(DeviceConfig_t *cfg);

/**
 * Reads/writes any data related to the chip. This function will interface
 * directly with the chip's hardware interface, and is the ONLY way that any code
 * can read from or write to the chip at 'address'.
 * Note that the order this happens in is important.
 * @return if update succeeded
 */

bool updateData();
}

;

#endif /* ifndef DEVICE_PWM_PCA9685_H */
