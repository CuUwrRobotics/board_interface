/**
 * @Author: Nick Steele <nichlock>
 * @Date:   9:08 Aug 15 2020
 * @Last modified by:   Nick Steele
 * @Last modified time: 15:24 Dec 16 2020
 */

#include "Device_Pwm_Pca9685.h"

/**
 * deviceInit is called by Device::init() just before completion. It should at
 * least init the chip and run updateData() once. Note that the uint8_t var
 * 'address' already has the address set by the time this is called.
 * @return whether the init worked.
 */

bool Device_Pwm_Pca9685::deviceInit(){
  // Init chip here
  Wire.begin();
  _reset();
  _oscillator_freq = FREQUENCY_OSCILLATOR;
  _setPWMFreq(1000);
  _setPWM(0, 0, 0);
  writeDataPending = true;
  deviceIsSetup = true;
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
 */

DataError_t Device_Pwm_Pca9685::setPinValue(PinValue_t *value) {
  if (!(value->pin >= 0 && value->pin < PIN_COUNT))
    return ERROR_DEV_PIN_INVALID;
  // Doesn't flag for a data write if no changes are made.
  switch (value->fmt) {
  case VALUE_PWM_FREQ:
    if (value->data[0] == currentFrequencyValue) {
      return ERROR_SUCCESS;
    }	else {
      requestedFrequencyValue = value->data[0];
      writeDataPending = true;
      return ERROR_SUCCESS;
    }
    break;
  case VALUE_PWM_ON_TICKS:
    if (value->data[0] == requestedPinTicks[value->pin]) {
      return ERROR_SUCCESS;
    }	else {
      requestedPinTicks[value->pin] = value->data[0];
      writeDataPending = true;
      return ERROR_SUCCESS;
    }
    break;
  default:
    return ERROR_NOT_AVAIL;
  } // switch
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
    if (writeDataPending) {
      for (uint8_t i = 0; i < PIN_COUNT; i++) {
        if (currentPinTicks[i] != requestedPinTicks[i]) {
          _setPWM(i, 0, requestedPinTicks[i]);
          currentPinTicks[i] = requestedPinTicks[i];
        }
      }
      if (currentFrequencyValue != requestedFrequencyValue) {
        _setPWMFreq(requestedFrequencyValue);
        currentFrequencyValue = requestedFrequencyValue;
      }
      writeDataPending = false;
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

void Device_Pwm_Pca9685::_reset() {
  _write8(PCA9685_MODE1, MODE1_RESTART);
  delay(10);
} // Device_Pwm_Pca9685::_reset

void Device_Pwm_Pca9685::_setPWMFreq(float freq) {
  // Range output modulation frequency is dependant on oscillator
  if (freq < 1)
    freq = 1;
  if (freq > 3500)
    freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

  float prescaleval = ((_oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
  if (prescaleval < PCA9685_PRESCALE_MIN)
    prescaleval = PCA9685_PRESCALE_MIN;
  if (prescaleval > PCA9685_PRESCALE_MAX)
    prescaleval = PCA9685_PRESCALE_MAX;
  uint8_t prescale = (uint8_t)prescaleval;

  uint8_t oldmode = _read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  _write8(PCA9685_MODE1, newmode); // go to sleep
  _write8(PCA9685_PRESCALE, prescale); // set the prescaler
  _write8(PCA9685_MODE1, oldmode);
  delay(5);
  // This sets the MODE1 register to turn on auto increment.
  _write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
} // Device_Pwm_Pca9685::_setPWMFreq

uint8_t Device_Pwm_Pca9685::_read8(uint8_t addr) {
  Wire.beginTransmission(address);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)address, (uint8_t)1);
  return Wire.read();
} // Device_Pwm_Pca9685::_read8

void Device_Pwm_Pca9685::_write8(uint8_t addr, uint8_t d) {
  Wire.beginTransmission(address);
  Wire.write(addr);
  Wire.write(d);
  Wire.endTransmission();
} // Device_Pwm_Pca9685::_write8

/*!
 *  @brief  Gets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @return requested PWM output value
 */
uint8_t Device_Pwm_Pca9685::_getPWM(uint8_t num) {
  Wire.requestFrom((int)address, PCA9685_LED0_ON_L + 4 * num, (int)4);
  return Wire.read();
} // Device_Pwm_Pca9685::_getPWM

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 */
void Device_Pwm_Pca9685::_setPWM(uint8_t num, uint16_t on, uint16_t off) {
  Wire.beginTransmission(address);
  Wire.write(PCA9685_LED0_ON_L + 4 * num);
  Wire.write(on);
  Wire.write(on >> 8);
  Wire.write(off);
  Wire.write(off >> 8);
  Wire.endTransmission();
} // Device_Pwm_Pca9685::_setPWM
