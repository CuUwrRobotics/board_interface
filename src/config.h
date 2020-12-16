/**
 * Hardware configruation values
 */
#include <stdint.h>

/**
 * Communication addresses
 */
const static uint8_t GPIO_0_ADDR = 0x25;
const static uint8_t GPIO_1_ADDR = 0x26;
const static uint8_t GPIO_2_ADDR = 0x27;

const static uint8_t PWM_0_ADDR = 0x41;
const static uint8_t PWM_1_ADDR = 0x40;

const static uint8_t ARDUINO_0_ADDR = 0x70;

const static uint8_t ADC_0_SPI_CS_PIN = 0;
const static uint8_t ADC_1_SPI_CS_PIN = 0;
const static uint8_t ADC_2_SPI_CS_PIN = 0;

/**
 * Adc 3 calibaration diode
 */
const float actualDiodeVoltage = 3; // This should be measured for accuracy
const float actualDiodeTolerance = .06; // 2%
float adcTolerance = .01; // (5v/2^9) for a 10-bit ADC after removing LSB
float avccTheoretical = 5;
