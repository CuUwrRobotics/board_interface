/**
 * @Author: Nick Steele <nichlock>
 * @Date:   15:41 Aug 16 2020
 * @Last modified by:   nichlock
 * @Last modified time: 19:18 Sep 19 2020
 */

#include "BitTesting.h"

/**
 * Prints a seperator into the conosole for visual grepping
 */
inline void printSeperator(std::string name, uint8_t index) {
  printf("%s", YELLOW);
  printf("// ");
  printf("%s #", name.c_str());
  printf("%d ", index);
  for (int i = 0; i < (80 - name.length() - 8); i++)
    printf("/");
  printf("%s\n", NO_COLOR);
} // printLines

bool test::pwm(uint8_t index, bool direction){
  printSeperator("PWM", index);
  printf("PWM test beginning:\n");

  return true;
} // testPwm

bool test::gpio(uint8_t index){
  printSeperator("GPIO", index);
  printf("GPIO test beginning:\n");

  board_interface::gpio gpio_msg;

  return true;
} // testGpio

bool test::power(uint8_t index){
  printSeperator("Power MOSFET", index);
  printf("Power test beginning:\n");

  return true;
} // testPower

bool test::leak(uint8_t index){
  printSeperator("Leak", index);
  printf("Leak test beginning:\n");

  return true;
} // testLeak

bool test::emergencyIo(uint8_t index){
  printSeperator("Backup I/O", index);
  printf("EmergencyIO test beginning:\n");

  return true;
} // testEmergencyIo

bool test::led(uint8_t index){
  printSeperator("LED", index);
  printf("LED test beginning:\n");

  return true;
} // testLed

bool test::adc(uint8_t index){
  printSeperator("ADC", index);
  printf("ADC test beginning:\n");

  return true;
} // testAdc
