/**
 * @Author: Nick Steele <nichlock>
 * @Date:   15:41 Aug 16 2020
 * @Last modified by:   Nick Steele
 * @Last modified time: 12:17 Feb 13 2021
 */

#include <stdio.h>
#include <string>
#include "BitTesting.h"
#include "HardwareMessages.h"

using std::cin;
using std::cout;
uint8_t verify = 0;

std::string getTopicName(char *type, uint8_t index){
  std::string str(type);
  str += '_';
  str += '0' + index;
  str += "_in";
  return str;
} // getTopicName

/**
 * Waits for subscribers to show up before returning
 * https://answers.ros.org/question/65774/publisher-not-publishing-on-topic/
 */
void waitForRosPublisherSubs(ros::Publisher &pub){
  if (pub.getNumSubscribers() <= 0)
    usleep(10 * 1000); // Give 10 ms before showing a message
  while (pub.getNumSubscribers() <= 0) {
    printf("waiting for subscribers to publisher\n");
    usleep(2000000); // 2 sec
  }
} // waitForRosPublisherSubs

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

void setPwm(uint8_t id, uint8_t pin, float val, ros::Publisher &pub){
  board_interface::pwm pwm_msg;
  pwm_msg.header.stamp = ros::Time::now();
  if (pin == 16) {
    // Set all pins
    for (uint8_t i = 0; i < 16; i++)
      pwm_msg.values[i] = val;
    pwm_msg.frequency = 0;
    pwm_msg.rw_mask = (uint16_t)0xFFFF;
    pub.publish(pwm_msg);
    return;
  }
  pwm_msg.values[pin] = val;
  pwm_msg.frequency = 0;
  pwm_msg.rw_mask = (uint16_t)0x0001 << (pin);
  pub.publish(pwm_msg);
} // setPwm

void setPwmFreq(uint8_t id, float val, ros::Publisher &pub){
  board_interface::pwm pwm_msg;
  pwm_msg.frequency = val;
  pwm_msg.rw_mask = 0;
  pwm_msg.header.stamp = ros::Time::now();
  pub.publish(pwm_msg);
} // setPwmFreq

bool test::pwm(uint8_t index, ros::NodeHandle n){
  printSeperator("PWM", index);
  ros::Publisher pub = n.advertise <board_interface::pwm>
                         (getTopicName("pwm", index).c_str(), 0);
  waitForRosPublisherSubs(pub);
  printf("PWM test beginning.\n");
  printf("50\% duty on pin #0.\n");
  setPwm(index, 0, 50, pub);
  printf("Please verify: (y/n)");
  std::cin >> verify;
  if (verify != 'y') {
    log_error_nargs("Test failed. Will not continue this cycle.");
    return false;
  }
  printf("50Hz.\n");
  setPwmFreq(index, 50, pub);
  printf("Please verify: (y/n)");
  std::cin >> verify;
  if (verify != 'y') {
    log_error_nargs("Test failed. Will not continue this cycle.");
    return false;
  }
  printf("3KHz.\n");
  setPwmFreq(index, 3000, pub);
  printf("Please verify: (y/n)");
  std::cin >> verify;
  if (verify != 'y') {
    log_error_nargs("Test failed. Will not continue this cycle.");
    return false;
  }
  printf("1KHz.\n");
  setPwmFreq(index, 1000, pub);
  printf("Please verify: (y/n)");
  std::cin >> verify;
  if (verify != 'y') {
    log_error_nargs("Test failed. Will not continue this cycle.");
    return false;
  }
  printf("90\% duty on pin #0.\n");
  setPwm(index, 0, 90, pub);
  printf("Please verify: (y/n)");
  std::cin >> verify;
  if (verify != 'y') {
    log_error_nargs("Test failed. Will not continue this cycle.");
    return false;
  }
  printf("10\% duty on pin #0.\n");
  setPwm(index, 0, 10, pub);
  printf("Please verify: (y/n)");
  std::cin >> verify;
  if (verify != 'y') {
    log_error_nargs("Test failed. Will not continue this cycle.");
    return false;
  }
  printf("50\% duty on all pins.\n");
  setPwm(index, 16, 50, pub);
  printf("Please verify: (y/n)");
  std::cin >> verify;
  if (verify != 'y') {
    log_error_nargs("Test failed.");
  }
  return false;
} // testPwm

void setGpio(uint8_t pin, uint8_t mode, uint8_t value, ros::Publisher &pub){
  board_interface::gpio gpio_msg;
  gpio_msg.header.stamp = ros::Time::now();
  if (pin == 16) {
    // Set all pins
    if (value == 1) {
      gpio_msg.values = (uint16_t)0xFFFF;
    } else {
      gpio_msg.values = 0;
    }
    if (mode == 1) {
      gpio_msg.modes = (uint16_t)0xFFFF;
    } else {
      gpio_msg.modes = 0;
    }
    gpio_msg.rw_mask = (uint16_t)0xFFFF;
    pub.publish(gpio_msg);
    return;
  }
  if (value == 1) {
    gpio_msg.values = (uint16_t)0x0001 << (pin);
  } else {
    gpio_msg.values = (gpio_msg.values) & (~((uint16_t)0x0001 << (pin)));
    gpio_msg.values = 0;
  }
  if (mode == 1) {
    gpio_msg.modes = (uint16_t)0x0001 << (pin);
  } else {
    gpio_msg.modes = (gpio_msg.modes) & (~((uint16_t)0x0001 << (pin)));
    gpio_msg.modes = 0;
  }
  gpio_msg.rw_mask = (uint16_t)0x0001 << (pin);
  pub.publish(gpio_msg);
} // setGpio

bool test::gpio(uint8_t index, ros::NodeHandle n){
  printSeperator("GPIO", index);
  ros::Publisher pub = n.advertise <board_interface::gpio>
                         (getTopicName("gpio", index).c_str(), 0);
  waitForRosPublisherSubs(pub);
  printf("GPIO test beginning.\n");
  printf("Pin 0: mode input, off. (pin should be OFF)\n");
  setGpio(0, 0, 0, pub);
  printf("Please verify: (y/n)");
  std::cin >> verify;
  if (verify != 'y') {
    log_error_nargs("Test failed. Will not continue this cycle.");
    return false;
  }
  printf("Pin 0: mode input, on. (pin should be OFF)\n");
  setGpio(0, 0, 1, pub);
  printf("Please verify: (y/n)");
  std::cin >> verify;
  if (verify != 'y') {
    log_error_nargs("Test failed. Will not continue this cycle.");
    return false;
  }
  printf("Pin 0: mode output, off. (pin should be OFF)\n");
  setGpio(0, 1, 0, pub);
  printf("Please verify: (y/n)");
  std::cin >> verify;
  if (verify != 'y') {
    log_error_nargs("Test failed. Will not continue this cycle.");
    return false;
  }
  printf("Pin 0: mode output, on. (pin should be ON)\n");
  setGpio(0, 1, 1, pub);
  printf("Please verify: (y/n)");
  std::cin >> verify;
  if (verify != 'y') {
    log_error_nargs("Test failed. Will not continue this cycle.");
    return false;
  }
  printf("All pins ON)\n");
  setGpio(16, 1, 1, pub);
  printf("Please verify: (y/n)");
  std::cin >> verify;
  if (verify != 'y') {
    log_error_nargs("Test failed. Will not continue this cycle.");
    return false;
  }
  printf("All pins OFF)\n");
  setGpio(16, 1, 0, pub);
  printf("Please verify: (y/n)");
  std::cin >> verify;
  if (verify != 'y') {
    log_error_nargs("Test failed. Will not continue this cycle.");
    return false;
  }

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
