/**
 * @Author: Nick Steele <nichlock>
 * @Date:   10:53 Aug 15 2020
 * @Last modified by:   Nick Steele
 * @Last modified time: 18:22 Dec 15 2020
 */

#include "BitTesting.h"

int main(int argc, char *argv[]) {
  printf("start\n");
  ros::init(argc, argv, "board_interface_bit_tester",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  while (!ros::ok()) {
    printf("waiting for ros\n");
    usleep(2000000);
  }
  bool testFailed = false;
  if (!test::pwm(0, n)) testFailed = true;
  if (!test::pwm(1, n)) testFailed = true;

  if (!test::gpio(0)) testFailed = true;
  if (!test::gpio(1)) testFailed = true;

  if (!test::power(0)) testFailed = true;
  if (!test::power(1)) testFailed = true;
  if (!test::power(2)) testFailed = true;
  if (!test::power(3)) testFailed = true;

  if (!test::leak(0)) testFailed = true;

  if (!test::emergencyIo(0)) testFailed = true;

  if (!test::led(0)) testFailed = true;

  if (!test::adc(0)) testFailed = true;
  if (!test::adc(1)) testFailed = true;

  if (testFailed) {
    log_error_nargs("Tests have failed!");
    exit(EXIT_FAILURE);
  }
  printf("%sAll BIT tests successful :)%s\n", GREEN, NO_COLOR);
  exit(EXIT_SUCCESS);
} // main
