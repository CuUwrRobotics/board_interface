/**
 * @Author: Nick Steele <nichlock>
 * @Date:   10:53 Aug 15 2020
 * @Last modified by:   Nick Steele
 * @Last modified time: 13:00 Nov 26 2020
 */

#include "BitTesting.h"

int main(int argc, char const *argv[]) {
  bool testFailed = false;
  if (!test::pwm(0, true)) testFailed = true;
  if (!test::pwm(0, false)) testFailed = true;
  if (!test::pwm(1, true)) testFailed = true;
  if (!test::pwm(1, false)) testFailed = true;

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
