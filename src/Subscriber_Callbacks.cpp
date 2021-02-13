#include "Subscriber_Callbacks.h"
#include "AllDevicesInterfaces.h"

namespace callbacks {
void pwm0(board_interface::pwm msg){
  pwmCallback(msg, 0);
} /* pwm0 */

void pwm1(board_interface::pwm msg){
  pwmCallback(msg, 1);
} /* pwm1 */

void gpio0(board_interface::gpio msg){
  gpioCallback(msg, 0);
} // gpio0

void gpio1(board_interface::gpio msg){
  gpioCallback(msg, 1);
} // gpio1

void gpio2(board_interface::gpio msg){
  gpioCallback(msg, 2);
} // gpio2

void pwmCallback(board_interface::pwm &msg, uint8_t index) {
  Interface_Indexer_t intf_i;
  intf_i.index = index;
  intf_i.type = INTF_PWM;

  PinValue_t pin_value_data; // Stores pin value data
  float data; // Stores the data to be sent to the interface by refrence
  pin_value_data.data = &data; // Pointer to data is reusable
  pin_value_data.pin = 0;

  if (msg.frequency != 0) {
    data = msg.frequency;
    pin_value_data.fmt = VALUE_PWM_FREQ;
    pin_value_data.pin = 0;
    interfaces[intf_i]->writePin(&pin_value_data);
  }

  pin_value_data.fmt = VALUE_PWM_DUTY_100;
  for (uint8_t pin_i = 0; pin_i < 16; pin_i++) {
    if (((msg.rw_mask) & ((uint16_t)0b1) << pin_i) != 0) { // if the rw bit is set
      data = msg.values[pin_i];
      pin_value_data.pin = pin_i;
      interfaces[intf_i]->writePin(&pin_value_data);
    }
  }
  devices[interfaces[intf_i]->getParentDeviceIndex()]->updateData();
} // pwmCallback

void gpioCallback(board_interface::gpio &msg, uint8_t index) {
  Interface_Indexer_t intf_i;
  intf_i.index = index;
  intf_i.type = INTF_GPIO;

  PinValue_t pin_value_data; // Stores pin value data
  float data; // Stores the data to be sent to the interface by refrence
  pin_value_data.data = &data; // Pointer to data is reusable
  pin_value_data.pin = 0;

  pin_value_data.fmt = VALUE_GPIO_MODE;
  for (uint8_t pin_i = 0; pin_i < 16; pin_i++) {
    if (((msg.rw_mask) & ((uint16_t)0b1) << pin_i) != 0) { // if the rw bit is set
      pin_value_data.pin = pin_i;

      pin_value_data.fmt = VALUE_GPIO_MODE;
      data = (float)((msg.modes >> pin_i) & 0x1);
      interfaces[intf_i]->writePin(&pin_value_data);

      pin_value_data.fmt = VALUE_GPIO_STATE;
      data = (float)((msg.values >> pin_i) & 0x1);
      interfaces[intf_i]->writePin(&pin_value_data);

      pin_value_data.fmt = VALUE_GPIO_PUPD;
      data = (float)((msg.pull_up >> pin_i) & 0x1);
      interfaces[intf_i]->writePin(&pin_value_data);
    }
  }
  devices[interfaces[intf_i]->getParentDeviceIndex()]->updateData();
} // pwmCallback
} // namespace callbacks
