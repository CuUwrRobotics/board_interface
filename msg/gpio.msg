# @Author: Nick Steele <nichlock>
# @Date:   22:38 Sep 01 2020
# @Last modified by:   nichlock
# @Last modified time: 19:26 Sep 19 2020

Header header
# All data is binary such that: GPIO 0A pin #X = (gpio_0A >> X) & 0x01
# Pins configured as inputs are descirbed in gpio_XY_cfg variable.
uint16 modes
uint16 values
uint16 rw_mask # Set a bit to write to that pin
