# @Author: Nick Steele <nichlock>
# @Date:   22:38 Sep 01 2020
# @Last modified by:   Nick Steele
# @Last modified time: 17:46 Feb 13 2021

Header header
# All data is binary such that:
## GPIO 0A pin #X = (gpio_0A >> X) & 0x01
## GPIO 0B pin #X = (gpio_0A >> 8+X) & 0x01
# Pins configured as inputs are descirbed in gpio_XY_cfg variable.
uint16 modes # 1 for output, 0 for input
uint16 values
uint16 pull_up # 1 for pull up on a pin, 0 to leave floating
uint16 rw_mask # Set a bit to write to that pin's state and modes
