Header header
# All data is binary such that: GPIO 0A pin #X = (gpio_0A >> X) & 0x01
# Pins configured as inputs are descirbed in gpio_XY_cfg variable.
uint16 gpio_modes
uint16 gpio
uint16 gpio_rw_mask # Set a bit to write to that pin
