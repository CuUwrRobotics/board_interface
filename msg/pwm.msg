Header header

# Values are always published as a duty cycle from 0-1
float32[16] values
float32 frequency
uint16 rw_mask # Set pins to be written to 1 in this mask. Last bit = first pin.
