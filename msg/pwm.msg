# @Author: Nick Steele <nichlock>
# @Date:   12:06 Aug 15 2020
# @Last modified by:   nichlock
# @Last modified time: 19:25 Sep 19 2020

Header header

# Values are always published as a duty cycle from 0-100
float32[16] values
float32 frequency # Set to zero to not set frequency
uint16 rw_mask # Set pins to be written to 1 in this mask. Last bit = first pin.
