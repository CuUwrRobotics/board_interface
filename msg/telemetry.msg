# @Author: Nick Steele <nichlock>
# @Date:   16:38 Aug 12 2020
# @Last modified by:   nichlock
# @Last modified time: 19:26 Sep 19 2020

Header header
# Array indexes for the voltage and current
uint8 VOLTAGE_3V3 = 0
uint8 VOLTAGE_5V = 1
uint8 VOLTAGE_12V = 2
uint8 VOLTAGE_VIN = 3
uint8 CURRENT_0 = 0
uint8 CURRENT_1 = 1


# Telemetry information.
float32 temperature_deg_c
float32 temperature_deg_c_tolerance
float32[2] current_amps
float32[2] current_amps_tolerance
float32[4] power_line_volts
float32[4] power_line_volts_tolerance
uint8 switches # Binary output;  0000dcba, Last bit = first pin.
uint8 em_io # Returns binary result: 00000cba
uint8 em_io_rw_mask # Set the corresponding bit here to 1 to write to em_io
