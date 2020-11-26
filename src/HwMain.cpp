/**
 * @Author: Nick Steele <nichlock>
 * @Date:   13:37 Sep 05 2020
 * @Last modified by:   Nick Steele
 * @Last modified time: 13:01 Nov 26 2020
 */

/**
 * Functionality:
 * Any device to be communicated with (eg, a GPIO chip) will be accessible via its class object.
 * The sub-interfaces (interfaces which aren't their own device, eg, PWR switching) will be defined
 * using classes containing pointers to the class given to their 'parent' interface. When new data
 * is recieved, it is intrerpreted and passed on to the parent class. To update data, each parent
 * class is told to update. It will check if data has been updated, and if it has, it will send the
 * updated info over the communication lines.
 */

#include "HwHeader.h"

#define DEVICE_ADDRESS 0x55

// Interface *interfaces[TOTAL_INTERFACES];
// Device *devices[TOTAL_DEVICES];

std::map <Interface_Indexer_t, Interface *> interfaces;
std::map <Interface_t, uint8_t> interface_qty;

std::map <Device_Indexer_t, Device *> devices;
std::map <Device_t, uint8_t> device_qty;

std::map <Publisher_Indexer_t, ros::Publisher> publishers;
std::map <Publisher_t, uint8_t> publisher_qty;

// Tells devices if the hardware is actually connected or if it should be faked
bool simulate_hw = false;
// Whether a BIT test will run at start
bool use_bit_test = false;

// void createInterface(Interface_t type, Device *parent_device, PinBus pin_bus){
//  Interface_Indexer_t intf = Interface_Indexer_t(type, 0);
//  for (int i = 0; i < 16; i++) { // For all possible indexes for the device
//    intf.index = i;
//    if (interfaces.find(intf) == interfaces.end()) { // If the index does not yet exist
//      createInterface(type, pin_bus, i);
//      break; // Stop search
//    }
//  }
// }; // createInterface

inline void createInterface(Interface_t type, Interface *interface,
                            Device *parent_device, PinBus pin_bus){
  Interface_Indexer_t i = Interface_Indexer_t(type, interface_qty[type]);
  // printf("Creating interface: %s, %d\n", interfaceIdToCharArray(type),
  //        interface_qty[type]);
  interfaces[i] = interface;
  interfaces[i]->start(parent_device, pin_bus, i);
  interface_qty[type] += 1;
} // createInterface

/**
 * Creates a new device at an unused device index
 */

// void createDevice(Device_t type, BusType_t bus_type, uint8_t addr){
//  Device_Indexer_t dev = Device_Indexer_t(type, 0);
//  for (int i = 0; i < 16; i++) { // For all possible indexes for the device
//    dev.index = i;
//    if (devices.find(dev) == devices.end()) { // If the index does not yet exist
//      createDevice(type, bus_type, addr, i);
//      break; // Stop search
//    }
//  }
// }; // createDevice

/**
 * Creates a new device at given index
 */

inline void createDevice(Device_t type, Device *device, BusType_t bus_type,
                         uint8_t addr){
  Device_Indexer_t d = Device_Indexer_t(type, device_qty[type]);
  // printf("Creating device: %s, %d\n", deviceIdToCharArray(type),
  //        device_qty[type]);
  devices[d] = device;
  devices[d]->init(d, addr, bus_type, simulate_hw);
  device_qty[type] += 1;
} // createDevice

/**
 * Creates device objects, defining what type of device each object is.
 */

void createAndInitDevices(){
  uint8_t i = 0; // Incremented within each device type.
  Device_Indexer_t dev; // Stores device type and index for map array

  // GPIO 0
  createDevice(DEVICE_GPIO, new Device_Gpio_Mcp23017(), BUS_GPIO,
               DEVICE_ADDRESS + i);
  i++;

  // GPIO 1
  createDevice(DEVICE_GPIO, new Device_Gpio_Mcp23017(), BUS_GPIO,
               DEVICE_ADDRESS + i);
  i++;

  // GPIO 2 (only accessible through other interfaces, like LEAK)
  createDevice(DEVICE_GPIO, new Device_Gpio_Mcp23017(), BUS_GPIO,
               DEVICE_ADDRESS + i);
  i++;

  // PWM 0
  createDevice(DEVICE_PWM, new Device_Pwm_Pca9685(), BUS_PWM,
               DEVICE_ADDRESS + i);
  i++;

  // PWM 1
  createDevice(DEVICE_PWM, new Device_Pwm_Pca9685(), BUS_PWM,
               DEVICE_ADDRESS + i);
  i++;

  // ADC 0
  createDevice(DEVICE_ADC, new Device_Adc_Mcp3008(), BUS_ADC,
               DEVICE_ADDRESS + i);
  i++;

  // ADC 1
  createDevice(DEVICE_ADC, new Device_Adc_Mcp3008(), BUS_ADC,
               DEVICE_ADDRESS + i);
  i++;

  // ADC 2 (only accessible through other interfaces.)
  createDevice(DEVICE_ADC, new Device_Adc_Mcp3008(), BUS_ADC,
               DEVICE_ADDRESS + i);
  i++;

  // variable value must be changed if this is true
  // if (i != TOTAL_DEVICES) {
  //  ROS_ERROR(
  //    "ERROR: TOTAL_DEVICES NOT SET CORRECTLY! DOUBLE CHECK DEVICE COUNT OR SETUP!\nSTOPPING.\n");
  //  while (true) {}
  // }

  // ADD MORE DEVICES HERE
} // createAndInitDevices

/**
 * Creates interface objects, defining what type of interface each object is.
 */

void createAndInitInterfaces(){
  uint8_t i = 0; // Increment on each new interface
  uint8_t d = 0; // Increment on each new device
  PinBus pinBus; // Set on every new interface
  BusType_t busType = BUS_INVALID; // Set on every new device type

  Interface_Indexer_t intf; // Stores device type and index for map array
  Device_Indexer_t dev; // Stores device type and index for map array

  // GPIO 0 (Device index 0)
  // ***************************************************************************
  busType = BUS_GPIO;
  dev = Device_Indexer_t(DEVICE_GPIO, 0);

  pinBus.createUniformPinBusFromSet(busType, 0, 15, MODE_INPUT);
  createInterface(INTF_GPIO, new Interface_Gpio(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // GPIO 1 (Device index 1)
  // ***************************************************************************
  busType = BUS_GPIO;
  dev = Device_Indexer_t(DEVICE_GPIO, 1);
  i = 0;

  pinBus.createUniformPinBusFromSet(busType, 0, 15, MODE_INPUT);
  createInterface(INTF_GPIO, new Interface_Gpio(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // GPIO 2 (device index 2)
  // ***************************************************************************
  busType = BUS_GPIO;
  dev = Device_Indexer_t(DEVICE_GPIO, 2);
  i = 0;

  // LEAK interface: 8 pins
  pinBus.createUniformPinBusFromSet(busType, 0, 7, MODE_INPUT);
  createInterface(INTF_LEAK, new Interface_Leak(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // PWR interface: 4 pins
  pinBus.createUniformPinBusFromSet(busType, 8, 11, MODE_OUTPUT);
  createInterface(INTF_PWR_SWITCHING, new Interface_Power(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // Emergeny I/O telemetry interface: 3 pins
  pinBus.createUniformPinBusFromSet(busType, 12, 14, MODE_OUTPUT);
  createInterface(INTF_EMERGENCY_IO, new Interface_EmergIO(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // Led which warns of leaks/other major issues. interface: 2 pin
  pinBus.createUniformPinBusFromSet(busType, 15, 15, MODE_OUTPUT);
  createInterface(INTF_LED, new Interface_LeakLed(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // PWM 0 (Device index 3)
  // ***************************************************************************
  busType = BUS_PWM;
  dev = Device_Indexer_t(DEVICE_PWM, 0);
  i = 0;

  // PWM interface: 16 pins
  pinBus.createUniformPinBusFromSet(busType, 0, 15, MODE_OUTPUT);
  createInterface(INTF_PWM, new Interface_Pwm(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // PWM 1 (Device index 4)
  // ***************************************************************************
  busType = BUS_PWM;
  dev = Device_Indexer_t(DEVICE_PWM, 1);
  i = 0;
  // PWM interface: 16 pins
  pinBus.createUniformPinBusFromSet(busType, 0, 15, MODE_OUTPUT);
  createInterface(INTF_PWM, new Interface_Pwm(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // ADC 0 (Device index 5)
  // ***************************************************************************
  busType = BUS_ADC;
  dev = Device_Indexer_t(DEVICE_ADC, 0);
  i = 0;

  // PWM interface: 16 pins
  pinBus.createUniformPinBusFromSet(busType, 0, 7, MODE_INPUT);
  createInterface(INTF_ADC, new Interface_Adc(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // ADC 1 (Device index 6)
  // ***************************************************************************
  busType = BUS_ADC;
  dev = Device_Indexer_t(DEVICE_ADC, 1);
  i = 0;
  // PWM interface: 16 pins
  pinBus.createUniformPinBusFromSet(busType, 0, 7, MODE_INPUT);
  createInterface(INTF_ADC, new Interface_Adc(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // ADC 2 (Device index 6)
  // ***************************************************************************
  busType = BUS_ADC;
  dev = Device_Indexer_t(DEVICE_ADC, 2);
  i = 0;

// CURRENT interface: 1 pin
  pinBus.createUniformPinBusFromSet(busType, 0, 0, MODE_INPUT);
  createInterface(INTF_CURRENT, new Interface_Current_Acs781(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // CURRENT interface: 1 pin
  pinBus.createUniformPinBusFromSet(busType, 1, 1, MODE_INPUT);
  createInterface(INTF_CURRENT, new Interface_Current_Acs781(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // VOLTAGE REFRENCE interface: 1 pin
  pinBus.createUniformPinBusFromSet(busType, 2, 2, MODE_INPUT);
  createInterface(INTF_VREF, new Interface_Voltage_Refrence(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // TEMP interface: 1 pin
  pinBus.createUniformPinBusFromSet(busType, 3, 3, MODE_INPUT);
  createInterface(INTF_TEMP, new Interface_Temp_Lm62(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // INTF_VOLTAGE_DIVIDER interface: 1 pin (3.3v)
  pinBus.createUniformPinBusFromSet(busType, 4, 4, MODE_INPUT);
  createInterface(INTF_VOLTAGE_DIVIDER, new Interface_Voltage_Div(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // INTF_VOLTAGE_DIVIDER interface: 1 pin (5v)
  pinBus.createUniformPinBusFromSet(busType, 5, 5, MODE_INPUT);
  createInterface(INTF_VOLTAGE_DIVIDER, new Interface_Voltage_Div(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // INTF_VOLTAGE_DIVIDER interface: 1 pin (12v [?])
  pinBus.createUniformPinBusFromSet(busType, 6, 6, MODE_INPUT);
  createInterface(INTF_VOLTAGE_DIVIDER, new Interface_Voltage_Div(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // INTF_VOLTAGE_DIVIDER interface: 1 pin (VIN, 48V)
  pinBus.createUniformPinBusFromSet(busType, 7, 7, MODE_INPUT);
  createInterface(INTF_VOLTAGE_DIVIDER, new Interface_Voltage_Div(),
                  devices[dev],
                  pinBus);
  pinBus.resetAll();

  // variable value must be changed if this is true
  // if (i != TOTAL_INTERFACES) {
  //  ROS_ERROR(
  //    "ERROR: TOTAL_INTERFACES NOT SET CORRECTLY! DOUBLE CHECK INTERACE COUNT OR
  // SETUP!\nSTOPPING.\n");
  //  while (true) {}
  // }
} // createAndInitInterfaces

/**
 *
 */

void runBitTest(){
  // // run built in testing
  // bool testsOk = true;
  // Interface_Indexer_t intf; // Stores device type and index for map array
  // Device_Indexer_t dev; // Stores device type and index for map array
  //
  // // Run the tests and flag if one fails, since they dump a LOT of data.
  // // PWM testing
  // // ===========
  // intf = Interface_Indexer_t(INTF_PWM, 6);
  // dev = Device_Indexer_t(DEVICE_PWM, 3);
  // if (!bit_testing::testPwm(interfaces[intf], devices[dev], true)) testsOk =
  //    false;
  // if (!bit_testing::testPwm(interfaces[intf], devices[dev], false)) testsOk =
  //    false;
  // intf = Interface_Indexer_t(INTF_PWM, 7);
  // dev = Device_Indexer_t(DEVICE_PWM, 4);
  // if (!bit_testing::testPwm(interfaces[intf], devices[dev], true)) testsOk =
  //    false;
  // if (!bit_testing::testPwm(interfaces[intf], devices[dev], false)) testsOk =
  //    false;
  //
  // // GPIO testing
  // // ===========
  // intf = Interface_Indexer_t(INTF_GPIO, 0);
  // dev = Device_Indexer_t(DEVICE_GPIO, 1);
  // if (!bit_testing::testGpio(interfaces[intf], devices[dev])) testsOk = false;
  // intf = Interface_Indexer_t(INTF_GPIO, 1);
  // dev = Device_Indexer_t(DEVICE_GPIO, 1);
  // if (!bit_testing::testGpio(interfaces[intf], devices[dev])) testsOk = false;
  //
  // // POWER testing
  // // ===========
  // intf = Interface_Indexer_t(INTF_PWR_SWITCHING, 3);
  // dev = Device_Indexer_t(DEVICE_GPIO, 2);
  // if (!bit_testing::testPower(interfaces[intf], devices[dev])) testsOk = false;
  //
  // // LEAK testing
  // // ===========
  // intf = Interface_Indexer_t(INTF_LEAK, 2);
  // dev = Device_Indexer_t(DEVICE_GPIO, 2);
  // if (!bit_testing::testLeak(interfaces[intf], devices[dev])) testsOk = false;
  //
  // // EMERG_IO testing
  // // ================
  // intf = Interface_Indexer_t(INTF_EMERGENCY_IO, 4);
  // dev = Device_Indexer_t(DEVICE_ADC, 2);
  // if (!bit_testing::testEmergencyIo(interfaces[intf], devices[dev])) testsOk =
  //    false;
  //
  // // LEAK_LED testing
  // // ================
  // intf = Interface_Indexer_t(INTF_LED, 5);
  // dev = Device_Indexer_t(DEVICE_GPIO, 2);
  // if (!bit_testing::testLed(interfaces[intf], devices[dev])) testsOk = false;
  //
  // // ADC testing
  // // ================
  // intf = Interface_Indexer_t(INTF_ADC, 8);
  // dev = Device_Indexer_t(DEVICE_ADC, 5);
  // if (!bit_testing::testAdc(interfaces[intf], devices[dev])) testsOk = false;
  // intf = Interface_Indexer_t(INTF_ADC, 9);
  // dev = Device_Indexer_t(DEVICE_ADC, 6);
  // if (!bit_testing::testAdc(interfaces[intf], devices[dev])) testsOk = false;
  //
  // // All tests done
  // if (!testsOk) {
  //  printf(
  //    "%sWARINING: AT LEAST ONE BIT TEST HAS FAILED. Read the data above to find the reason.%s\n",
  //    RED, NO_COLOR);
  // }	else {
  //  printf("%sAll BIT tests good :)%s\n", GREEN, NO_COLOR);
  // }
} // runBitTest

/**
 *
 */

void calibrateAdc() {
  InterfaceConfig_t cfg; // For configuring interfaces
  PinValue_t val; // For reading VREF

  float diodeVoltage; // Known voltage.
  float measuredDiodeVoltage;
  float refReady;

  float refResults[2];
  float offsetRatio;
  float toleranceRatio; // Multiply by a voltage to get tolerance of estimate.
  float toleranceAtAvcc;
  float avccActual;

  Interface_Indexer_t vrefIndex = Interface_Indexer_t(INTF_VREF, 0);

  if (interface_qty[vrefIndex.type] == 0)
    log_error_nargs(
      "NO VREF interfaces detected! Nominal values will be chosen. (TODO?)");

  if (interface_qty[vrefIndex.type] > 1)
    log_error_nargs(
      "Multiple VREF interfaces detected. The code is currently able to use only one, so it will use VREF[0]");

  // First, set the truth values.
  float diodeData[2] = {actualDiodeVoltage, actualDiodeTolerance};
  cfg.fmt = ICFG_REF_KNOWN_VOLTS_WITH_TOLERANCE; // Set format
  cfg.data = diodeData;
  interfaces[vrefIndex]->writeConfig(&cfg);
  cfg.fmt = ICFG_REF_ADC_TOLERANCE; // Set format
  cfg.data = &adcTolerance;
  interfaces[vrefIndex]->writeConfig(&cfg);
  cfg.fmt = ICFG_REF_READY; // Set format
  cfg.data = &refReady;
  interfaces[vrefIndex]->readConfig(&cfg);
  if (!refReady) {
    log_error_nargs("VREF Interface did not give ready, cannot calibrate!\n");
    return;
  }
  cfg.fmt = ICFG_ADC_OFFSET_AND_TOLERANCE_RATIOS; // Set format
  cfg.data = refResults;
  interfaces[vrefIndex]->readConfig(&cfg);
  offsetRatio = refResults[0];
  toleranceRatio = refResults[1];
  val.fmt = VALUE_REF_VOLTAGE_NO_CORRECT; // Set format
  val.pin = 0;
  val.data = &measuredDiodeVoltage;
  interfaces[vrefIndex]->readPin(&val);

  printf("\n%sCalibrate ADC: Got calibration values:%s\n", YELLOW,
         NO_COLOR);
  printf("\tActual diode voltage\t  = %s%5.2f%sV\t±%s%.2f%sV\n",
         WHITE, actualDiodeVoltage, NO_COLOR,
         WHITE, actualDiodeTolerance, NO_COLOR);
  printf("\tMeasured diode voltage\t  = %s%5.2f%sV\t±%s%.2f%sV\n",
         WHITE, measuredDiodeVoltage, NO_COLOR,
         WHITE, adcTolerance, NO_COLOR);
  // offsetRatio = actualDiodeVoltage / diodeVoltage;
  avccActual = offsetRatio * avccTheoretical;
  // Gets a tolerance ratio; multiply by the voltage to get tolerance.
  // toleranceRatio = ((actualDiodeTolerance / actualDiodeVoltage) +
  //                   (adcTolerance / diodeVoltage));
  toleranceAtAvcc = toleranceRatio * avccActual;
  printf("\tOffset ratio          \t  = %s%5.2f%s\t±%s%4.2f%s\n",
         WHITE, offsetRatio, NO_COLOR,
         WHITE, toleranceRatio * 100, NO_COLOR);
  printf("\t%sAt measured %s%5.2f%sV actual = %s%5.2f%sV\t±%s%.2f%sV\n",
         NO_COLOR, // Text color
         WHITE, avccTheoretical, NO_COLOR,
         WHITE, avccActual, NO_COLOR,
         WHITE, toleranceAtAvcc, NO_COLOR);
  // Pack data into any interface which identifies as an ADC-based interface
  printf("%sCalibrate ADC: Storing values:%s\n", YELLOW,
         NO_COLOR);
  float data[2] = {offsetRatio, toleranceRatio};
  cfg.fmt = ICFG_ADC_OFFSET_AND_TOLERANCE_RATIOS; // Set format
  cfg.data = data;
  for (Interface_Indexer_t intf = Interface_Indexer_t(INTF_INVALID_, 0);
       intf.step(); ) {
    // Interface DNE, so don't check it
    if (interface_qty[intf.type] == 0) { continue; }
    for (intf.index = 0; intf.index < interface_qty[intf.type];
         intf.index++) {
      // Not an analog device interface
      if (interfaces[intf]->getParentTypeId() != DEVICE_ADC) {continue;}
      // Write data into interface:
      interfaces[intf]->writeConfig(&cfg);
      printf("\tData has been stored in %s%s%s interface, index %s%d%s.\n",
             WHITE, interfaces[intf]->getInterfaceName(), NO_COLOR,
             WHITE, intf.index, NO_COLOR);
    }
  }
} // calibrateAdc

/**
 * TODO: YAML
 */

void setupPowerLineReaders() {
  // TODO: FIX
  InterfaceConfig_t cfg; // For configuring interfaces
  Interface_Indexer_t vDiv_i = Interface_Indexer_t(INTF_VOLTAGE_DIVIDER, 0);

  // Prevent SEGFAULTS
  if (interface_qty[vDiv_i.type] == 0) {
    log_error_nargs("No voltage divider interfaces found.");
    return;
  }

  // These are all chosen based on the interface board REV A.
  printf("\n%sPower Line Startup: assigning resistor divider values.%s\n",
         YELLOW,
         NO_COLOR);

  // Technically in Ohms, but the ratio is all that matters.
  // To reduce error, measure actual resistors once installed, enter values, and
  // use a tolerance of zero.
  float highResistorValuesAndTolerances[4][2] = {
    {0, 0},
    {0, 0},
    {2.2, 0.05},
    {11,  0.05}};
  float lowResistorValuesAndTolerances[4][2] = {
    {0, 0},
    {0, 0},
    {1, 0.05},
    {1, 0.05}};

  printf("Voltage divider interfaces: %d\n", interface_qty[vDiv_i.type]);

  // For each V_DIV interface
  for (vDiv_i.index = 0; vDiv_i.index < interface_qty[vDiv_i.type];
       vDiv_i.index++) {
    cfg.fmt = ICFG_PL_HIGH_RESISTOR_WITH_TOLERANCE; // Set format
    cfg.data = highResistorValuesAndTolerances[vDiv_i.index];
    interfaces[vDiv_i]->writeConfig(&cfg);

    cfg.fmt = ICFG_PL_LOW_RESISTOR_WITH_TOLERANCE; // Set format
    cfg.data = lowResistorValuesAndTolerances[vDiv_i.index];
    interfaces[vDiv_i]->writeConfig(&cfg);

    printf("\tVoltage divider #%s%d%s assigned.",
           WHITE, vDiv_i.index, NO_COLOR);
  }
  printf("\n");
} // setupPowerLineReaders

void updateAllDevices() {
  Device_Indexer_t dev_i;
  dev_i.type = DEVICE_INVALID_;
  for ( ; dev_i.step(); ) {
    // No device was never created of this type, so it would segfault
    if (device_qty[dev_i.type] == 0) continue;
    for (dev_i.index = 0; dev_i.index < device_qty[dev_i.type]; dev_i.index++) {
      devices[dev_i]->updateData();
    }
  }
} // updateDevices

void startupConfig(){
  printf("CONFIGURING BOARD INTERFACE");
  updateAllDevices();

  calibrateAdc(); // Calibrate the ADCs based on the known real AVCC value.
  setupPowerLineReaders(); // Assign resitor values for the power line interfaces

  updateAllDevices();
} // startupConfig

void hardwareExit() {
  ros::shutdown();
} // hardwareExit

void dumpConfiguration(){
  Interface_Indexer_t intf_i; // Stores device type and index for map array
  Interface_Indexer_t intf_temp; // For checking other interfaces when formatting
  Device_Indexer_t dev_i; // Stores device type and index for map array
  dev_i.type = DEVICE_INVALID_; // Start with first device
  intf_i.type = INTF_INVALID_; // Start with first interface

  char SP[4] = "  "; // Spacing for the tree view
  bool moreDevfLeft = true;
  bool moreIntfLeft = true;
  bool morePinsLeft = true;
  printf("%s", YELLOW);
  for (int i = 0; i < 80; i++)
    printf(":");
  printf("%s\n", NO_COLOR);
  printf("\nCONFIGURATION DUMP:\n");
  // For each device type
  for ( ; dev_i.step(); ) {
    if (device_qty[dev_i.type] == 0) {
      // No device was never created of this type, so it would segfault
      continue;
    }
    printf("%s\n", deviceHardwareFunction(dev_i.type));
    for (dev_i.index = 0; dev_i.index < device_qty[dev_i.type]; dev_i.index++) {
      // Last device of type is end of tree
      if (dev_i.index == device_qty[dev_i.type] - 1) {
        printf("└─");
        moreDevfLeft = false;
      }	else {
        printf("├─"); // Formatting
        moreDevfLeft = true;
      }
      printf("Device #%s%d%s, Hardware: %s%s%s\n", WHITE, dev_i.index,
             NO_COLOR,
             devices[dev_i]->ready() ? GREEN :
             RED, devices[dev_i]->getHardwareName(),
             NO_COLOR);
      for (intf_i.type = INTF_INVALID_; intf_i.step(); ) {
        if (interface_qty[intf_i.type] == 0) {
          // No interface was never created of this type, so it would segfault
          continue;
        }
        for (intf_i.index = 0; intf_i.index < interface_qty[intf_i.type];
             intf_i.index++) {
          // If this interface is not for the device, just skip it.
          if (interfaces[intf_i]->getParentDeviceIndex() != dev_i) {
            continue;
          }
          // Check if this is the last interface on the tree (for formatting)
          moreIntfLeft = false; // Default: no more
          for (intf_temp = intf_i; intf_temp.step(); ) {
            if (interface_qty[intf_temp.type] == 0) { continue; }
            for (intf_temp.index = 0; intf_temp.index <
                 interface_qty[intf_temp.type];
                 intf_temp.index++) {
              // Check if the following interfaces wil be used
              if (interfaces[intf_temp]->getParentDeviceIndex() == dev_i) {
                moreIntfLeft = true; // Found another one, so break out
                break;
              }
            }
          }
          // Device branch and spacing
          printf("%s%s", moreDevfLeft ? "│" : " ", SP);
          // Interface branch
          printf("%s─", moreIntfLeft ? "├" : "└");
          printf("Interface #%s%d%s, Type: %s%s%s\n", WHITE, intf_i.index,
                 NO_COLOR, interfaces[intf_i]->ready() ? GREEN : RED,
                 interfaceName(intf_i.type),
                 NO_COLOR);
          // Device branch, spacing
          printf("%s%s", moreDevfLeft ? "│" : " ", SP);
          // Interface branch, spacing
          printf("%s%s", moreIntfLeft ? "│" : " ", SP);
          // Pins branch
          printf("└─");
          // Print interface info
          if (!interfaces[intf_i]->ready()) {
            printf("Interface not ready, no pins to show.\n");
            continue;
          }
          printf("Pin count: %s%d%s\n", WHITE,
                 interfaces[intf_i]->getPinCount(), NO_COLOR);
          morePinsLeft = true;
        }
      }
      // devices[dev_i]->getPinBus().getBusTypeString(true));
    }
  }
} // dumpConfiguration

/**
 * Reads commands from command line and assigns appropriate flags/data.
 */
void readCommands(int argc, char *argv[]) {
  bool show_help = false;
  if (argc > 0) {
    for (int arg = 0; arg < argc; arg++) {
      // Simulate hardware interfaces; will not actuually connect to hardware
      if ((!strcmp(argv[arg], "--sim")) || (!strcmp(argv[arg], "-s"))) {
        simulate_hw = true; // TODO
      }	else if ((!strcmp(argv[arg], "--test")) || (!strcmp(argv[arg], "-t"))) {
        use_bit_test = true;
      }	else {
        // ROS likes to pass a file path as an argument. This will ignore that path/
        if (argv[arg][0] == '/')
          continue;
        // This will show help, but first it will go therough each invalid argument.
        // Showing help will also kill the program.
        printf("Got invalid argument: %s\n", argv[arg]);
        show_help = true;
      }
    }
  }
  if (show_help) {
    printf("Valid board interface arguemnts: \n");
    printf(
      "\t-s\t--sim\tSimulate hardware without connecting to the board.\n");
    printf(
      "\t-t\t--test\tUse the built-in-testing to test the interfaces. DO NOT LEAVE HARDWARE CONNECTED TO THE BOARD!\n");
    raise(SIGTERM); // Kill program
  }

  // Simulated Hardware
  if (!simulate_hw)
    log_info_nargs("--sim not specified, not simulating hardware.");
  else
    log_info_nargs("Simulating I/O, no hardware will be used.");

  // Bit testing
  if (!use_bit_test)
    log_info_nargs("--test not specified, not running built in tests.");
  else
    log_warn_nargs(
      "Using BIT tests. WARNING: DISCONNECT ALL HARDWARE FROM THE BOARD BEFORE PROCEDING!");
} // readCommands

void enterLoop() {
  // Connect ROS
  printf("Starting up ROS.\n");
  // Start ROS and get the node instance
  ros::NodeHandle nd;
  // Print the node name
  std::string nodeName = ros::this_node::getName();
  printf("Node name: %s\n", nodeName.c_str());

  // Set how many publishers to create

  // Set up the message publishers
  Publisher_Indexer_t pub;
  std::string topic_name = "";
  char index_number = 0;
  // For each type of publisher
  for (pub.type = PUB_INVALID; pub.step(); ) {
    publisher_qty[pub.type] = interface_qty[pub.getInterfaceType()];
    if (pub.type == PUB_TELEMETRY)
      publisher_qty[pub.type] = 1;
    if (publisher_qty[pub.type] < 1) {
      // No publishers of this type, so don't try anything.
      continue;
    }
    // For each interface that needs publishing
    for (pub.index = 0; pub.index < publisher_qty[pub.type];
         pub.index++) {
      topic_name = pub.getTopicName();
      printf("Topic name: %s\n", topic_name.c_str());
      // Create the publisher, based on the type to use
      switch (pub.type) {
      case PUB_GPIO:
        publishers[pub] =
          nd.advertise <board_interface::gpio> (topic_name.c_str(), 0);
        break;
      case PUB_PWM:
        publishers[pub] =
          nd.advertise <board_interface::pwm> (topic_name.c_str(), 0);
        break;
      case PUB_ADC:
        publishers[pub] =
          nd.advertise <board_interface::adc> (topic_name.c_str(), 0);
        break;
      case PUB_LEAK:
        publishers[pub] =
          nd.advertise <board_interface::leak> (topic_name.c_str(), 0);
        break;
      case PUB_POWER:
        publishers[pub] =
          nd.advertise <board_interface::power> (topic_name.c_str(), 0);
        break;
      case PUB_TELEMETRY:
        publishers[pub] =
          nd.advertise <board_interface::telemetry> (topic_name.c_str(), 0);
      default:
        break;
      } // switch
    }
  }
  // ros::Publisher gpio_publisher[3];
// gpio_publisher[0] = nd.advertise <board_interface::gpio> ("gpio_0", 0);
  // Allows for a 1 second delay between messages
  ros::Duration loop_wait(1);
  // For storing messages temporarily
  board_interface::gpio gpio_msg;
  board_interface::pwm pwm_msg;
  board_interface::adc adc_msg;
  board_interface::leak leak_msg;
  board_interface::power power_msg;
  board_interface::telemetry telemetry_msg;
  // Since we are just pushing out data, no writes are performed, so R/W masks are 0
  gpio_msg.rw_mask = 0;
  pwm_msg.rw_mask = 0;
  power_msg.rw_mask = 0;
  telemetry_msg.em_io_rw_mask = 0;

  // These will be used to access the interfaces and their data
  Interface_Indexer_t intf_i;
  PinValue_t pin_value_reader; // Reads pin value
  float data[17]; // Stores the data for all pins + one extra for PWM frequency
  pin_value_reader.data = data; // Pointer to data
  pin_value_reader.fmt = VALUE_ROS_DATA_; // Gets all data from all interface pins
  pin_value_reader.pin = 0;

  uint8_t pin_i; // For iterating through devices. Kept out of loop for efficiency.

  while (ros::ok()) {
    // Update data (is done in the switch)
    for (pub.type = PUB_INVALID; pub.step(); ) {
      publisher_qty[pub.type] = interface_qty[pub.getInterfaceType()];
      if (pub.type == PUB_TELEMETRY)
        publisher_qty[pub.type] = 1;
      if (publisher_qty[pub.type] < 1) {
        // No publishers of this type, so don't try anything.
        continue;
      }

      intf_i.type = pub.getInterfaceType(); // Load the type into the data for the interface

      // Publish data, based on the type to use
      switch (pub.type) {
      case PUB_GPIO: // Pet the dog
        // For each interface that needs publishing
        for (pub.index = 0; pub.index < publisher_qty[pub.type];
             pub.index++) {
          intf_i.index = pub.index; // Assign index
          interfaces[intf_i]->readPin(&pin_value_reader); // Get data from interface
          gpio_msg.modes = (uint16_t)data[0]; // Modes
          gpio_msg.values = (uint16_t)data[1]; // States
          gpio_msg.header.stamp = ros::Time::now(); // TODO: this must be accessed from device!
          publishers[pub].publish(gpio_msg); // publish
        }
        break;
      case PUB_PWM:
        // For each interface that needs publishing
        for (pub.index = 0; pub.index < publisher_qty[pub.type];
             pub.index++) {
          intf_i.index = pub.index; // Assign index
          interfaces[intf_i]->readPin(&pin_value_reader); // Get data from interface
          pwm_msg.frequency = data[16];
          for (pin_i = 0; pin_i < 16; pin_i++)
            pwm_msg.values[pin_i] = data[pin_i];
          pwm_msg.header.stamp = ros::Time::now(); // TODO: this must be accessed from device!
          publishers[pub].publish(pwm_msg); // publish
        }
        break;
      case PUB_ADC:
        // For each interface that needs publishing
        for (pub.index = 0; pub.index < publisher_qty[pub.type];
             pub.index++) {
          intf_i.index = pub.index; // Assign index
          interfaces[intf_i]->readPin(&pin_value_reader); // Get data from interface
          for (pin_i = 0; pin_i < 8; pin_i++) {
            adc_msg.values[pin_i] = data[pin_i];
            // Tolerance is stored in second half of array
            adc_msg.values_tolerance[pin_i] = data[8 + pin_i];
          }
          adc_msg.header.stamp = ros::Time::now(); // TODO: this must be accessed from device!
          publishers[pub].publish(adc_msg); // publish
        }
        break;
      case PUB_LEAK:
        // For each interface that needs publishing
        for (pub.index = 0; pub.index < publisher_qty[pub.type];
             pub.index++) {
          intf_i.index = pub.index; // Assign index
          interfaces[intf_i]->readPin(&pin_value_reader); // Get data from interface
          leak_msg.values = (uint16_t)data[0];
          leak_msg.header.stamp = ros::Time::now(); // TODO: this must be accessed from device!
          publishers[pub].publish(leak_msg); // publish
        }
        break;
      case PUB_POWER:
        // For each interface that needs publishing
        for (pub.index = 0; pub.index < publisher_qty[pub.type];
             pub.index++) {
          intf_i.index = pub.index; // Assign index
          interfaces[intf_i]->readPin(&pin_value_reader); // Get data from interface
          power_msg.values = (uint16_t)data[0];
          power_msg.header.stamp = ros::Time::now(); // TODO: this must be accessed from device!
          publishers[pub].publish(power_msg); // publish
        }
        break;
      case PUB_TELEMETRY:
        // For each interface that needs publishing
        for (pub.index = 0; pub.index < publisher_qty[pub.type];
             pub.index++) {
          // Telemetry must read from many interfaces.
          // TEMPERATURE //////////////////////////////////////////////////////
          intf_i.type = INTF_TEMP;
          intf_i.index = 0;
          interfaces[intf_i]->readPin(&pin_value_reader); // Get data from interface
          telemetry_msg.temperature_deg_c = data[0];
          telemetry_msg.temperature_deg_c_tolerance = data[1];
          // CURRENTx2 ////////////////////////////////////////////////////////
          intf_i.type = INTF_CURRENT;
          for (intf_i.index = 0; intf_i.index < 2; intf_i.index++) {
            interfaces[intf_i]->readPin(&pin_value_reader); // Get data from interface
            telemetry_msg.current_amps[intf_i.index] = data[0];
            telemetry_msg.current_amps_tolerance[intf_i.index] = data[1];
          }
          // POWER LINE x4 ////////////////////////////////////////////////////
          intf_i.type = INTF_VOLTAGE_DIVIDER;
          for (intf_i.index = 0; intf_i.index < 4; intf_i.index++) {
            interfaces[intf_i]->readPin(&pin_value_reader); // Get data from interface
            telemetry_msg.power_line_volts[intf_i.index] = data[0];
            telemetry_msg.power_line_volts_tolerance[intf_i.index] = data[1];
          }
          // SWITCHES /////////////////////////////////////////////////////////
          // intf_i.type = INTF_SWITCHES;
          // Switches are not inplemented yet
          // EMERGENCY IO /////////////////////////////////////////////////////
          intf_i.type = INTF_EMERGENCY_IO;
          intf_i.index = 0;
          interfaces[intf_i]->readPin(&pin_value_reader); // Get data from interface
          telemetry_msg.em_io = (uint16_t)data[0];
          // Publish the data /////////////////////////////////////////////////
          publishers[pub].publish(telemetry_msg); // Publish the data
        }
        break;
      default:
        break;
      } // switch
    }
    updateAllDevices(); // TODO: Does this need to happen more often?
    loop_wait.sleep();
  }
} // enterLoop

int main(int argc, char *argv[]){
  atexit(hardwareExit);

  readCommands(argc, &argv[0]);

  createAndInitDevices(); // Setup Devices
  createAndInitInterfaces(); // Setup Interfaces
  // YAML CONFIG GOES HERE
  // All set; dump data
  dumpConfiguration(); // False for full pin listing
  // bit_testing::dumpConfiguration(true, &interfaces, &devices); // False for full pin listing
  startupConfig();
  // if (use_bit_test) // If the argument to ensable BIT tests was passed, run them
  //  runBitTest(); // Test interfaces

  ros::init(argc, argv, "board_interface");
  // Startup complete; start looping
  enterLoop();
} // main
