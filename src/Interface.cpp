/**
 * @Author: Nick Steele <nichlock>
 * @Date:   9:08 Aug 15 2020
 * @Last modified by:   nichlock
 * @Last modified time: 19:21 Sep 19 2020
 */

 #include "Interface.h"

// Device *commDevice; // The device to actually communicate using
// PinBus pinBus;
//
// bool initerrorVal = false;
// bool commDeviceExists = false;
// uint8_t interfaceIndex = 0xFF; // Index of the interface object

bool Interface::start(Device *device, PinBus pb, Interface_Indexer_t
                      ifaceIndex) {
  initerrorVal = false;
  commDevice = device;
  commDeviceExists = true;
  if (getParentTypeId() != commDevice->getDeviceTypeId()) {
    log_error("Parent type ID bad for interface %s. Expected: %s, Got: %s\n",
              interfaceIndex.toString(),
              deviceIdToCharArray(getParentTypeId()),
              deviceIdToCharArray(commDevice->getDeviceTypeId()));
    return false;
  }
  // Check that device is initialized
  if (!commDevice->ready()) {
    printf("ERROR: parentDevice not ready\n");
    return false;
  }
  // Check that devicePins is not longer than getPinCount()
  if (pb.getPinCount() != getPinCount()) {
    printf("ERROR: device pins size bad: %d, pin count = %d\n",
           pb.getPinCount(),
           getPinCount());
    return false;
  }
  // Some variable setups
  interfaceIndex = ifaceIndex;
  pinBus = pb;

  if (commDevice->attachInterface(pinBus, getInterfaceTypeId())) {
    prepareInterface(); // Set interface-specific default modes to pinBus
    initerrorVal = true;
  } else log_error("Interface %s (%s) could not attatch to the parent device.",
                   interfaceIndex.toString(), getInterfaceName());
  return initerrorVal;

  // return true
} /* start */
