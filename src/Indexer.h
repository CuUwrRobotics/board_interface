/**
 * @Author: Nick Steele <nichlock>
 * @Date:   9:08 Aug 15 2020
 * @Last modified by:   nichlock
 * @Last modified time: 19:21 Sep 19 2020
 */

#ifndef INDEXER_H
#define INDEXER_H

#include <stdint.h>
#include <string>
#include "HardwareDescription.h"
// #include <map> // Mot used here, but necesary for the maps
// #include <utility> // TODO: do we need this?

/**
 * These two structs are used for indexing the interface and device maps. Using a
 * simple integer was not practical, since it made refrencing a specific device
 * inefficient, and other solutions were too clunky.
 *
 * These structs can be used directly to map interfaces or devices.
 */

// TODO: Create an iterator function capable of iterating accross types AND
// indexes, as well as catching when it is done.

struct Interface_Indexer_t {
public:

  bool step(){
    if (type != INTF_LAST_) {
      type = (Interface_t)(((int)type) + 1);
      if (type != INTF_LAST_)
        return true;
    }
    return false;
  } /* step */

  const char *toString(){
    std::string name;
    name = interfaceIdToCharArray(type);
    name += " #";
    name += (char) ('0' + index);
    return name.c_str();
  } /* toCharArray */

  // For inline definitions
  Interface_Indexer_t(Interface_t t, uint8_t i) {
    type = t; index = i;
  };
  // For non-inline definitions
  Interface_Indexer_t() {
  };
  Interface_t type;
  uint8_t index; // Must be < 255
  // Required for map array. Works by prioritizing type first, then index second.
  bool operator < (const Interface_Indexer_t &t) const {
    return ((((uint16_t)this->type) << 8) | (this->index)) <
           ((((uint16_t)t.type) << 8) | (t.index));
  }
  bool operator == (const Interface_Indexer_t &t) const {
    return (this->type == t.type) && (this->index == t.index);
  }
  bool operator != (const Interface_Indexer_t &t) const {
    return !(*this == t);
  }
};

struct Device_Indexer_t {
public:

  bool step(){
    if (type != DEVICE_LAST_) {
      type = (Device_t)(((int)type) + 1);
      if (type != DEVICE_LAST_)
        return true;
    }
    return false;
  } /* step */

  const char *toString(){
    std::string name;
    name = deviceIdToCharArray(type);
    name += " #";
    name += (char) ('0' + index);
    return name.c_str();
  } /* toCharArray */

  Device_Indexer_t(Device_t t, uint8_t i) {
    type = t; index = i;
  };
  // For non-inline definitions
  Device_Indexer_t() {
  };
  Device_t type;
  uint8_t index; // Must be < 255
  // Required for map array. Works by prioritizing type first, then index second.
  bool operator < (const Device_Indexer_t &t) const {
    return ((((uint16_t)this->type) << 8) | (this->index)) <
           ((((uint16_t)t.type) << 8) | (t.index));
  }
  bool operator == (const Device_Indexer_t &t) const {
    return (this->type == t.type) && (this->index == t.index);
  }
  bool operator != (const Device_Indexer_t &t) const {
    return !(*this == t);
  }
};

#endif /* end of include guard: INDEXER_H */
