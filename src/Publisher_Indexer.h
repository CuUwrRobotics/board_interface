#ifndef PUBLISHERS_H
#define PUBLISHERS_H

#include <stdint.h>
#include <string>
#include "HardwareDescription.h"

enum Publisher_t {
	PUB_INVALID,
	PUB_GPIO,
	PUB_PWM,
	PUB_ADC,
	PUB_LEAK,
	PUB_POWER,
	PUB_TELEMETRY,
	PUB_LAST_
};

struct Publisher_Indexer_t {
public:

	bool step(){
		if (type != PUB_LAST_) {
			type = (Publisher_t)(((int)type) + 1);
			if (type != PUB_LAST_)
				return true;
		}
		return false;
	} /* step */

	Interface_t getInterfaceType() {
		switch (type) {
		case PUB_GPIO:
			return INTF_GPIO;
			break;
		case PUB_PWM:
			return INTF_PWM;
			break;
		case PUB_ADC:
			return INTF_ADC;
			break;
		case PUB_LEAK:
			return INTF_LEAK;
			break;
		case PUB_POWER:
			return INTF_PWR_SWITCHING;
			break;
		case PUB_TELEMETRY:
			return INTF_INVALID_; // No telemetry interface
			break;
		default:
			return INTF_INVALID_;
			break;
		} /* switch */
	} /* getInterfaceType */

	std::string getTopicName(){
		std::string topic_name = "";
		char index_char = 0;
		switch (type) {
		case PUB_GPIO:
			topic_name = "gpio_";
			break;
		case PUB_PWM:
			topic_name = "pwm_";
			break;
		case PUB_ADC:
			topic_name = "adc_";
			break;
		case PUB_LEAK:
			topic_name = "leak_";
			break;
		case PUB_POWER:
			topic_name = "power_";
			break;
		case PUB_TELEMETRY:
			topic_name = "telemetry_";
			break;
		default:
			return "error"; // prevents crashes from requesting invalid names
			break;
		} /* switch */
		index_char = '0' + index;
		topic_name += index_char;
		return topic_name;
	} /* toCharArray */

	// For inline definitions
	Publisher_Indexer_t(Publisher_t t, uint8_t i) {
		type = t; index = i;
	};
	// For non-inline definitions
	Publisher_Indexer_t() {
	};
	Publisher_t type;
	uint8_t index; // Must be < 255
	// std::string topicString;
	// Required for map array. Works by prioritizing type first, then index second.
	bool operator < (const Publisher_Indexer_t &t) const {
		return ((((uint16_t)this->type) << 8) | (this->index)) <
		       ((((uint16_t)t.type) << 8) | (t.index));
	}
	bool operator == (const Publisher_Indexer_t &t) const {
		return (this->type == t.type) && (this->index == t.index);
	}
	bool operator != (const Publisher_Indexer_t &t) const {
		return !(*this == t);
	}
};

#endif /* end of include guard: PUBLISHERS_H */
