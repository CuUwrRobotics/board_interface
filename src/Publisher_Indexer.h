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
		case PUB_TELEMETRY:
			return INTF_INVALID; // No telemetry interface
			break;
		default:
			return INTF_INVALID;
			break;
		} /* switch */
	} /* getInterfaceType */

	const std::string getTopicName(){
		switch (type) {
		case PUB_GPIO:
			topicString = "gpio_" + (index + '0');
			break;
		case PUB_PWM:
			topicString = "pwm_" + (index + '0');
			break;
		case PUB_ADC:
			topicString = "adc_" + (index + '0');
			break;
		case PUB_TELEMETRY:
			topicString = "telemetry";
			break;
		default:
			return "error_getting_topic_name";
			break;
		} /* switch */
		return topicString;
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
	std::string topicString;
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
