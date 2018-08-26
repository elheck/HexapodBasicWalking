/*
 * Modes.hpp
 *
 *  Created on: Jan 21, 2018
 *      Author: aron
 */

#ifndef DIRECTIONALCONTROL_MODES_HPP_
#define DIRECTIONALCONTROL_MODES_HPP_

enum class WalkingGait{
	WAVE,
	RIPPLE,
	TRIPOD,
	RULE_BASED,
	NONE,
	BROKEN
};

enum class Mode{
	WALKING,
	RISE_BODY,
	WAVE,
	BROKEN
};


#endif /* DIRECTIONALCONTROL_MODES_HPP_ */
