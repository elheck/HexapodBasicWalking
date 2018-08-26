/*
 * Parts.hpp
 *
 *  Created on: Feb 26, 2018
 *      Author: aron
 */

#ifndef ROBOT_PARTS_HPP_
#define ROBOT_PARTS_HPP_

enum class LegPart{
	COXA,
	FEMUR,
	TIBIA
};

enum class LegPosition{
	LEFT_FRONT,
	LEFT_MIDDLE,
	LEFT_BACK,
	RIGHT_FRONT,
	RIGHT_MIDDLE,
	RIGHT_BACK
};

enum class LegPhase{
	SWING,
	SUPPORT,
	UNASSIGNED
};

#endif /* ROBOT_PARTS_HPP_ */
