/*
 * Motor.hpp
 *
 *  Created on: Jan 10, 2018
 *      Author: aron
 */

#ifndef ROBOT_MOTOR_HPP_
#define ROBOT_MOTOR_HPP_

#include "dynamixel/DynamixelControl.hpp"
#include "Parts.hpp"

class Motor {
public:
	Motor(uint8_t id , DynamixelControl &dynamixelControl, LegPart part);
	void set_angle(uint16_t angle);
	void set_speed(uint16_t rpm);
	virtual ~Motor(){};

private:
	uint8_t id;
	DynamixelControl& dynamixelControl;
	LegPart part;

};




#endif /* ROBOT_MOTOR_HPP_ */
