/*
 * Motor.cpp
 *
 *  Created on: Jan 10, 2018
 *      Author: aron
 */


#include "Motor.hpp"

Motor::Motor(uint8_t id , DynamixelControl &dynamixelControl, LegPart part)
:dynamixelControl(dynamixelControl), part(part){
	this->id = id;
}

void Motor::set_angle(uint16_t angle){
	this->dynamixelControl.set_servo_position(angle, this->id);
}

void Motor::set_speed(uint16_t rpm){
	this->dynamixelControl.set_servo_speed(rpm, this->id);
}
