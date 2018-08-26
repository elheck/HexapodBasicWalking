/*
 * DynamixelControl.hpp
 *
 *  Created on: Dec 13, 2017
 *      Author: aron
 */

#ifndef DYNAMIXEL_DYNAMIXELCONTROL_HPP_
#define DYNAMIXEL_DYNAMIXELCONTROL_HPP_

#include "DynamixelSendReceive.hpp"

class DynamixelControl : public DynamixelSendReceive {
public:
	DynamixelControl(uint32_t baudRate);
	virtual ~DynamixelControl(){}
	void set_servo_position(uint16_t positionInDegrees, uint8_t servoID);
	void set_servo_speed(uint16_t speedInRPM, uint8_t servoID);

private:
	static constexpr float degreeToIncrements = 0.29;//see robotis manual
	static constexpr float rpmToIncrements = 0.111;//see robotis manual
	void wait_for_status_packet(uint8_t id);
};



#endif /* DYNAMIXEL_DYNAMIXELCONTROL_HPP_ */
