/*
 * DynamixelControl.cpp
 *
 *  Created on: Dec 13, 2017
 *      Author: aron
 */
#include "DynamixelControl.hpp"
#include "DynamixelInstructions.hpp"

DynamixelControl::DynamixelControl(uint32_t baudRate) : DynamixelSendReceive(baudRate){}

//takes degrees but calculates to 0.29 degree increments (0-1023)
void DynamixelControl::set_servo_position(uint16_t positionInDegrees, uint8_t servoID){
	std::vector<uint8_t> instruction;
	uint16_t positionInIncrements = positionInDegrees / this->degreeToIncrements;
	uint8_t lowByte = positionInIncrements & 0xFF;
	uint8_t highByte = positionInIncrements >> 8;
	instruction.push_back(servoID);
	instruction.push_back(dynamixel::special::servoPositionLength);
	instruction.push_back(dynamixel::command::write);
	instruction.push_back(dynamixel::address::ram::goalPositionLow);
	instruction.push_back(lowByte);
	instruction.push_back(highByte);
	DynamixelSendReceive::transmit_instruction(instruction);
	this->wait_for_status_packet(servoID);
}

//in joint mode it is 0.111 rpm per increment | 0  means max speed
void DynamixelControl::set_servo_speed(uint16_t speedInRPM, uint8_t servoID){
	std::vector<uint8_t> instruction;
	uint16_t speedInIncrements = speedInRPM / this->rpmToIncrements;
	uint8_t lowByte = speedInIncrements & 0xFF;
	uint8_t highByte = speedInIncrements >> 8;
	instruction.push_back(servoID);
	instruction.push_back(dynamixel::special::servoSpeedLength);
	instruction.push_back(dynamixel::command::write);
	instruction.push_back(dynamixel::address::ram::movingSpeedLow);
	instruction.push_back(lowByte);
	instruction.push_back(highByte);
	DynamixelSendReceive::transmit_instruction(instruction);
	this->wait_for_status_packet(servoID);
}

void DynamixelControl::wait_for_status_packet(uint8_t id){
	/*Disabled for now, kinematic equations are more important then figuring this out
	 * if(id != dynamixel::special::broadcastId){
		DynamixelSendReceive::check_status_packet();
	}
	*/
}
