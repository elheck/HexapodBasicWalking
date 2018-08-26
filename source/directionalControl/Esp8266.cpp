/*
 * esp8266.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: aron
 */

#include "Esp8266.hpp"
#include "esp_driver_compatability.h"

#define CONFIG_HIGH 0
#define CONFIG_LOW 1
#define LX_HIGH 2
#define LX_LOW 3
#define LY_HIGH 4
#define LY_LOW 5
#define RX_HIGH 6
#define RX_LOW 7
#define RY_HIGH 8
#define RY_LOW 9
#define MODE_WALKING 1
#define MODE_RISE_BODY 2
#define MODE_WAVE 4
#define GAIT_WAVE 128
#define GAIT_RIPPLE 64
#define GAIT_TRIPOD 32
#define BYTE_SHIFT 8

#define MAX_STICK_VALUE 1024



Esp8266::Esp8266(uint32_t baudrate) : bufferVector(){
	init_driver_compatability(baudrate);
	this->receivedLength = 0;
}

DirectionVector Esp8266::get_vector(){
	DirectionVector vec;
	get_arrived_data(this->data, &receivedLength);
	uint16_t lx = 0;
	uint16_t ly = 0;
	uint16_t rx = 0;
	uint16_t ry = 0;
	WalkingGait currentGait = WalkingGait::NONE;
	Mode currentMode = Mode::BROKEN;
	if(this->receivedLength != BUFFER_SIZE + BUFFER_SIZE){
		for(int i = 0; i < int(receivedLength / MESSAGE_LENGTH); i++){
			uint8_t offset = i * MESSAGE_LENGTH;
			lx += uint16_t((data[LX_HIGH + offset] << BYTE_SHIFT) | (data[LX_LOW + offset]));
			ly += uint16_t((data[LY_HIGH + offset] << BYTE_SHIFT) | (data[LY_LOW + offset]));
			rx += uint16_t((data[RX_HIGH + offset] << BYTE_SHIFT) | (data[RX_LOW + offset]));
			ry += uint16_t((data[RY_HIGH + offset] << BYTE_SHIFT) | (data[RY_LOW + offset]));
		}
		lx /= uint16_t(receivedLength / MESSAGE_LENGTH);
		ly /= uint16_t(receivedLength / MESSAGE_LENGTH);
		rx /= uint16_t(receivedLength / MESSAGE_LENGTH);
		ry /= uint16_t(receivedLength / MESSAGE_LENGTH);
		currentGait = get_current_gait(data[CONFIG_HIGH]);
		currentMode = get_current_mode(data[CONFIG_LOW]);

		vec = this->safety_check(DirectionVector(lx, ly, rx, ry, currentGait, currentMode));
	}else{
		vec = DirectionVector(lx, ly, rx, ry, currentGait, currentMode);
	}
	return vec;
}

DirectionVector Esp8266::safety_check(DirectionVector vec){
	DirectionVector returnVector = vec;
	if(vec.get_leftX() > MAX_STICK_VALUE ||
		vec.get_leftY() > MAX_STICK_VALUE ||
		vec.get_rightX() > MAX_STICK_VALUE ||
		vec.get_rightY() > MAX_STICK_VALUE ||
		vec.get_mode() == Mode::BROKEN ||
		vec.get_walking_gait() == WalkingGait::BROKEN){
		returnVector = this->bufferVector;
	}else{
		this->bufferVector.set_leftX(vec.get_leftX());
		this->bufferVector.set_leftY(vec.get_leftY());
		this->bufferVector.set_rightX(vec.get_rightX());
		this->bufferVector.set_rightY(vec.get_rightY());
		this->bufferVector.set_gait(vec.get_walking_gait());
		this->bufferVector.set_mode(vec.get_mode());
	}
	return returnVector;
}

Mode Esp8266::get_current_mode(uint8_t& lowByte){
	Mode mode;
	if(lowByte == MODE_RISE_BODY){
		mode = Mode::RISE_BODY;
	}else if(lowByte == MODE_WALKING){
		mode = Mode::WALKING;
	}else if(lowByte == MODE_WAVE){
		mode = Mode::WAVE;
	}else{
		mode = Mode::BROKEN;
	}
	return mode;

}

WalkingGait Esp8266::get_current_gait(uint8_t& highByte){
	WalkingGait gait;
	if(highByte == GAIT_WAVE){
		gait = WalkingGait::WAVE;
	}else if(highByte == GAIT_TRIPOD){
		gait = WalkingGait::TRIPOD;
	}else if(highByte == GAIT_RIPPLE){
		gait = WalkingGait::RIPPLE;
	}else{
		gait = WalkingGait::BROKEN;
	}
	return gait;
}



