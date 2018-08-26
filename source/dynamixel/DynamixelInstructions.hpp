/*
 * DynamixelAdresses.hpp
 *
 *  Created on: Nov 25, 2017
 *      Author: aron
 */

#ifndef DYNAMIXELINSTRUCTIONS_HPP_
#define DYNAMIXELINSTRUCTIONS_HPP_

#include <stdint.h>
namespace dynamixel{

	namespace address {

		namespace eeprom{
			static constexpr uint8_t modelNumberLow = 0x00;
			static constexpr uint8_t modelNumberHigh = 0x01;
			static constexpr uint8_t version = 0x02;
			static constexpr uint8_t id = 0x03;
			static constexpr uint8_t baudRate= 0x04;
			static constexpr uint8_t returnDelayTime = 0x05;
			static constexpr uint8_t clockWiseAngleLimitLow = 0x06;
			static constexpr uint8_t clockWiseAngleLimitHigh = 0x07;
			static constexpr uint8_t counterClockWiseAngleLimitLow = 0x08;
			static constexpr uint8_t counterClockWiseAngleLimitHigh = 0x09;
			static constexpr uint8_t temperatureLimit = 0x0B;
			static constexpr uint8_t lowVoltageLimit = 0x0C;
			static constexpr uint8_t highVoltageLimit = 0x0D;
			static constexpr uint8_t maxTorqueLow = 0x0E;
			static constexpr uint8_t maxTorqueHigh = 0x0F;
			static constexpr uint8_t statusReturnLevel = 0x10;
			static constexpr uint8_t alarmLed = 0x11;
			static constexpr uint8_t alarmShutdown = 0x12;
		}

		namespace ram{
			static constexpr uint8_t torqueEnable = 0x18;
			static constexpr uint8_t led = 0x19;
			static constexpr uint8_t clockWiseComplianceMargin = 0x1A;
			static constexpr uint8_t counterClockWiseComplianceMargin = 0x1B;
			static constexpr uint8_t clockWiseComplianceSlope = 0x1C;
			static constexpr uint8_t counterClockWiseComplianceSlope = 0x1D;
			static constexpr uint8_t goalPositionLow = 0x1E;
			static constexpr uint8_t goalPositionHigh = 0x1F;
			static constexpr uint8_t movingSpeedLow = 0x20;
			static constexpr uint8_t movingSpeedHigh = 0x21;
			static constexpr uint8_t torqueLimitLow = 0x22;
			static constexpr uint8_t torqueLimitHigh = 0x23;
			static constexpr uint8_t presentPositionLow = 0x24;
			static constexpr uint8_t presentPositionHigh = 0x25;
			static constexpr uint8_t presentSpeedLow = 0x26;
			static constexpr uint8_t presentSpeedHigh = 0x27;
			static constexpr uint8_t presentLoadLow = 0x28;
			static constexpr uint8_t presentLoadHigh = 0x29;
			static constexpr uint8_t presentVoltage = 0x2A;
			static constexpr uint8_t presentTemperature = 0x2B;
			static constexpr uint8_t isRegistered = 0x2C;
			static constexpr uint8_t isMoving = 0x2E;
			static constexpr uint8_t lockEeprom = 0x2F;
			static constexpr uint8_t punchLow = 0x30;
			static constexpr uint8_t punchHigh = 0x31;
		}
	}

	namespace special{
		static constexpr uint8_t header = 0xFF;
		static constexpr uint8_t broadcastId = 0xFE;
		static constexpr uint8_t on = 0x01;
		static constexpr uint8_t off = 0x00;
		static constexpr uint8_t left = 0x00;
		static constexpr uint8_t right = 0x01;
		static constexpr uint8_t headerLength = 0x02;
		static constexpr uint8_t addressLength = 0x01;
		static constexpr uint8_t uartErrorLength = 0x01;
		static constexpr uint8_t errorPositon = 0x02;
		static constexpr uint8_t servoPositionLength = 0x05;
		static constexpr uint8_t servoSpeedLength = 0x05;
	}

	namespace error{
		static constexpr uint8_t voltage = 0b00000001;
		static constexpr uint8_t angleLimit = 0b00000010;
		static constexpr uint8_t overHeat =  0b00000100;
		static constexpr uint8_t commandOutOffRange =  0b00001000;
		static constexpr uint8_t checksum =  0b00010000;
		static constexpr uint8_t overload =  0b00100000;
		static constexpr uint8_t instruction =  0b01000000;
		static constexpr uint8_t receive =  0b10000000;
		static constexpr uint8_t uart = 0b11111111;
		static constexpr uint8_t noError = 0b00000000;
	}

	namespace command{
		static constexpr uint8_t ping = 0x01;
		static constexpr uint8_t read = 0x02;
		static constexpr uint8_t write = 0x03;
		static constexpr uint8_t registerWrite = 0x04;
		static constexpr uint8_t action = 0x05;
		static constexpr uint8_t reset = 0x06;
		static constexpr uint8_t syncWrite = 0x83;
	}
}
#endif /* DYNAMIXELINSTRUCTIONS_HPP_ */
