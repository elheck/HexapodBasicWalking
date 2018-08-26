/*
 * DynamixelControl.cpp
 *
 *  Created on: Nov 20, 2017
 *      Author: aron
 */

#include "DynamixelSendReceive.hpp"
#include "DynamixelInstructions.hpp"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include <cassert>

#ifndef DYNAMIXEL_DIR_GPIO
	#define DYNAMIXEL_DIR_GPIO GPIOD
	#define DYNAMIXEL_DIR_GPIO_PIN 4U
#endif

#define OSCERCLK_SOURCE 2U //see documentation page 269 (SIM_SOPT2 field)
#define DYNAMIXEL_UART LPUART2

DynamixelSendReceive::DynamixelSendReceive(uint32_t baudRate){
	set_up_uart(baudRate);
}

DynamixelSendReceive::DynamixelSendReceive(){
	set_up_uart(115200);
}

void DynamixelSendReceive::set_up_uart(uint32_t baudRate){
	lpuart_config_t config;
	CLOCK_SetLpuartClock(OSCERCLK_SOURCE);
	LPUART_GetDefaultConfig(&config);
	config.baudRate_Bps = baudRate;
	config.enableRx = true;
	config.enableTx = true;
	config.parityMode = kLPUART_ParityDisabled;
	config.stopBitCount = kLPUART_OneStopBit;
	uint32_t clockFrequency = CLOCK_GetFreq(kCLOCK_Osc0ErClk);
	LPUART_Init(DYNAMIXEL_UART, &config, clockFrequency);
}

void DynamixelSendReceive::transmit_instruction(std::vector<uint8_t> instruction){
	this->set_connection_to_transmit();
	std::vector<uint8_t> packet( this->construct_packet(instruction));
	this->send_blocking(packet);
	this->set_connection_to_receive();
}

void DynamixelSendReceive::set_connection_to_transmit(){
	GPIO_ClearPinsOutput(DYNAMIXEL_DIR_GPIO, 1 << DYNAMIXEL_DIR_GPIO_PIN);
}

void DynamixelSendReceive::set_connection_to_receive(){
	GPIO_SetPinsOutput(DYNAMIXEL_DIR_GPIO, 1 << DYNAMIXEL_DIR_GPIO_PIN);
}

//instruction is ID|LENGTH|INSTRUCTION|PARAMETERS -> length is number of parameters +2
std::vector<uint8_t> DynamixelSendReceive::construct_packet(std::vector<uint8_t> &instruction){
	assert(instruction.size() >= 3);
	std::vector<uint8_t> packet;
	packet.push_back(dynamixel::special::header);
	packet.push_back(dynamixel::special::header);
	for(uint8_t& byte : instruction){
		packet.push_back(byte);
	}
	packet.push_back(this->calculate_checksum(instruction));
	return packet;
}

uint8_t DynamixelSendReceive::calculate_checksum(std::vector<uint8_t> &instruction){
	uint32_t checksum = 0;
	for(auto& byte : instruction){
		checksum += byte;
	}
	return uint8_t(~checksum & 0xFF);
}

void DynamixelSendReceive::send_blocking(std::vector<uint8_t> &packet){
	for(uint8_t& byte : packet){
		while (!(DYNAMIXEL_UART->STAT & LPUART_STAT_TDRE_MASK)){}
		LPUART_WriteByte(DYNAMIXEL_UART, byte);
	}
	while(!(DYNAMIXEL_UART->STAT & LPUART_STAT_TC_MASK)){//wait for transmission complete
		__asm volatile("NOP");
	}
}

//do something with an error
void DynamixelSendReceive::check_status_packet(){
	std::vector<uint8_t> statusPacket = this->get_status_packet();
	if(statusPacket.at(dynamixel::special::errorPositon) != dynamixel::error::noError){
		for(;;){
			__asm("NOP");
		}
	}
}

//status paket is ID|LENGTH|ERROR|PARAMETERS|CHECKSUM or 0xFF|0x01|ERROR
std::vector<uint8_t> DynamixelSendReceive::get_status_packet(){
	uint8_t headerOne = LPUART_ReadByte(DYNAMIXEL_UART);
	uint8_t headerTwo = LPUART_ReadByte(DYNAMIXEL_UART);
	std::vector<uint8_t> statusPacket;
	if(headerOne == dynamixel::special::header && headerTwo == dynamixel::special::header){
		statusPacket.push_back(LPUART_ReadByte(DYNAMIXEL_UART));
		uint8_t length = LPUART_ReadByte(DYNAMIXEL_UART);
		statusPacket.push_back(length);
		for(int i = 0; i < length; i++){
			statusPacket.push_back(LPUART_ReadByte(DYNAMIXEL_UART));
		}
	}else{
		statusPacket.push_back(0xFF);
		statusPacket.push_back(dynamixel::special::uartErrorLength);
		statusPacket.push_back(dynamixel::error::uart);
	}
	return statusPacket;
}
