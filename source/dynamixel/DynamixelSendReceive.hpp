/*
 * DynamixelControl.hpp
 *
 *  Created on: Nov 20, 2017
 *      Author: aron
 */

#ifndef DYNAMIXELSENDRECEIVE_HPP_
#define DYNAMIXELSENDRECEIVE_HPP_

#include <stdint.h>
#include <vector>


class DynamixelSendReceive {
public:
	DynamixelSendReceive(uint32_t baudRate);
	DynamixelSendReceive();
	virtual ~DynamixelSendReceive(){};

private:
	void set_up_uart(uint32_t baudRate);
	std::vector<uint8_t> construct_packet(std::vector<uint8_t> &instruction);
	void set_connection_to_transmit();
	void set_connection_to_receive();
	uint8_t calculate_checksum(std::vector<uint8_t> &instruction);
	void send_blocking(std::vector<uint8_t> &packet);
	std::vector<uint8_t> get_status_packet();

protected:
	void transmit_instruction(std::vector<uint8_t> instruction);
	void check_status_packet();
};



#endif /* DYNAMIXELSENDRECEIVE_HPP_ */
