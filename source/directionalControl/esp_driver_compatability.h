/*
 * esp_driver_compatability.h
 *
 *  Created on: Jan 14, 2018
 *      Author: aron
 */

#ifndef DIRECTIONALCONTROL_ESP_DRIVER_COMPATABILITY_H_
#define DIRECTIONALCONTROL_ESP_DRIVER_COMPATABILITY_H_

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdint.h>
#include "fsl_lpuart.h"

#define UART_IRQ LPUART0_IRQn
#define UART_RECEIVE_INTERRUPT LPUART0_IRQHandler
#define BUFFER_SIZE 2048
#define MESSAGE_LENGTH 11
#define DELIMITER '#'

extern volatile uint8_t ringBuffer[BUFFER_SIZE]; /*Ringbuffer*/
extern volatile uint16_t rxIndex ; /* Index of the memory to save new arrived data. */
extern volatile uint16_t txIndex;

void init_driver_compatability(uint32_t baudRate);

void get_arrived_data(uint8_t *data, uint32_t *dataLength);

uint32_t calculate_esp_read_write_distance(void);

uint8_t read_next_byte_from_buffer();

#if defined(__cplusplus)
}
#endif




#endif /* DIRECTIONALCONTROL_ESP_DRIVER_COMPATABILITY_H_ */
