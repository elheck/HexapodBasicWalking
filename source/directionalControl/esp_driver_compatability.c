/*
 * esp_driver_compatability.c
 *
 *  Created on: Jan 14, 2018
 *      Author: aron
 */


#include "esp_driver_compatability.h"
//Documentation page ~1916
#define OSCERCLK_SOURCE 2U
#define ESP_UART LPUART0

volatile uint8_t ringBuffer[BUFFER_SIZE] = {0x00};
volatile uint16_t rxIndex = 0;
volatile uint16_t txIndex = 0;

void init_driver_compatability(uint32_t baudRate){
	lpuart_config_t config;
	CLOCK_SetLpuartClock(OSCERCLK_SOURCE);
	LPUART_GetDefaultConfig(&config);
	config.baudRate_Bps = baudRate;
	config.enableRx = true;
	config.enableTx = true;
	uint32_t clockFrequency = CLOCK_GetFreq(kCLOCK_Osc0ErClk);
	LPUART_Init(ESP_UART, &config, clockFrequency);
	ESP_UART->CTRL &= ~LPUART_CTRL_RE_MASK; //disable TE and RE for setting FIFO
	ESP_UART->CTRL &= ~LPUART_CTRL_TE_MASK;
	ESP_UART->FIFO &= ~LPUART_FIFO_RXFE_MASK;
	ESP_UART->FIFO &= ~LPUART_FIFO_TXFE_MASK;
	ESP_UART->CTRL |= LPUART_CTRL_RE_MASK;
	ESP_UART->CTRL |= LPUART_CTRL_TE_MASK;
	LPUART_EnableInterrupts(ESP_UART, kLPUART_RxDataRegFullInterruptEnable | kLPUART_RxOverrunInterruptEnable);
	EnableIRQ(UART_IRQ);

}

void UART_RECEIVE_INTERRUPT(void){
	uint8_t data;
	uint32_t flags = LPUART_GetStatusFlags(ESP_UART);
	if (kLPUART_RxDataRegFullFlag & flags){
		data = LPUART_ReadByte(ESP_UART);
		if((rxIndex + 1) % BUFFER_SIZE != txIndex){
			ringBuffer[rxIndex] = data;
			rxIndex++;
			rxIndex %= BUFFER_SIZE;
		}
	}
	if(kLPUART_RxOverrunFlag & flags){
		ESP_UART->STAT |= LPUART_STAT_OR_MASK;
	}
}

void get_arrived_data(uint8_t *data, uint32_t *dataLength){
	uint32_t diff = calculate_esp_read_write_distance();
	if(diff > MESSAGE_LENGTH){
		*dataLength = diff / MESSAGE_LENGTH;
		*dataLength = *dataLength * MESSAGE_LENGTH;
		for(int i = 0; i < *dataLength; i++){
			data[i] = read_next_byte_from_buffer();
			if((i % (MESSAGE_LENGTH-1)) != 0 && (data[i] == DELIMITER)){
				*dataLength = BUFFER_SIZE + BUFFER_SIZE ;
				return;
			}
		}
	}else{
		*dataLength = BUFFER_SIZE + BUFFER_SIZE ;
	}
}

uint32_t calculate_esp_read_write_distance(void){
	uint32_t diff = 0;
	if (txIndex != rxIndex){
		if(rxIndex - txIndex > 0){
			diff = (rxIndex - txIndex);
		}else {
			diff = (BUFFER_SIZE - txIndex) + rxIndex;
		}
	}
	return diff;
}

uint8_t read_next_byte_from_buffer(){
	uint8_t data =  ringBuffer[txIndex];
	txIndex++;
	if (txIndex+1 % BUFFER_SIZE == 0){
		txIndex = 0;
	}
	return data;
}

