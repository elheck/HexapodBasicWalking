/*
 * pin_init.c
 *
 *  Created on: Oct 29, 2017
 *      Author: aron
 */
#include "pin_init.h"
#include "fsl_gpio.h"

void init_pins(){
	BOARD_InitBootPins();
	set_led_pin_direction();
	set_dynamixel_dir_configuration();
}

void set_led_pin_direction(void){
	gpio_pin_config_t onboardLedControl = {
		  kGPIO_DigitalOutput,
		  1,
	  };
	GPIO_PinInit(ONBOARD_LED_BLUE_GPIO, ONBOARD_LED_BLUE_GPIO_PIN, &onboardLedControl);
	GPIO_PinInit(ONBOARD_LED_RED_GPIO, ONBOARD_LED_RED_GPIO_PIN, &onboardLedControl);
	GPIO_PinInit(ONBOARD_LED_GREEN_GPIO, ONBOARD_LED_GREEN_GPIO_PIN, &onboardLedControl);
}

void set_dynamixel_dir_configuration(void){
	gpio_pin_config_t dynamixelDirection = {
			  kGPIO_DigitalOutput,
			  1,
	};
	GPIO_PinInit(DYNAMIXEL_DIR_GPIO, DYNAMIXEL_DIR_GPIO_PIN, &dynamixelDirection);
}
