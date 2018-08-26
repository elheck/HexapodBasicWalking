/*
 * onboard_led_control.c
 *
 *  Created on: Mar 15, 2018
 *      Author: aron
 */
#include "onboard_led_control.h"

void toggle_red_led(){
	GPIO_TogglePinsOutput(ONBOARD_LED_RED_GPIO, 1 << ONBOARD_LED_RED_GPIO_PIN);
}

void red_led_on(){
	GPIO_SetPinsOutput(ONBOARD_LED_RED_GPIO, 1 << ONBOARD_LED_RED_GPIO_PIN);
}

void red_led_off(){
	GPIO_ClearPinsOutput(ONBOARD_LED_RED_GPIO, 1 << ONBOARD_LED_RED_GPIO_PIN);
}

void toggle_green_led(){
	GPIO_TogglePinsOutput(ONBOARD_LED_GREEN_GPIO, 1 << ONBOARD_LED_GREEN_GPIO_PIN);
}

void green_led_on(){
	GPIO_SetPinsOutput(ONBOARD_LED_GREEN_GPIO, 1 << ONBOARD_LED_GREEN_GPIO_PIN);
}

void green_led_off(){
	GPIO_ClearPinsOutput(ONBOARD_LED_GREEN_GPIO, 1 << ONBOARD_LED_GREEN_GPIO_PIN);
}

void toggle_blue_led(){
	GPIO_TogglePinsOutput(ONBOARD_LED_BLUE_GPIO, 1 << ONBOARD_LED_BLUE_GPIO_PIN);
}

void blue_led_on(){
	GPIO_SetPinsOutput(ONBOARD_LED_BLUE_GPIO, 1 << ONBOARD_LED_BLUE_GPIO_PIN);
}

void blue_led_off(){
	GPIO_ClearPinsOutput(ONBOARD_LED_BLUE_GPIO, 1 << ONBOARD_LED_BLUE_GPIO_PIN);
}
