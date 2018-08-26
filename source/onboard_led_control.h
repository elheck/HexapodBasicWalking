/*
 * OnboardLedControl.h
 *
 *  Created on: Oct 28, 2017
 *      Author: aron
 */

#ifndef ONBOARD_LED_CONTROL_H_
#define ONBOARD_LED_CONTROL_H_

#include "fsl_gpio.h"

#ifndef ONBOARD_LED_RED_GPIO
	#define ONBOARD_LED_RED_GPIO GPIOC
	#define ONBOARD_LED_RED_GPIO_PIN 8U
#endif
#ifndef ONBOARD_LED_GREEN_GPIO
	#define ONBOARD_LED_GREEN_GPIO GPIOC
	#define ONBOARD_LED_GREEN_GPIO_PIN 9U
#endif
#ifndef ONBOARD_LED_BLUE_GPIO
	#define ONBOARD_LED_BLUE_GPIO GPIOC
	#define ONBOARD_LED_BLUE_GPIO_PIN 10U
#endif

#if defined(__cplusplus)
extern "C" {
#endif

void toggle_red_led();

void red_led_on();

void red_led_off();

void toggle_green_led();

void green_led_on();

void green_led_off();

void toggle_blue_led();

void blue_led_on();

void blue_led_off();

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* ONBOARD_LED_CONTROL_H_ */
