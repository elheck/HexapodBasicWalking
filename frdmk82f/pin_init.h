/*
 * pin_init.h
 *
 *  Created on: Oct 29, 2017
 *      Author: aron
 */

#ifndef PIN_INIT_H_
#define PIN_INIT_H_

#include "pin_mux.h"

#if defined(__cplusplus)
extern "C" {
#endif

void init_pins();

void set_led_pin_direction(void);

void set_dynamixel_dir_configuration(void);


#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* PIN_INIT_H_ */
