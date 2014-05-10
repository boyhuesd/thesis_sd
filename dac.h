/*
 * dac.h
 *
 *  Created on: 10-05-2014
 *      Author: boyhuesd
 */

#ifndef DAC_H_
#define DAC_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_adc.h"
#include "inc/hw_ints.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/udma.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "cirbuf.h"
#include "global.h"

extern volatile uint16_t dacIndex;
extern volatile elementT * dacBuf;
extern volatile bufT * gpBuf;
extern volatile bool stop;

void dacSetup(void);
void dacEnable(void);
void dacDisable(void);
void dacIntHandler(void);

#endif /* DAC_H_ */
