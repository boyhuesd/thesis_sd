/*
 * pwm.c
 *
 *  Created on: 09-05-2014
 *      Author: boyhuesd
 */


#include "pwm.h"

void pwmSetup(void) {
  // Enable Peripheral Clocks
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

  // Enable pin PE4 for PWM0 M0PWM4
  GPIOPinConfigure(GPIO_PE4_M0PWM4);
  GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);

  // Config PWM module
  PWMGenConfigure(PWM_BASE, PWM_GEN_0, );
}

