/*
 * dac.c
 *
 *  Created on: 10-05-2014
 *      Author: boyhuesd
 */
#include "dac.h"

void dacSetup(void) {
  // Enable Peripheral Clocks
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

  // Enable pin PE4 for PWM0 M0PWM4
  GPIOPinConfigure(GPIO_PE4_M0PWM4);
  GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);

  // Config PWM module
  PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 1023);

  // Use timer 0 as a full-width timer.
  TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

  // Set timer to trigger ADC at rate 8000Hz
  TimerLoadSet(TIMER1_BASE, TIMER_A, SYS_CLK/8000); // DAC output rate

  // Interrupt for timer1
  IntEnable(INT_TIMER1A);
  TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}

void dacEnable(void) {
  // Enable timer
  TimerEnable(TIMER1_BASE, TIMER_A);

  // Enable PWM
  PWMGenEnable(PWM0_BASE, PWM_GEN_2);
  PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
}

void dacDisable(void) {
  // Disable timer
  TimerDisable(TIMER1_BASE, TIMER_A);

  // Disable PWM
  PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
  PWMGenDisable(PWM0_BASE, PWM_GEN_2);
}

void dacIntHandler(void) {
  // Clear timer
  TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

  // Output the DAC value
  if (dacBuf) {
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, (dacBuf->data[dacIndex]>>2));
  }
  else {
    if (stop) {
      dacDisable(); // Stop DAC output
    }

    dacIndex = 0;
    dacBuf = bufGet(gpBuf); // Try to get the data again
  }

  dacIndex++;
  if (dacIndex == 512) {
    // Remember to set buffer item FREE after using it
    bufItemSetFree(gpBuf, dacBuf->index);

    dacIndex = 0; // Reset output index
    dacBuf = bufGet(gpBuf); // Get new data
  }
}


