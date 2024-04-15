#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#include <avr/io.h>
#include <avr/interrupt.h>

//#include "micros.h"

/*
PWM input using timer and interrupt.
 - uses PCINT1 interrupt vector (any adc/pinc pin)
 - callback PWM_ON_PERIOD when period is detected

Usage:
 - Call startPWM() to start and enable interrupt
 - #define PWM_ON_PERIOD(period, duty) to run code each period
    - period -> uint32_t (microseconds)
    - duty   -> uint8_t (0 is ~0%, 255 is ~100%)
 - !! Disabling interrupts prevent PWM from working properly !!
*/

uint32_t _pwm_last = 0;
uint32_t _pwm_edge = 0;
uint8_t _pwm_pin = 7;

ISR(PCINT1_vect) {
    uint8_t val = PINC & (1 << _pwm_pin);
    uint32_t now = micros();

    if (val) {
        uint32_t period = now - _pwm_last;
        uint8_t duty = (_pwm_edge - _pwm_last)*256/period;
        _pwm_last = now;
        #ifdef PWM_ON_PERIOD
        PWM_ON_PERIOD(period, duty)
        #endif
    } else {
        _pwm_edge = now;
    }
}

void startPWM(uint8_t adcPin) {
//    startMicros();

    // enable adc interrupt
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << adcPin);
    _pwm_pin = adcPin;
}

#endif
