#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H

#include <avr/io.h>
#include <avr/interrupt.h>

/*
PWM with variable frequency/duty using counter 0.
Counter 0 configuration:
 - fast PWM mode (TOP = OCR0A)
 - OCR0A and prescaler based on frequency, OCR0B based on duty
 - overflow interrupt enabled
 - non-inverting output on d5

Usage:
 - Call setPWM(rate, duty) to configure PWM generator
 - Call startPWM() to output the PWM wave on d5
 - Call stopPWM() to stop outputting on d5
 - #define PWM_ON_PERIOD to run code at the start of each period
 - !! Disabling interrupts prevent PWM from working properly !!
*/

uint8_t _pwm_prescaling = 0;

ISR(TIMER0_OVF_vect) {
    // update prescaling
    TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
    TCCR0B |= _pwm_prescaling;

    // call pwm update handler
    #ifdef PWM_ON_PERIOD
    PWM_ON_PERIOD
    #endif
}

void setPWM(uint16_t rate, uint8_t duty) {
    // Recalculate period and prescaling
    uint32_t freq = 0;
    if (rate > 62378UL) {
        // no prescaling
        freq = 16000000UL;
        _pwm_prescaling = (1 << CS00);
    } else if (rate > 7797UL) {
        // 8x prescaling
        freq = 2000000UL;
        _pwm_prescaling = (1 << CS01);
    } else if (rate > 974UL) {
        // 64x prescaling
        freq = 250000UL;
        _pwm_prescaling = (1 << CS01) | (1 << CS00);
    } else if (rate > 243UL) {
        // 256x prescaling
        freq = 62500UL;
        _pwm_prescaling = (1 << CS02);
    } else if (rate > 60UL) {
        // 1024x prescaling
        freq = 15625UL;
        _pwm_prescaling = (1 << CS02) | (1 << CS00);
    }

    // Calculate full period and pulse period
    OCR0A = (freq - (rate/2))/rate;
    OCR0B = OCR0A*duty/255;
}

void startPWM() {
    // set fast PWM Mode (TOP = OCR0A)
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	TCCR0B |= (1 << WGM02);

    // enable overflow interrupt
    sei();
    TIMSK0 |= (1 << TOIE0);

    // output non-inverting on OC0B (D5)
    TCCR0A |= (1 << COM0B1);
    DDRD |= (1 << DDD5);

    // start clock
    TCCR0B |= _pwm_prescaling;
}

void stopPWM() {
    // stop outputting on OC0B (D5)
    DDRD &= ~(1 << DDD5);
    TCCR0A &= ~(1 << COM0B1);

    // stop and reset timer
    TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
    TCNT0 = 0;
}

#endif
