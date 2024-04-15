#ifndef RX_ANALOG_H
#define RX_ANALOG_H

#include <avr/io.h>

void _rx_onPeriod(uint32_t period, uint8_t duty);
#define PWM_ON_PERIOD(p, d) _rx_onPeriod(p, d);
#include "pwm_input.h"

/*
PWM based receiver. Receives two values continuously

Usage:
 - Call rxInit(uint8_t adcPin) to begin receiving on adcPin
 - Call rxRecv(uint8_t* a, uint8_t* b) to get data
    - Returns current state and sets received values
 - !! Disabling interrupts prevents reception from working properly !!
*/

#ifndef RX_LOW_RATE
#define RX_LOW_RATE 80
#endif

#ifndef RX_HIGH_RATE
#define RX_HIGH_RATE 250
#endif

#ifndef RX_RATE_MARGIN
#define RX_RATE_MARGIN 20
#endif

#ifndef RX_LOW_DUTY
#define RX_LOW_DUTY 100
#endif

#ifndef RX_HIGH_DUTY
#define RX_HIGH_DUTY 200
#endif

#ifndef RX_DUTY_MARGIN
#define RX_DUTY_MARGIN 25
#endif

#ifndef RX_SYNC_TIMEOUT
#define RX_SYNC_TIMEOUT 100000
#endif

#ifndef RX_SYNC_THRESHOLD
#define RX_SYNC_THRESHOLD 8
#endif

#define RX_MID_RATE ((RX_HIGH_RATE + RX_LOW_RATE)/2)
#define RX_RATE_RANGE (RX_HIGH_RATE - RX_LOW_RATE)
#define RX_MID_DUTY ((RX_HIGH_DUTY - RX_LOW_DUTY)/2)
#define RX_DUTY_RANGE (RX_HIGH_DUTY - RX_LOW_DUTY)

#define RX_NOISE 0
#define RX_SYNC 1

volatile uint8_t _rx_state = RX_NOISE;
uint8_t _rx_counter = 0;
uint16_t _rx_rate = RX_MID_RATE;
uint8_t _rx_duty = RX_MID_DUTY;

inline
void rxInit(uint8_t adcPin) {
    // Start pwm input on adc pin 0
    startPWM(adcPin);
}

uint8_t rxRecv(uint8_t* a, uint8_t* b) {
    // Return to noise if no pulse received in timeout period
    if (micros() - _pwm_last > RX_SYNC_TIMEOUT) {
        _rx_counter = 0;
        _rx_state = RX_NOISE;
    }

    // Set period and duty
    uint16_t rateRanged = (_rx_rate < RX_HIGH_RATE ? (_rx_rate < RX_LOW_RATE ? RX_LOW_RATE : _rx_rate) : RX_HIGH_RATE);
    *a = (rateRanged - RX_LOW_RATE)*255/RX_RATE_RANGE;

    // Set duty
    uint8_t dutyRanged = (_rx_duty < RX_HIGH_DUTY ? (_rx_duty < RX_LOW_DUTY ? RX_LOW_DUTY : _rx_duty) : RX_HIGH_DUTY);
    *b = (dutyRanged - RX_LOW_DUTY)*255/RX_DUTY_RANGE;

    // Return state
    return _rx_state;
} 

void _rx_onPeriod(uint32_t period, uint8_t duty) {
    // Determine if rate is in range
    uint16_t rate = 1000000/period;
    if (rate < RX_LOW_RATE - RX_RATE_MARGIN || rate > RX_HIGH_RATE + RX_RATE_MARGIN) _rx_counter /= 2;
    else {
        _rx_rate = (rate + _rx_rate)/2;
        if (_rx_counter != 255) _rx_counter++;
    }

    // Determine if duty is in range
    if (duty < RX_LOW_DUTY - RX_DUTY_MARGIN || duty > RX_HIGH_DUTY + RX_DUTY_MARGIN) _rx_counter /= 2;
    else {
        _rx_duty = (duty + _rx_duty)/2;
        if (_rx_counter != 255) _rx_counter++;
    }
    _rx_duty = duty;

    // Set state
    _rx_state = (_rx_counter >= RX_SYNC_THRESHOLD ? RX_SYNC : RX_NOISE);
}

#endif
