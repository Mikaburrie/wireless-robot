#ifndef TX_ANALOG_H
#define TX_ANALOG_H

void _tx_onPeriod();
#define PWM_ON_PERIOD _tx_onPeriod();
#include "pwm_output.h"

/*
PWM based analog transmitter. Sends two values continuously

Usage:
 - Call txSend(uint8_t a, uint8_t b) to set transmitted values
 - Call txStart() to begin transmitting on pin d5
 - Call txStop() to stop transmitting
 - !! Disabling interrupts prevents transmission from working properly !!
*/

#ifndef TX_LOW_RATE
#define TX_LOW_RATE 80
#endif

#ifndef TX_HIGH_RATE
#define TX_HIGH_RATE 250
#endif

#ifndef TX_LOW_DUTY
#define TX_LOW_DUTY 100
#endif

#ifndef TX_HIGH_DUTY
#define TX_HIGH_DUTY 200
#endif

#define TX_MID_RATE ((TX_HIGH_RATE + TX_LOW_RATE)/2)
#define TX_RATE_RANGE (TX_HIGH_RATE - TX_LOW_RATE)
#define TX_MID_DUTY ((TX_HIGH_DUTY + TX_LOW_DUTY)/2)
#define TX_DUTY_RANGE (TX_HIGH_DUTY - TX_LOW_DUTY)

uint16_t _tx_rate = TX_MID_RATE;
uint8_t _tx_duty = TX_MID_DUTY;

void txSend(uint8_t a, uint8_t b) {
    _tx_rate = TX_RATE_RANGE*(uint16_t)a/255 + TX_LOW_RATE;
    _tx_duty = TX_DUTY_RANGE*b/255 + TX_LOW_DUTY;
}

void txStart() {
    setPWM(_tx_rate, _tx_duty);
    startPWM();
}

void txStop() {
    stopPWM();
}

void _tx_onPeriod() {
    setPWM(_tx_rate, _tx_duty);
}

#endif