
#include <avr/io.h>
#include <avr/interrupt.h>

#include "../robot/txrx_settings.h"
#include "tx_analog.h"

uint8_t adc = 0;
uint8_t x = 0;
uint8_t y = 0;

ISR(ADC_vect) {
    // Read adc value and switch sources
    if (adc) {
        x = ADCH;
        adc = 0;
        ADMUX &= ~(1 << MUX0);
    } else {
        y = ADCH;
        adc = 1;
        ADMUX |= (1 << MUX0);
    }

    // Set transmission and start next conversion
    txSend(y, x);
    ADCSRA |= (1 << ADSC);
}

// Transmit on pin 5
int main() {
    // Set adc reference voltage to avcc and left align output
    ADMUX |= (1 << REFS0) | (1 << ADLAR);

    // Disable digital input for analog pins
    DIDR0 |= (1 << ADC0D) | (1 << ADC1D);

    // Enable adc, interrupt, and start conversion
    sei();
    ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADSC);

    // Turn on led
    DDRB |= (1 << DDB5);
    PORTB |= (1 << PORTB5);

    // Start transmitting
    txStart();

    while (1) {}
}
