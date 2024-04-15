#!/bin/sh

avr-gcc -Wall -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o controller.o controller.c
avr-gcc -mmcu=atmega328p controller.o -o controller
avr-objcopy -O ihex -R .eeprom controller controller.hex
avrdude -F -V -c arduino -p ATMEGA328P -P $1 -b 115200 -U flash:w:controller.hex
rm controller.o controller controller.hex
