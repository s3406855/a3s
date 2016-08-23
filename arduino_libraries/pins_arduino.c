/*
  pins_arduino.c - pin definitions for the Arduino board
  Part of Arduino / Wiring Lite

  Copyright (c) 2005 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id$
*/

#include <avr/io.h>
#include "wiring_private.h"
#include "pins_arduino.h"

#define PA 1
#define PB 2
#define PC 3
#define PD 4

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
        NOT_A_PORT,
        (uint16_t) &DDRA,
        (uint16_t) &DDRB,
        (uint16_t) &DDRC,
        (uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
        NOT_A_PORT,
        (uint16_t) &PORTA,
        (uint16_t) &PORTB,
        (uint16_t) &PORTC,
        (uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
        NOT_A_PORT,
        (uint16_t) &PINA,
        (uint16_t) &PINB,
        (uint16_t) &PINC,
        (uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
        PB, /* 1 */
        PB,
        PB,
        PB,
        PB,
        PB,
        PB,
        PB,
        NOT_A_PIN, /* RESET */
        NOT_A_PIN, /* VCC */
        NOT_A_PIN, /* GND */
        NOT_A_PIN, /* XTAL2 */
        NOT_A_PIN, /* XTAL1 */
        PD, /* 14 */
        PD,
        PD,
        PD,
        PD,
        PD,
        PD,
        PD,
        PC, /* 22 */
        PC,
        PC,
        PC,
        PC,
        PC,
        PC,
        PC,
        NOT_A_PIN, /* AVCC */
        NOT_A_PIN, /* GND */
        NOT_A_PIN, /* AREF */
        PA, /* 33 */
        PA,
        PA,
        PA,
        PA,
        PA,
        PA,
        PA,
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
        _BV(0), /* 1, port B */
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(4),
        _BV(5),
        _BV(6),
        _BV(7),
        NOT_A_PIN,
        NOT_A_PIN,
        NOT_A_PIN,
        NOT_A_PIN,
        NOT_A_PIN,
        _BV(0), /* 14, port D */
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(4),
        _BV(5),
        _BV(6),
        _BV(7),
        _BV(0), /* 22, port C */
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(4),
        _BV(5),
        _BV(6),
        _BV(7),
        NOT_A_PIN,
        NOT_A_PIN,
        NOT_A_PIN,
        _BV(7), /* 33, port A */
        _BV(6),
        _BV(5),
        _BV(4),
        _BV(3),
        _BV(2),
        _BV(1),
        _BV(0)
};

// No idea about this part... hopefully nothing breaks!
const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
        NOT_ON_TIMER, /* 1 - port B */
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER, // 4, OC0 PB3
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_A_PIN,
        NOT_A_PIN,
        NOT_A_PIN,
        NOT_A_PIN,
        NOT_A_PIN,
        NOT_ON_TIMER, /* 14 - port D */
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER, // 18, OCR1B PD4
        NOT_ON_TIMER, // 19, OCR1A PD5
        NOT_ON_TIMER,
        NOT_ON_TIMER, // 21, OC2 PD7
        NOT_ON_TIMER, // 22, port C
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_A_PIN,
        NOT_A_PIN,
        NOT_A_PIN,
        NOT_ON_TIMER, //33, port A 
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
}; 
