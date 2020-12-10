#pragma once

/**
 * Pin mapping for the 2313/2313A/4313
 *
 *   Logical Pin: 03 02 17 09 10 11 12 13 14 15 16 00 01 04 05 06 07 08
 *   Port:        A0 A1 A2 B0 B1 B2 B3 B4 B5 B6 B7 D0 D1 D2 D3 D4 D5 D6
 */

#undef PA0
#define PA0           3
#define DIO3_PIN      PINA0
#define DIO3_RPORT    PINA
#define DIO3_WPORT    PORTA
#define DIO3_DDR      DDRA

#undef PA1
#define PA1           2
#define DIO2_PIN      PINA1
#define DIO2_RPORT    PINA
#define DIO2_WPORT    PORTA
#define DIO2_DDR      DDRA

#undef PA2
#define PA2           17
#define DIO17_PIN     PINA2
#define DIO17_RPORT   PINA
#define DIO17_WPORT   PORTA
#define DIO17_DDR     DDRA

#undef PB0
#define PB0           9
#define DIO9_PIN      PINB0
#define DIO9_RPORT    PINB
#define DIO9_WPORT    PORTB
#define DIO9_DDR      DDRB

#undef PB1
#define PB1           10
#define DIO10_PIN     PINB1
#define DIO10_RPORT   PINB
#define DIO10_WPORT   PORTB
#define DIO10_DDR     DDRB

#undef PB2
#define PB2           11
#define DIO11_PIN     PINB2
#define DIO11_RPORT   PINB
#define DIO11_WPORT   PORTB
#define DIO11_DDR     DDRB

#undef PB3
#define PB3           12
#define DIO12_PIN     PINB3
#define DIO12_RPORT   PINB
#define DIO12_WPORT   PORTB
#define DIO12_DDR     DDRB

#undef PB4
#define PB4           13
#define DIO13_PIN     PINB4
#define DIO13_RPORT   PINB
#define DIO13_WPORT   PORTB
#define DIO13_DDR     DDRB

#undef PB5
#define PB5           14
#define DIO14_PIN     PINB5
#define DIO14_RPORT   PINB
#define DIO14_WPORT   PORTB
#define DIO14_DDR     DDRB

#undef PB6
#define PB6           15
#define DIO15_PIN     PINB6
#define DIO15_RPORT   PINB
#define DIO15_WPORT   PORTB
#define DIO15_DDR     DDRB

#undef PB7
#define PB7           16
#define DIO16_PIN     PINB7
#define DIO16_RPORT   PINB
#define DIO16_WPORT   PORTB
#define DIO16_DDR     DDRB

#undef PD0
#define PD0           0
#define DIO0_PIN      PIND0
#define DIO0_RPORT    PIND
#define DIO0_WPORT    PORTD
#define DIO0_DDR      DDRD

#undef PD1
#define PD1           1
#define DIO1_PIN      PIND1
#define DIO1_RPORT    PIND
#define DIO1_WPORT    PORTD
#define DIO1_DDR      DDRD

#undef PD2
#define PD2           4
#define DIO4_PIN      PIND2
#define DIO4_RPORT    PIND
#define DIO4_WPORT    PORTD
#define DIO4_DDR      DDRD

#undef PD3
#define PD3           5
#define DIO5_PIN      PIND3
#define DIO5_RPORT    PIND
#define DIO5_WPORT    PORTD
#define DIO5_DDR      DDRD

#undef PD4
#define PD4           6
#define DIO6_PIN      PIND4
#define DIO6_RPORT    PIND
#define DIO6_WPORT    PORTD
#define DIO6_DDR      DDRD

#undef PD5
#define PD5           7
#define DIO7_PIN      PIND5
#define DIO7_RPORT    PIND
#define DIO7_WPORT    PORTD
#define DIO7_DDR      DDRD

#undef PD6
#define PD6           8
#define DIO8_PIN      PIND6
#define DIO8_RPORT    PIND
#define DIO8_WPORT    PORTD
#define DIO8_DDR      DDRD
