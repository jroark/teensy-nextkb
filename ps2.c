/*
  Modified by John Roark <john.roark@gmail.com>

  removed Arduino library calls

  PS2Keyboard.cpp - PS2Keyboard library
  Copyright (c) 2007 Free Software Foundation.  All right reserved.
  Written by Christian Weichel <info@32leaves.net>

  ** Mostly rewritten Paul Stoffregen <paul@pjrc.com> 2010, 2011
  ** Modified for use beginning with Arduino 13 by L. Abraham Smith, <n3bah@microcompdesign.com> * 
  ** Modified for easy interrup pin assignement on method begin(datapin,irq_pin). Cuningan <cuninganreset@gmail.com> **

  for more information you can read the original wiki in arduino.cc
  at http://www.arduino.cc/playground/Main/PS2Keyboard
  or http://www.pjrc.com/teensy/td_libs_PS2Keyboard.html

  Version 2.3 (October 2011)
  - Minor bugs fixed

  Version 2.2 (August 2011)
  - Support non-US keyboards - thanks to Rainer Bruch for a German keyboard :)

  Version 2.1 (May 2011)
  - timeout to recover from misaligned input
  - compatibility with Arduino "new-extension" branch
  - TODO: send function, proposed by Scott Penrose, scooterda at me dot com

  Version 2.0 (June 2010)
  - Buffering added, many scan codes can be captured without data loss
    if your sketch is busy doing other work
  - Shift keys supported, completely rewritten scan code to ascii
  - Slow linear search replaced with fast indexed table lookups
  - Support for Teensy, Arduino Mega, and Sanguino added

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "ps2.h"
#include <string.h>
#include <util/delay.h>

#define LED_ON    (PORTD |= (1<<6))
#define LED_OFF   (PORTD &= ~(1<<6))

#define LED_CONFIG      (DDRD |= (1<<6))
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

/* pins for PS2 KB CLK/DATA */
#define PS2KBCLK   0 /*PD0 == 5*/
#define PS2KBDATA  3 /*PD3 == 8*/

#define PS2KBCLK_HI          (PORTD |= (1<<PS2KBCLK))
#define PS2KBCLK_LO          (PORTD &= ~(1<<PS2KBCLK))
#define PS2KBDATA_HI         (PORTD |= (1<<PS2KBDATA))
#define PS2KBDATA_LO         (PORTD &= ~(1<<PS2KBDATA))

#define PS2KBDATA_ISHI       ((PIND & (1<<PS2KBDATA)) ? 1 : 0)

#define PS2KBCLK_CONFIG_OUT  (DDRD |= (1<<PS2KBCLK))
#define PS2KBCLK_CONFIG_IN   (DDRD &= ~(1<<PS2KBCLK))

#define PS2KBDATA_CONFIG_OUT (DDRD |= (1<<PS2KBDATA))
#define PS2KBDATA_CONFIG_IN  (DDRD &= ~(1<<PS2KBDATA))

/* pins for PS2 mouse CLK/DATA */
/* TODO */

/* pins for NeXT IN/OUT from soundbox */
#define FROM_NEXT  0 /*PB0 = 0 */
#define TO_NEXT    1 /*PB1 = 1 */

#define FROM_NEXT_HI         (PORTB |= (1<<FROM_NEXT))
#define FROM_NEXT_LO         (PORTB &= ~(1<<FROM_NEXT))
#define TO_NEXT_HI           (PORTB |= (1<<TO_NEXT))
#define TO_NEXT_LO           (PORTB &= ~(1<<TO_NEXT))

#define TO_NEXT_ISHI         ((PINB & (1<<TO_NEXT)) ? 1 : 0)
#define FROM_NEXT_ISHI       ((PINB & (1<<FROM_NEXT)) ? 1 : 0)

#define FROM_NEXT_CONFIG_OUT (DDRB |= (1<<FROM_NEXT))
#define FROM_NEXT_CONFIG_IN  (DDRB &= ~(1<<FROM_NEXT))

#define TO_NEXT_CONFIG_OUT   (DDRB |= (1<<TO_NEXT))
#define TO_NEXT_CONFIG_IN    (DDRB &= ~(1<<TO_NEXT))

#define TIMING 0.05

#define BUFFER_SIZE 45
static volatile uint8_t buffer[BUFFER_SIZE];
static volatile uint8_t head, tail;

volatile unsigned long timer0_millis_count = 0; 

ISR(TIMER0_OVF_vect)
{
    timer0_millis_count++;
}

static inline uint32_t millis(void) __attribute__((always_inline, unused));
static inline uint32_t millis(void)
{
    uint32_t out; 
    asm volatile(
        "in __tmp_reg__, __SREG__"      "\n\t"
        "cli"                   "\n\t"
        "lds    %A0, timer0_millis_count"   "\n\t"
        "lds    %B0, timer0_millis_count+1" "\n\t"
        "lds    %C0, timer0_millis_count+2" "\n\t"
        "lds    %D0, timer0_millis_count+3" "\n\t"
        "out    __SREG__, __tmp_reg__"
        : "=r" (out) : : "r0" 
    );   
    return out; 
}

// The ISR for the external PS2 KB interrupt
ISR(INT0_vect)
{
    static uint8_t bitcount=0;
    static uint8_t incoming=0;
    static uint32_t prev_ms=0;
    uint32_t now_ms;
    uint8_t n, val;

    val = PS2KBDATA_ISHI;
    now_ms = millis();

    if (now_ms - prev_ms > 250) {
        bitcount = 0;
        incoming = 0;
    }
    prev_ms = now_ms;
    n = bitcount - 1;
    if (n <= 7) {
        incoming |= (val << n);
    }
    bitcount++;
    if (bitcount == 11) {
        uint8_t i = head + 1;
        if (i >= BUFFER_SIZE) i = 0;
        if (i != tail) {
            buffer[i] = incoming;
            head = i;
        }
        bitcount = 0;
        incoming = 0;
    }
}

static inline uint8_t get_scan_code (void)
{
    uint8_t c, i;

    i = tail;
    if (i == head)
        return 0;
    i++;
    if (i >= BUFFER_SIZE) 
        i = 0;
    c = buffer[i];
    tail = i;

    return c;
}

static inline void send_response (uint8_t resp_lo, uint8_t resp_hi, uint8_t resp_hihi)
{
    uint8_t i = 0;


    //cli ();
    for (i = 0; i < 8; i++)
    {
        if (resp_lo & ((uint32_t)1 << i))
            TO_NEXT_HI;
        else
            TO_NEXT_LO;
        _delay_ms (TIMING);
    }
    for (i = 0; i < 8; i++)
    {
        if (resp_hi & ((uint32_t)1 << i))
            TO_NEXT_HI;
        else
            TO_NEXT_LO;
        _delay_ms (TIMING);
    }
    for (i = 0; i < 6; i++)
    {
        if (resp_hihi & ((uint32_t)1 << i))
            TO_NEXT_HI;
        else
            TO_NEXT_LO;
        _delay_ms (TIMING);
    }
    TO_NEXT_HI;
    //sei ();
}

int main (void)
{
    CPU_PRESCALE (0);
    LED_CONFIG;
    LED_OFF;

    // timer 0, fast pwm mode
    TCCR0A = (1<<WGM01) | (1<<WGM00);
    TCCR0B = (1<<CS01) | (1<<CS00);        // div 64 prescaler
    TIMSK0 |= (1<<TOIE0);
    TCNT0 = 0;

    FROM_NEXT_CONFIG_IN;
    FROM_NEXT_HI;
    TO_NEXT_CONFIG_OUT;
    TO_NEXT_HI;

    PS2KBCLK_CONFIG_IN;
    PS2KBCLK_HI;
    PS2KBDATA_CONFIG_IN;
    PS2KBDATA_HI;
  
    head = 0;
    tail = 0;

    // falling edge for PS2 int
    EICRA = (EICRA & 0xFC) | (0x02&0x03);
    EIMSK |= (1 << PS2KBCLK);
    sei ();

    while (1)
    {
        uint32_t val, preval=0;
        uint8_t i;

        LED_OFF;
        while (FROM_NEXT_ISHI);

        val = 0;
        //cli (); 
        _delay_ms (TIMING/2);
        for (i = 0; i < 6; i++) {
            if (FROM_NEXT_ISHI)
                val |= ((uint32_t)1 << i); 
            _delay_ms (TIMING);
        }   

        switch (val) {
            case 0x00000020: /* KB QUERY */
            case 0x00000022: /* MOUSE QUERY */
                for (i = 6; i < 9; i++) {
                    if (FROM_NEXT_ISHI)
                        val |= ((uint32_t)1 << i); 
                    _delay_ms (TIMING);
                }   
                break;
            case 0x0000001E: /* KB RESET */
            case 0x00000000: /* LED CMD */
            default:
                for (i = 6; i < 22; i++) {
                    if (FROM_NEXT_ISHI)
                        val |= ((uint32_t)1 << i);
                    _delay_ms (TIMING);
                }
                break;
        }
        //sei ();

        if (val == 0x00000020) {
            uint32_t resp = 0x00300600;
            uint8_t ps2_scancode = get_scan_code ();

            /* send kb response */
            if (ps2_scancode && (ps2_scancode < 0x83)) {
                resp = 2*ps2_next_scancodes[ps2_scancode];
                resp |= 0x00280400;
                LED_ON;
            } else if (ps2_scancode == 0xE0) {
                /* multi-byte scancode */
                ps2_scancode = get_scan_code ();

                if (ps2_scancode == 0xF0) {
                    ps2_scancode = get_scan_code ();
                    resp = 0x00280500;
                } else
                    resp = 0x00280400;
                resp |= 2*ps2_next_scancodes[ps2_scancode];
                LED_ON;
            } else if (ps2_scancode == 0xF0) {
                /* key up scancdoe */
                ps2_scancode = get_scan_code ();

                resp = 2*ps2_next_scancodes[ps2_scancode];
                resp |= 0x00280500;
                LED_ON;
            }
            _delay_ms (TIMING*4.5);
            send_response (resp & 0x000000FF, (resp & 0x0000FF00)>>8, (resp & 0x00FF0000)>>16);
        } else if (val == 0x00000022) {
            /* TODO: if PS2 mouse available send data */
            uint32_t resp = 0x00300600;
            _delay_ms (TIMING*4.5);
            send_response (resp & 0x000000FF, (resp & 0x0000FF00)>>8, (resp & 0x00FF0000)>>16);
        }
        _delay_ms (TIMING*4);
    }

    return 0;
}


