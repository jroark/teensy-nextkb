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

/* this will overwhelm the Teensy */
#define USE_MOUSE 0

#if USE_MOUSE
/* pins for PS2 mouse CLK/DATA */
#define PS2MSCLK   1 /*PD1 == 6*/
#define PS2MSDATA  6 /*PC6 == 9*/

#define PS2MSCLK_HI          (PORTD |= (1<<PS2MSCLK))
#define PS2MSCLK_LO          (PORTD &= ~(1<<PS2MSCLK))
#define PS2MSDATA_HI         (PORTC |= (1<<PS2MSDATA))
#define PS2MSDATA_LO         (PORTC &= ~(1<<PS2MSDATA))

#define PS2MSDATA_ISHI       ((PINC & (1<<PS2MSDATA)) ? 1 : 0)

#define PS2MSCLK_CONFIG_OUT  (DDRD |= (1<<PS2MSCLK))
#define PS2MSCLK_CONFIG_IN   (DDRD &= ~(1<<PS2MSCLK))

#define PS2MSCLK_ISHI        ((PIND & (1<<PS2MSCLK)) ? 1 : 0)

#define PS2MSDATA_CONFIG_OUT (DDRC |= (1<<PS2MSDATA))
#define PS2MSDATA_CONFIG_IN  (DDRC &= ~(1<<PS2MSDATA))
#endif
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

#define TIMING 50

#define BUFFER_SIZE (5*45)
static volatile uint8_t kb_buffer[BUFFER_SIZE];
static volatile uint8_t kb_head, kb_tail;

#if USE_MOUSE
static volatile uint8_t ms_buffer[BUFFER_SIZE];
static volatile uint8_t ms_head, ms_tail;
#endif

volatile unsigned long timer0_millis_count = 0; 

#if USE_MOUSE
static inline uint8_t read_mouse_bit ();
static uint8_t read_mouse_byte ();
#endif

/*----------------------------------------------------------------------------*/

ISR (TIMER0_OVF_vect)
{
    timer0_millis_count++;
}

/*----------------------------------------------------------------------------*/

static inline uint32_t millis (void) __attribute__((always_inline, unused));
static inline uint32_t millis (void)
{
    uint32_t out; 
    asm volatile (
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

/*----------------------------------------------------------------------------*/

// The ISR for the external PS2 KB interrupt
ISR (INT0_vect)
{
    static uint8_t  bitcount = 0;
    static uint8_t  incoming = 0;
    static uint32_t prev_ms  = 0;
    uint32_t        now_ms;
    uint8_t         n, val;

    LED_ON;
    val = PS2KBDATA_ISHI;
    now_ms = millis();

    if (now_ms - prev_ms > 250)
    {
        bitcount = 0;
        incoming = 0;
    }
    prev_ms = now_ms;
    n = bitcount - 1;
    if (n <= 7)
        incoming |= (val << n);

    bitcount++;
    if (bitcount == 11) 
    {
        uint8_t i = kb_head + 1;
        if (i >= BUFFER_SIZE) i = 0;
        if (i != kb_tail)
        {
            kb_buffer[i] = incoming;
            kb_head = i;
        }
        bitcount = 0;
        incoming = 0;
    }

    LED_OFF;
}

/*----------------------------------------------------------------------------*/

// The ISR for the external PS2 Mouse interrupt
#if USE_MOUSE
ISR (INT1_vect)
{
    static uint8_t  bitcount = 0;
    static uint8_t  incoming = 0;
    static uint32_t prev_ms  = 0;
    uint32_t        now_ms;
    uint8_t         n, val;

    LED_ON;

    val = PS2MSDATA_ISHI;
    now_ms = millis();

    if (now_ms - prev_ms > 250)
    {
        bitcount = 0;
        incoming = 0;
    }
    prev_ms = now_ms;
    n = bitcount - 1;
    if (n <= 7)
        incoming |= (val << n);

    bitcount++;
    if (bitcount == 11) 
    {
        uint8_t i = ms_head + 1;
        if (i >= BUFFER_SIZE) i = 0;
        if (i != ms_tail)
        {
            ms_buffer[i] = incoming;
            ms_head = i;
        }
        bitcount = 0;
        incoming = 0;
    }

    LED_OFF;
}
#endif

/*----------------------------------------------------------------------------*/

    static inline uint8_t
get_scan_code (void)
{
    uint8_t c = 0, i;

    i = kb_tail;
    if (i == kb_head)
        return 0;
    i++;
    if (i >= BUFFER_SIZE) 
        i = 0;
    c = kb_buffer[i];
    kb_tail = i;

    return c;
}

/*----------------------------------------------------------------------------*/

#if USE_MOUSE
    static inline uint8_t
get_mouse_code (void)
{
    uint8_t c = 0, i;

    i = ms_tail;
    if (i == ms_head)
        return 0;
    i++;
    if (i >= BUFFER_SIZE) 
        i = 0;
    c = ms_buffer[i];
    ms_tail = i;

    return c;
}
#endif

/*----------------------------------------------------------------------------*/

    static inline uint8_t
get_special_scancode (uint8_t ps2_scancode)
{
    switch (ps2_scancode)
    {
        case 0x11:
            return 0x51;
            break;
        case 0x14:
            return 0x00;
            break;
        case 0x1F:
            return 0x54;
            break;
        case 0x27:
            return 0x53;
            break;
        case 0x4A:
            return 0x28;
            break;
        case 0x69:
            return 0x00;
            break;
        case 0x6B:
            return 0x09;
            break;
        case 0x6C:
            return 0x7E;
            break;
        case 0x70:
            return 0x00;
            break;
        case 0x71:
            return 0x00;
            break;
        case 0x72:
            return 0x0F;
            break;
        case 0x74:
            return 0x10;
            break;
        case 0x75:
            return 0x16;
            break;
        case 0x7A:
            return 0x01;
            break;
        case 0x7D:
            return 0x19;
            break;

        default:
            break;
    }

    return 0x00;
}

/*----------------------------------------------------------------------------*/

    static inline void
send_response (
    register uint8_t resp_lo, 
    register uint8_t resp_mid, 
    register uint8_t resp_hi)
{
    register uint8_t i = 0;

    for (i = 0; i < 8; i++)
    {
        if (resp_lo & ((uint32_t)1 << i))
            TO_NEXT_HI;
        else
            TO_NEXT_LO;
        _delay_us (TIMING);
    }
    for (i = 0; i < 8; i++)
    {
        if (resp_mid & ((uint32_t)1 << i))
            TO_NEXT_HI;
        else
            TO_NEXT_LO;
        _delay_us (TIMING);
    }
    for (i = 0; i < 6; i++)
    {
        if (resp_hi & ((uint32_t)1 << i))
            TO_NEXT_HI;
        else
            TO_NEXT_LO;
        _delay_us (TIMING);
    }
    TO_NEXT_HI;
}

/*----------------------------------------------------------------------------*/

    static inline uint32_t
get_modifier (uint8_t code, uint8_t right)
{
    uint32_t mod = 0;

    switch (code)
    {
        case 0x12: /* L SHIFT */
            mod = 0x00002000;
            break;
        case 0x59: /* R SHIFT */
            mod = 0x00004000;
            break;
        case 0x14: /* CTRL */
            if (right)
                mod = 0x00010000; 
            else
                mod = 0x00008000;
            break;
        case 0x11: /* ALT */
            if (right)
                mod = 0x00020000; 
            else
                mod = 0x00040000;
            break;
        default:
            mod = 0x00000000;
            break;
    }

    return mod;
}

/*----------------------------------------------------------------------------*/
#if USE_MOUSE
#define PULL_CLK_HI    { PS2MSCLK_CONFIG_IN; PS2MSCLK_HI; }
#define PULL_DATA_HI   { PS2MSDATA_CONFIG_IN; PS2MSDATA_HI; }

#define PULL_CLK_LO    { PS2MSCLK_CONFIG_OUT; PS2MSCLK_LO; }
#define PULL_DATA_LO   { PS2MSDATA_CONFIG_OUT; PS2MSDATA_LO; }

static void write_mouse (uint8_t data)
{
    uint8_t i      = 0;
    uint8_t parity = 1;

    PULL_DATA_HI;
    PULL_CLK_HI;
    _delay_us (300);
    PULL_CLK_LO;
    _delay_us (300);
    PULL_DATA_LO;
    _delay_us (10);
    PULL_CLK_HI;  // Start Bit
    _delay_us (10);
    while (PS2MSCLK_ISHI);// wait for mouse to take control of clock)
    // clock is low, and we are clear to send data 
    for (i = 0; i < 8; i++)
    {
        if (data & 0x01) {
            PULL_DATA_HI;
        } else {
            PULL_DATA_LO;
        }
        // wait for clock cycle 
        while (!PS2MSCLK_ISHI);
        while (PS2MSCLK_ISHI);
        parity = parity ^ (data & 0x01);
        data = data >> 1;
    }  
    // parity 
    if (parity) {
        PULL_DATA_HI;
    } else {
        PULL_DATA_LO;
    }
    while (!PS2MSCLK_ISHI);
    while (PS2MSCLK_ISHI);
    PULL_DATA_HI;
    _delay_us (50);

    while (PS2MSCLK_ISHI);
    while ((!PS2MSCLK_ISHI) || (!PS2MSDATA_ISHI)); // wait for mouse to switch modes
    PULL_CLK_LO; // put a hold on the incoming data.
}

/*----------------------------------------------------------------------------*/

static inline uint8_t read_mouse_bit ()
{
    uint8_t bit;

    while (PS2MSCLK_ISHI);
    bit = PS2MSDATA_ISHI;
    while (!PS2MSCLK_ISHI);

    return bit;
}

/*----------------------------------------------------------------------------*/

static uint8_t read_mouse_byte ()
{
    uint8_t data = 0;
    uint8_t i    = 0;

    PULL_CLK_HI;
    PULL_DATA_HI;
    _delay_us (50);

    while (PS2MSCLK_ISHI);

    _delay_us (5);
    while (!PS2MSCLK_ISHI);
    for (i = 0; i < 8; i++)
        data |= (read_mouse_bit () << i);

    read_mouse_bit ();
    read_mouse_bit ();

    return data;
}

/*----------------------------------------------------------------------------*/

static void init_mouse ()
{
    _delay_ms (20);
    write_mouse (0xFF);
    read_mouse_byte ();
    _delay_ms (20);
    read_mouse_byte ();
    read_mouse_byte ();
    _delay_ms (20);

    /* turn the sample rate way down */
    write_mouse (0xF3);
    read_mouse_byte ();
    write_mouse (0x0A);
    read_mouse_byte ();

    write_mouse (0xF4);
    read_mouse_byte ();

    _delay_us (100);
}
#endif

/*----------------------------------------------------------------------------*/

int main (void)
{
    uint32_t modifiers = 0x00000000;
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

#if USE_MOUSE
    PULL_DATA_HI;
    PULL_CLK_HI;

    init_mouse ();

    ms_head = 0;
    ms_tail = 0;
#endif

    kb_head = 0;
    kb_tail = 0;

    // falling edge for PS2 KB/Mouse int
    EICRA = (EICRA & 0xFC) | (0x02&0x03);
    EIMSK |= (1 << PS2KBCLK);
#if USE_MOUSE
    EIMSK |= (1 << PS2MSCLK);
#endif

    sei ();

    while (1)
    {
        uint32_t val;
        uint8_t i;

        LED_OFF;
        while (FROM_NEXT_ISHI);

        val = 0;
        _delay_us (TIMING/2);
        for (i = 0; i < 6; i++)
        {
            if (FROM_NEXT_ISHI)
                val |= ((uint32_t)1 << i); 
            _delay_us (TIMING);
        }   

        switch (val)
        {
            case 0x00000020: /* KB QUERY */
            case 0x00000022: /* MOUSE QUERY */
                for (i = 6; i < 9; i++)
                {
                    if (FROM_NEXT_ISHI)
                        val |= ((uint32_t)1 << i); 
                    _delay_us (TIMING);
                }   
                break;
            case 0x0000001E: /* KB RESET */
            case 0x00000000: /* LED CMD */
            default:
                for (i = 6; i < 22; i++)
                {
                    if (FROM_NEXT_ISHI)
                        val |= ((uint32_t)1 << i);
                    _delay_us (TIMING);
                }
                break;
        }

        if (val == 0x00000020)
        {
            uint32_t mod  = 0;
            uint32_t resp = 0x00300600;
            uint8_t ps2_scancode = get_scan_code ();

            /* send kb response */
            if (ps2_scancode && (ps2_scancode < 0x83))
            {
                if ((mod = get_modifier (ps2_scancode, 0)))
                {
                    modifiers |= mod;
                    resp = 0x00300600;
                    goto send_kb_response;
                }
                resp = 0x00280400;
            }
            else if (ps2_scancode == 0xE0)
            {
                /* multi-byte scancode */
                ps2_scancode = get_scan_code ();

                if (0 == ps2_scancode)
                {
                    kb_tail--;
                    resp = 0x00300600;
                    goto send_kb_response;
                }
                else if (ps2_scancode == 0xF0)
                {
                    ps2_scancode = get_scan_code ();
                    if (0 == ps2_scancode)
                    {
                        kb_tail -= 2;
                        resp = 0x00300600;
                        goto send_kb_response;
                    }

                    if ((mod = get_modifier (ps2_scancode, 1)))
                    {
                        modifiers &= ~mod;
                        resp = 0x00300600;
                        goto send_kb_response;
                    }
                    resp = 0x00280500;
                }
                else
                {
                    if ((mod = get_modifier (ps2_scancode, 1)))
                    {
                        modifiers |= mod;
                        resp = 0x00300600;
                        goto send_kb_response;
                    }
                    resp = 0x00280400;
                }
            }
            else if (ps2_scancode == 0xF0)
            {
                /* key up scancdoe */
                ps2_scancode = get_scan_code ();

                if (0 == ps2_scancode)
                {
                    kb_tail--; /* put the code back */
                    resp = 0x00300600;
                    goto send_kb_response;
                }

                if ((mod = get_modifier (ps2_scancode, 0)))
                {
                    modifiers &= ~mod;
                    resp = 0x00300600;
                    goto send_kb_response;
                }

                resp = 0x00280500;
            }

            if (0x00300600 != resp)
            {
                resp |= 2*ps2_next_scancodes[ps2_scancode];
                resp |= modifiers;
                LED_ON;
            }

send_kb_response:
            _delay_us (TIMING*4.5);
            send_response (resp & 0x000000FF, (resp & 0x0000FF00)>>8, (resp & 0x00FF0000)>>16);
        }
        else if (val == 0x00000022)
        {
            uint32_t resp = 0x00300600;
#if USE_MOUSE
            uint8_t  ps2_mouse_code[3];

            ps2_mouse_code[0] = get_mouse_code (); /* status */
            if (0 == ps2_mouse_code[0])
                goto send_ms_response;

            ps2_mouse_code[1] = get_mouse_code (); /* x */
            if (0 == ps2_mouse_code[1])
            {
                ms_tail--; /* put status byte back */
                goto send_ms_response;
            }
            ps2_mouse_code[2] = get_mouse_code (); /* y */
            if (0 == ps2_mouse_code[2])
            {
                ms_tail -= 2; /* put status and x byte back */
                goto send_ms_response;
            }

            resp = 0x00200400;

            /* right button */
            if (ps2_mouse_code[0] & 0x01)
                resp |= 0x00001000;
            else
                resp &= ~0x00001000;

            /* left button */
            if (ps2_mouse_code[0] & 0x02)
                resp |= 0x00000002;
            else
                resp &= ~0x00000002;

            /* x movement */
            if (ps2_mouse_code[1])
                resp |= 0x04;

            /* +/- */
            if (ps2_mouse_code[0] & 0x10)
                resp |= 0x1f8;

            /* y movement */
            if (ps2_mouse_code[2])
                resp |= 0x2000;

            if (ps2_mouse_code[0] & 0x20)
                resp |= 0xFC000;

            /* PS2 mouse cmd is ready */

send_ms_response:
#endif
            _delay_us (TIMING*4.5);
            send_response (resp & 0x000000FF, (resp & 0x0000FF00)>>8, (resp & 0x00FF0000)>>16);
        }
        else if (val == 0x00000FDE)
        {
            /* KB reset */
        }
        _delay_us (TIMING);
    }

    return 0;
}


