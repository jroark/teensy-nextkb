 /*
  * Copyright (c) 2012 John Roark <john.roark@gmail.com>
  *
  * This library is free software; you can redistribute it and/or
  * modify it under the terms of the GNU Lesser General Public
  * License as published by the Free Software Foundation; either
  * version 2.1 of the License, or (at your option) any later version.
  *
  * This library is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  * Lesser General Public License for more details.
  *
  * You should have received a copy of the GNU Lesser General Public
  * License along with this library; if not, write to the Free Software
  * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  *
  * based on work from http://m0115.web.fc2.com/ specifically http://m0115.web.fc2.com/next.jpg
  */
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#ifdef DEBUG
#include "usb_keyboard_debug.h"
#include "print.h"
#else
#include "usb_keyboard.h"
#endif

#include "nextkeyboard.h"


#define LED_ON    (PORTD |= (1<<6))
#define LED_OFF   (PORTD &= ~(1<<6))

#define LED_CONFIG      (DDRD |= (1<<6))
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

/* pins for input/output */
#define TOKB   0
#define FROMKB 1

#define TOKB_HI       (PORTB |= (1<<TOKB))
#define TOKB_LO       (PORTB &= ~(1<<TOKB))
#define FROMKB_HI     (PINB & (1<<FROMKB))

#define TOKB_CONFIG   (DDRB |= (1<<TOKB))
#define FROMKB_CONFIG (DDRB &= ~(1<<FROMKB))

#define TIMING 0.05

/*---------------------------------------------------------------------------*/
/*
zz123456789zz
--_____-___--
*/

static void query_kb ()
{
    TOKB_LO;
    _delay_ms (TIMING*5.0);
    TOKB_HI;
    _delay_ms (TIMING);
    TOKB_LO;
    _delay_ms (TIMING*3.0);
    TOKB_HI;
}

/*---------------------------------------------------------------------------*/
/*
zz1234567890123456789012zz
--_----_------__________--
*/

static void reset_kb ()
{
    TOKB_LO;
    _delay_ms (TIMING);
    TOKB_HI;
    _delay_ms (TIMING*4.0);
    TOKB_LO;
    _delay_ms (TIMING);
    TOKB_HI;
    _delay_ms (TIMING*6.0);
    TOKB_LO;
    _delay_ms (TIMING*10);
    TOKB_HI;
}

/*---------------------------------------------------------------------------*/
/*
zz1234567890
--_________---
*/

static void set_kbled (uint8_t r, uint8_t l)
{
    TOKB_LO;
    _delay_ms (TIMING *9);
    TOKB_HI;
    _delay_ms (TIMING *3);
    TOKB_LO;
    _delay_ms (TIMING);

    if (l)
        TOKB_HI;
    else 
        TOKB_LO;
    _delay_ms (TIMING);

    if (r)
        TOKB_HI;
    else 
        TOKB_LO;
    _delay_ms (TIMING);
    TOKB_LO;
    _delay_ms (TIMING *7);
    TOKB_HI;
}

/*---------------------------------------------------------------------------*/
/*
zz1234567890123456789zz
--_____-___-_________--
*/

static uint32_t getkb_response ()
{
    uint32_t data = 0;
    uint8_t  i    = 0;
    cli ();

    while (FROMKB_HI);

    _delay_ms (TIMING/2);
    for (i = 0; i < 22; i++)
    {
        if (FROMKB_HI)
            data |= ((uint32_t)1 << i);
        _delay_ms (TIMING);
    }
    sei ();
    return data;
}

/*---------------------------------------------------------------------------*/

int main (void)
{
    CPU_PRESCALE (0);
    LED_CONFIG;
    LED_OFF;

    usb_init ();
    while (!usb_configured ());

    _delay_ms (1500);

    TOKB_CONFIG;
    TOKB_HI;

    FROMKB_CONFIG;

    query_kb ();
    _delay_ms (5.0);
    reset_kb ();
    _delay_ms (8.0);

    query_kb ();
    _delay_ms (5.0);
    reset_kb ();
    _delay_ms (8.0);

    keyboard_modifier_keys = 0;
    memset (keyboard_keys, 0, 6);

    while (1)
    {
        uint32_t resp    = 0;
        uint8_t  keycode = 0;
        uint8_t  code    = 0;

        LED_OFF;
        _delay_ms (11.0);
        query_kb ();

        resp = getkb_response ();

        if (0x00200600 == resp) continue;

        LED_ON;

        keycode = resp & 0x000000FF;
        keycode /= 2;

        if (resp & 0x00001000)
            keyboard_modifier_keys |= KEY_LEFT_GUI;
        else
            keyboard_modifier_keys &= ~KEY_LEFT_GUI;

        if (resp & 0x00002000) 
        {
            keyboard_modifier_keys |= KEY_LEFT_SHIFT;
            set_kbled (0, 1);
        }
        else
        {
            keyboard_modifier_keys &= ~KEY_LEFT_SHIFT;
            set_kbled (0, 0);
        }

        if (resp & 0x00004000)
        {
            keyboard_modifier_keys |= KEY_RIGHT_SHIFT;
            set_kbled (1, 0);
        }
        else
        {
            keyboard_modifier_keys &= ~KEY_RIGHT_SHIFT;
            set_kbled (0, 0);
        }


        if (resp & 0x00008000)
            keyboard_modifier_keys |= KEY_LEFT_CTRL;
        else
            keyboard_modifier_keys &= ~KEY_LEFT_CTRL;

        if (resp & 0x00010000)
            keyboard_modifier_keys |= KEY_RIGHT_CTRL;
        else
            keyboard_modifier_keys &= ~KEY_RIGHT_CTRL;

        if (resp & 0x00020000)
            keyboard_modifier_keys |= KEY_LEFT_ALT;
        else
            keyboard_modifier_keys &= ~KEY_LEFT_ALT;

        if (resp & 0x00040000)
            keyboard_modifier_keys |= KEY_RIGHT_ALT;
        else
            keyboard_modifier_keys &= ~KEY_RIGHT_ALT;

        if (0 == keycode) continue; 

        code = nextkbd_keydesc_us[keycode];

        if ((resp & 0x00000F00) == 0x00000400)
            keyboard_keys[0] = code;
        else if ((resp & 0x00000F00) == 0x00000500)
            keyboard_keys[0] = 0;

        usb_keyboard_send ();
    }
}

