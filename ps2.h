/*
  PS2Keyboard.h - PS2Keyboard library
  Copyright (c) 2007 Free Software Foundation.  All right reserved.
  Written by Christian Weichel <info@32leaves.net>

  ** Mostly rewritten Paul Stoffregen <paul@pjrc.com>, June 2010
  ** Modified for use with Arduino 13 by L. Abraham Smith, <n3bah@microcompdesign.com> * 
  ** Modified for easy interrup pin assignement on method begin(datapin,irq_pin). Cuningan <cuninganreset@gmail.com> **

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


#ifndef PS2Keyboard_h
#define PS2Keyboard_h

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

/* 0 == no map */
uint8_t ps2_next_scancodes [0x84] = {
0, /* space to push into correct position */
0, /* F9 */
0, /* ? */
0, /* F5 */
0, /* F3 */
0, /* F1 */
0, /* F2 */
0, /* F12 */
0, /* ? */
0, /* F10 */
0, /* F8 */
0, /* F6 */
0, /* F4 */
0x41, /* TAB */
0x26, /* ~ */
0, 0, /* ? */
0x52, /* L ALT */
0x56, /* L SHIFT */
0, /* ? */
0x57, /* L CTRL */
0x42, /* Q */
0x4A, /* 1 */
0, 0, 0, /* ? */
0x31, /* Z */
0x3A, /* S */
0x39, /* A */
0x43, /* W */
0x4B, /* 2 */
0, 0, /* ? */
0x33, /* C */
0x32, /* X */
0x3B, /* D */
0x44, /* E */
0x4D, /* 4 */
0x4C, /* 3 */
0, 0, /* ? */
0x38, /* SPACE */
0x34, /* V */
0x3C, /* F */
0x48, /* T */
0x45, /* R */
0x50, /* 5 */
0, 0, /* ? */
0x37, /* N */
0x35, /* B */
0x40, /* H */
0x3D, /* G */
0x47, /* Y */
0x4F, /* 6 */
0, 0, 0, /* ? */
0x36, /* M */
0x3F, /* J */
0x46, /* U */
0x4E, /* 7 */
0x1E, /* 8 */
0, 0, /* ? */
0x2E, /* , */
0x3E, /* K */
0x06, /* I */
0x07, /* O */
0x20, /* 0 */
0x1F, /* 9 */
0, 0, /* ? */
0x2F, /* . */
0x30, /* / */
0x2D, /* L */
0x2C, /* ; */
0x08, /* P */
0x1D, /* - */
0, 0, 0, 0, 0, /* ? */
0x05, /* [ */
0x1C, /* = */
0, 0, /* ? */
0, /* CAPS LOCK */
0x55, /* R SHIFT */
0x2A, /* ENTER */
0x04, /* ] */
0, /* ? */
0x03, /* \ */
0, 0, 0, 0, 0, 0, 0, 0, /* ? */
0x1B, /* BACKSPACE */
0, 0, /* ? */
0x11, /* N 1 */
0, /* ? */
0x12, /* N 4 */
0x21, /* N 7 */
0, 0, 0, /* ? */
0x0B, /* N 0 */
0x0C, /* N . */
0x17, /* N 2 */
0x18, /* N 5 */
0x13, /* N 6 */
0x22, /* N 8 */
0x49, /* ESC */
0, /* NUM LOCK */
0, /* F11 */
0x15, /* N + */
0x14, /* N 3 */
0x24, /* N - */
0x25, /* N * */
0x23, /* N 9 */
0, /* SCROLL LOCK */
0, 0, 0, 0, /* ? */
0 /* F7 */ 
};

/* scancodes that start with 0xE0 */
uint8_t ps2_next_special [] = {
0x11,0x51, /* R ALT */
0x14,0x00, /* R CTRL */
0x1F,0x54, /* L GUI */
0x27,0x53, /* R GUI */
0x4A,0x28, /* N / */
0x69,0x00, /* END */
0x6B,0x09, /* L ARROW */
0x6C,0x7E, /* HOME */
0x70,0x00, /* INSERT */
0x71,0x00, /* DELETE */
0x72,0x0F, /* D ARROW */
0x74,0x10, /* R ARROW */
0x75,0x16, /* U ARROW */
0x7A,0x01, /* PG DN */
0x7D,0x19 /* PG UP */
};

#endif


