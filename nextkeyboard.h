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

#include "usb_keyboard.h"

static const uint8_t nextkbd_keydesc_us[] = {
    0, /* space to start at 1 */
    0, /* 1: Brightness DOWN */ 
    0, /* 2: Volume DOWN */
    KEY_BACKSLASH,
    KEY_RIGHT_BRACE,
    KEY_LEFT_BRACE,
    KEY_I,
    KEY_O,
    KEY_P,
    KEY_LEFT,
    0, /* 10: ? */
    KEYPAD_0,
    KEYPAD_PERIOD,
    KEYPAD_ENTER,
    0, /* 14: ? */
    KEY_DOWN,
    KEY_RIGHT,
    KEYPAD_1,
    KEYPAD_4,
    KEYPAD_6,
    KEYPAD_3,
    KEYPAD_PLUS,
    KEY_UP,
    KEYPAD_2,
    KEYPAD_5,
    0, /* 25: Brightness UP */
    0, /* 26: Volume UP */
    KEY_BACKSPACE,
    KEY_EQUAL,
    KEY_MINUS,
    KEY_8,
    KEY_9,
    KEY_0,
    KEYPAD_7,
    KEYPAD_8,
    KEYPAD_9,
    KEYPAD_MINUS,
    KEYPAD_ASTERIX,
    KEY_TILDE,
    KEY_EQUAL,
    KEYPAD_SLASH,
    0, /* 41: ? */
    KEY_ENTER,
    KEY_QUOTE,
    KEY_SEMICOLON,
    KEY_L,
    KEY_COMMA,
    KEY_PERIOD,
    KEY_SLASH,
    KEY_Z,
    KEY_X,
    KEY_C,
    KEY_V,
    KEY_B,
    KEY_M,
    KEY_N,
    KEY_SPACE,
    KEY_A,
    KEY_S,
    KEY_D,
    KEY_F,
    KEY_G,
    KEY_K,
    KEY_J,
    KEY_H,
    KEY_TAB,
    KEY_Q,
    KEY_W,
    KEY_E,
    KEY_R,
    KEY_U,
    KEY_Y,
    KEY_T,
    KEY_ESC,
    KEY_1,
    KEY_2,
    KEY_3,
    KEY_4,
    KEY_7,
    KEY_6,
    KEY_5,
    0, 0, 0, 0, 0, 0, 0, 0, 0, /* 81-89: ? */
    KEY_LEFT_SHIFT,
    KEY_RIGHT_SHIFT,
    KEY_LEFT_ALT,
    KEY_RIGHT_ALT,
    KEY_LEFT_CTRL,
    0, /* 95: LEFT_CMD */
    0 /* 96: RIGHT_CMD */
};

