/*
 * remote.h
 *
 * This file is part of the QtSixA, the Sixaxis Joystick Manager
 * Copyright 2011 Filipe Coelho <falktx@gmail.com>
 *
 * QtSixA can be redistributed and/or modified under the terms of the GNU General
 * Public License (Version 2), as published by the Free Software Foundation.
 * A copy of the license is included in the QtSixA source code, or can be found
 * online at www.gnu.org/licenses.
 *
 * QtSixA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 */

#ifndef REMOTE_H
#define REMOTE_H

#define REMOTE_KEY_1           0x00
#define REMOTE_KEY_2           0x01
#define REMOTE_KEY_3           0x02
#define REMOTE_KEY_4           0x03
#define REMOTE_KEY_5           0x04
#define REMOTE_KEY_6           0x05
#define REMOTE_KEY_7           0x06
#define REMOTE_KEY_8           0x07
#define REMOTE_KEY_9           0x08
#define REMOTE_KEY_0           0x09
#define REMOTE_KEY_ENTER       0x0B
#define REMOTE_KEY_RETURN      0x0E
#define REMOTE_KEY_CLEAR       0x0F
#define REMOTE_KEY_EJECT       0x16
#define REMOTE_KEY_TIME        0x28
#define REMOTE_KEY_PREV        0x30
#define REMOTE_KEY_NEXT        0x31
#define REMOTE_KEY_PLAY        0x32
#define REMOTE_KEY_SCAN_PREV   0x33
#define REMOTE_KEY_SCAN_FORW   0x34
#define REMOTE_KEY_STOP        0x38
#define REMOTE_KEY_PAUSE       0x39
#define REMOTE_KEY_POP_UP_MENU 0x40
#define REMOTE_KEY_SLOW_PREV   0x60
#define REMOTE_KEY_SLOW_FORW   0x61
#define REMOTE_KEY_SUBTITLE    0x63
#define REMOTE_KEY_AUDIO       0x64
#define REMOTE_KEY_ANGLE       0x65
#define REMOTE_KEY_DISPLAY     0x70
#define REMOTE_KEY_BLUE        0x80
#define REMOTE_KEY_RED         0x81
#define REMOTE_KEY_GREEN       0x82
#define REMOTE_KEY_YELLOW      0x83
#define REMOTE_KEY_TOP_MENU    0x1A

#define REMOTE_KEY_JS_SELECT   0x50
#define REMOTE_KEY_JS_L3       0x51
#define REMOTE_KEY_JS_R3       0x52
#define REMOTE_KEY_JS_START    0x53
#define REMOTE_KEY_JS_UP       0x54
#define REMOTE_KEY_JS_RIGHT    0x55
#define REMOTE_KEY_JS_DOWN     0x56
#define REMOTE_KEY_JS_LEFT     0x57
#define REMOTE_KEY_JS_L2       0x58
#define REMOTE_KEY_JS_R2       0x59
#define REMOTE_KEY_JS_L1       0x5A
#define REMOTE_KEY_JS_R1       0x5B
#define REMOTE_KEY_JS_TRIANGLE 0x5C
#define REMOTE_KEY_JS_CIRCLE   0x5D
#define REMOTE_KEY_JS_CROSS    0x5E
#define REMOTE_KEY_JS_SQUARE   0x5F

#define REMOTE_KEY_OPTIONS     REMOTE_KEY_JS_TRIANGLE
#define REMOTE_KEY_BACK        REMOTE_KEY_JS_CIRCLE
#define REMOTE_KEY_VIEW        REMOTE_KEY_JS_SQUARE

#define REMOTE_KEYMODE_NUMBERIC    0x01
#define REMOTE_KEYMODE_DVD         0x02
#define REMOTE_KEYMODE_DIRECTIONAL 0x04
#define REMOTE_KEYMODE_MULTIMEDIA  0x08

void do_joystick(int fd, unsigned char* buf, struct dev_joystick joystick);
void do_remote(int fd, unsigned char* buf, int modes);
void do_input(int fd, unsigned char* buf, struct dev_input input);

#endif // REMOTE_H
