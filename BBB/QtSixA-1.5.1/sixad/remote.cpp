/*
 * remote.cpp
 *
 * This file is part of the QtSixA, the Sixaxis Joystick Manager
 * Copyright 2008-11 Filipe Coelho <falktx@gmail.com>
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

#include "remote.h"
#include "uinput.h"
#include "shared.h"

#include <syslog.h>
#include <sys/socket.h>
#include <unistd.h>

int b1, b2, b3;
int last_jb1 = 0;
int last_jb2 = 0;
int last_jb3 = 0;
int last_rb1 = 0xff;
int last_ib1 = 0;
int last_ib2 = 0;
int last_ib3 = 0;
int last_key = -1;

static int remote2key(int key, int modes) {
    switch (key) {
    // Numeric keys
    case REMOTE_KEY_0:
        return (modes & REMOTE_KEYMODE_NUMBERIC) ? KEY_0 : 0;
    case REMOTE_KEY_1:
        return (modes & REMOTE_KEYMODE_NUMBERIC) ? KEY_1 : 0;
    case REMOTE_KEY_2:
        return (modes & REMOTE_KEYMODE_NUMBERIC) ? KEY_2 : 0;
    case REMOTE_KEY_3:
        return (modes & REMOTE_KEYMODE_NUMBERIC) ? KEY_3 : 0;
    case REMOTE_KEY_4:
        return (modes & REMOTE_KEYMODE_NUMBERIC) ? KEY_4 : 0;
    case REMOTE_KEY_5:
        return (modes & REMOTE_KEYMODE_NUMBERIC) ? KEY_5 : 0;
    case REMOTE_KEY_6:
        return (modes & REMOTE_KEYMODE_NUMBERIC) ? KEY_6 : 0;
    case REMOTE_KEY_7:
        return (modes & REMOTE_KEYMODE_NUMBERIC) ? KEY_7 : 0;
    case REMOTE_KEY_8:
        return (modes & REMOTE_KEYMODE_NUMBERIC) ? KEY_8 : 0;
    case REMOTE_KEY_9:
        return (modes & REMOTE_KEYMODE_NUMBERIC) ? KEY_9 : 0;

    // DVD Keys
    case REMOTE_KEY_CLEAR:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_CLEAR : 0;
    case REMOTE_KEY_EJECT:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_EJECTCD : 0;
    case REMOTE_KEY_TIME:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_TIME : 0;
    case REMOTE_KEY_POP_UP_MENU:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_MENU : 0;
    case REMOTE_KEY_SUBTITLE:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_SUBTITLE : 0;
    case REMOTE_KEY_AUDIO:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_AUDIO : 0;
    case REMOTE_KEY_ANGLE:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_ANGLE : 0;
    case REMOTE_KEY_DISPLAY:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_SCREEN : 0;
    case REMOTE_KEY_BLUE:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_BLUE : 0;
    case REMOTE_KEY_RED:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_RED : 0;
    case REMOTE_KEY_GREEN:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_GREEN : 0;
    case REMOTE_KEY_YELLOW:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_YELLOW : 0;
    case REMOTE_KEY_TOP_MENU:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_SETUP : 0;
#if 0
    case REMOTE_KEY_OPTIONS:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_ : 0;
    case REMOTE_KEY_BACK:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_ : 0;
    case REMOTE_KEY_VIEW:
        return (modes & REMOTE_KEYMODE_DVD) ? KEY_ : 0;
#endif

    // Directional Keys
    case REMOTE_KEY_ENTER:
        return (modes & REMOTE_KEYMODE_DIRECTIONAL) ? KEY_ENTER : 0;
    case REMOTE_KEY_RETURN:
        return (modes & REMOTE_KEYMODE_DIRECTIONAL) ? KEY_BACKSPACE : 0;
    case REMOTE_KEY_JS_UP:
        return (modes & REMOTE_KEYMODE_DIRECTIONAL) ? KEY_UP : 0;
    case REMOTE_KEY_JS_RIGHT:
        return (modes & REMOTE_KEYMODE_DIRECTIONAL) ? KEY_RIGHT : 0;
    case REMOTE_KEY_JS_DOWN:
        return (modes & REMOTE_KEYMODE_DIRECTIONAL) ? KEY_DOWN : 0;
    case REMOTE_KEY_JS_LEFT:
        return (modes & REMOTE_KEYMODE_DIRECTIONAL) ? KEY_LEFT : 0;

    // Multimedia Keys
    case REMOTE_KEY_PREV:
        return (modes & REMOTE_KEYMODE_MULTIMEDIA) ? KEY_BACK : 0;
    case REMOTE_KEY_NEXT:
        return (modes & REMOTE_KEYMODE_MULTIMEDIA) ? KEY_FORWARD : 0;
    case REMOTE_KEY_PLAY:
        return (modes & REMOTE_KEYMODE_MULTIMEDIA) ? KEY_PLAY : 0;
    case REMOTE_KEY_SCAN_PREV:
        return (modes & REMOTE_KEYMODE_MULTIMEDIA) ? KEY_REWIND : 0;
    case REMOTE_KEY_SCAN_FORW:
        return (modes & REMOTE_KEYMODE_MULTIMEDIA) ? KEY_FASTFORWARD : 0;
    case REMOTE_KEY_STOP:
        return (modes & REMOTE_KEYMODE_MULTIMEDIA) ? KEY_STOP : 0;
    case REMOTE_KEY_PAUSE:
        return (modes & REMOTE_KEYMODE_MULTIMEDIA) ? KEY_PAUSE : 0;
    case REMOTE_KEY_SLOW_PREV:
        return (modes & REMOTE_KEYMODE_MULTIMEDIA) ? KEY_FRAMEBACK : 0;
    case REMOTE_KEY_SLOW_FORW:
        return (modes & REMOTE_KEYMODE_MULTIMEDIA) ? KEY_FRAMEFORWARD : 0;

    default:
        return 0;
    }
}

void do_joystick(int fd, unsigned char* buf, struct dev_joystick joystick)
{
    b1 = buf[2];
    b2 = buf[3];
    b3 = buf[4];

    if (joystick.buttons) {
        //part1
        if (last_jb1 != b1) {
            uinput_send(fd, EV_KEY, BTN_JOYSTICK + 0, b1 & 0x01 ? 1 : 0);
            uinput_send(fd, EV_KEY, BTN_JOYSTICK + 1, b1 & 0x02 ? 1 : 0);
            uinput_send(fd, EV_KEY, BTN_JOYSTICK + 2, b1 & 0x04 ? 1 : 0);
            uinput_send(fd, EV_KEY, BTN_JOYSTICK + 3, b1 & 0x08 ? 1 : 0);
            uinput_send(fd, EV_KEY, BTN_JOYSTICK + 4, b1 & 0x10 ? 1 : 0);
            uinput_send(fd, EV_KEY, BTN_JOYSTICK + 5, b1 & 0x20 ? 1 : 0);
            uinput_send(fd, EV_KEY, BTN_JOYSTICK + 6, b1 & 0x40 ? 1 : 0);
            uinput_send(fd, EV_KEY, BTN_JOYSTICK + 7, b1 & 0x80 ? 1 : 0);
        }
        //part2
        if (last_jb2 != b2) {
            uinput_send(fd, EV_KEY, BTN_JOYSTICK +  8, b2 & 0x01 ? 1 : 0);
            uinput_send(fd, EV_KEY, BTN_JOYSTICK +  9, b2 & 0x02 ? 1 : 0);
            uinput_send(fd, EV_KEY, BTN_JOYSTICK + 10, b2 & 0x04 ? 1 : 0);
            uinput_send(fd, EV_KEY, BTN_JOYSTICK + 11, b2 & 0x08 ? 1 : 0);
            uinput_send(fd, EV_KEY, BTN_JOYSTICK + 12, b2 & 0x10 ? 1 : 0);
            uinput_send(fd, EV_KEY, BTN_JOYSTICK + 13, b2 & 0x20 ? 1 : 0);
            uinput_send(fd, EV_KEY, BTN_JOYSTICK + 14, b2 & 0x40 ? 1 : 0);
            uinput_send(fd, EV_KEY, BTN_JOYSTICK + 15, b2 & 0x80 ? 1 : 0);
        }
        //part3
        if (last_jb3 != b3) {
            uinput_send(fd, EV_KEY, BTN_JOYSTICK + 16, b3 & 0x01 ? 1 : 0);
        }

        if (b1 > 0 || b2 > 0 || b3 > 0) {
          set_active(true);
        }
    }

    last_jb1 = b1;
    last_jb2 = b2;
    last_jb3 = b3;

    uinput_send(fd, EV_SYN, SYN_REPORT, 0);
}

void do_remote(int fd, unsigned char* buf, int modes)
{
    b1 = buf[5];

    if (last_rb1 != b1) {
        int x = 0;
        if (b1 != 0xff) {
            last_key = remote2key(b1, modes);
            if (last_key > 0) {
                x = 1;
            }
        }
        uinput_send(fd, EV_KEY, last_key, x);
        set_active(true);
    }

    last_rb1 = b1;
    uinput_send(fd, EV_SYN, SYN_REPORT, 0);
}

void do_input(int fd, unsigned char* buf, struct dev_input input)
{
    b1 = buf[2];
    b2 = buf[3];
    b3 = buf[4];

    //part1
    if (last_ib1 != b1) {
        if (input.key_select) uinput_send(fd, EV_KEY, input.key_select, b1 & 0x01 ? 1 : 0);
        if (input.key_l3) uinput_send(fd, EV_KEY, input.key_l3, b1 & 0x02 ? 1 : 0);
        if (input.key_r3) uinput_send(fd, EV_KEY, input.key_r3, b1 & 0x04 ? 1 : 0);
        if (input.key_start) uinput_send(fd, EV_KEY, input.key_start, b1 & 0x08 ? 1 : 0);
        if (input.key_up) uinput_send(fd, EV_KEY, input.key_up, b1 & 0x10 ? 1 : 0);
        if (input.key_right) uinput_send(fd, EV_KEY, input.key_right, b1 & 0x20 ? 1 : 0);
        if (input.key_down) uinput_send(fd, EV_KEY, input.key_down, b1 & 0x40 ? 1 : 0);
        if (input.key_left) uinput_send(fd, EV_KEY, input.key_left, b1 & 0x80 ? 1 : 0);
    }
    //part2
    if (last_ib2 != b2) {
        if (input.key_l2) uinput_send(fd, EV_KEY, input.key_l2, b2 & 0x01 ? 1 : 0);
        if (input.key_r2) uinput_send(fd, EV_KEY, input.key_r2, b2 & 0x02 ? 1 : 0);
        if (input.key_l1) uinput_send(fd, EV_KEY, input.key_l1, b2 & 0x04 ? 1 : 0);
        if (input.key_r1) uinput_send(fd, EV_KEY, input.key_r1, b2 & 0x08 ? 1 : 0);
        if (input.key_tri) uinput_send(fd, EV_KEY, input.key_tri, b2 & 0x10 ? 1 : 0);
        if (input.key_cir) uinput_send(fd, EV_KEY, input.key_cir, b2 & 0x20 ? 1 : 0);
        if (input.key_cro) uinput_send(fd, EV_KEY, input.key_cro, b2 & 0x40 ? 1 : 0);
        if (input.key_squ) uinput_send(fd, EV_KEY, input.key_squ, b2 & 0x80 ? 1 : 0);
    }
    //part3
    if (last_ib3 != b3) {
        if (input.key_ps) uinput_send(fd, EV_KEY, input.key_ps, b3 & 0x01 ? 1 : 0);
    }

    if (b1 > 0 || b2 > 0 || b3 > 0) {
      set_active(true);
    }

    last_ib1 = b1;
    last_ib2 = b2;
    last_ib3 = b3;

    uinput_send(fd, EV_SYN, SYN_REPORT, 0);
}
