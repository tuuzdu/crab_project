/*
 * sixad-raw.cpp
 *
 * This file is part of the QtSixA, the Sixaxis Joystick Manager
 * Copyright 2008-10 Filipe Coelho <falktx@gmail.com>
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

#include "shared.h"
#include "sixaxis.h"
#include "uinput.h"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <syslog.h>
#include <unistd.h>

#define KEYMOTE_KEY_SELECT   1 << 0
#define KEYMOTE_KEY_START    1 << 1
#define KEYMOTE_KEY_L3       1 << 2
#define KEYMOTE_KEY_R3       1 << 3
#define KEYMOTE_KEY_PS       1 << 4

#define KEYMOTE_KEY_SQUARE   1 << 0
#define KEYMOTE_KEY_CROSS    1 << 1
#define KEYMOTE_KEY_CIRCLE   1 << 2
#define KEYMOTE_KEY_TRIANGLE 1 << 3
#define KEYMOTE_KEY_L1       1 << 4
#define KEYMOTE_KEY_R1       1 << 5
#define KEYMOTE_KEY_L2       1 << 6
#define KEYMOTE_KEY_R2       1 << 7

int main(int argc, char **argv)
{
    int fd, nr;
    unsigned char buf[128];
    struct uinput_fd *ufd;
    struct device_settings settings;

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " /dev/hidrawX" << std::endl;
        return 1;
    }

    if ((fd = open(argv[1], O_RDONLY|O_NONBLOCK)) < 0) {
        std::cerr << "sixad-3in1::open(hidrawX) - failed to open hidraw device" << std::endl;
        return 1;
    }

    nr=read(fd, buf, sizeof(buf));
    if (nr < 0 && errno != EAGAIN) {
        std::cerr << "sixad-3in1::read(fd) - failed to read from device" << std::endl;
        return 1;
    }

    if (nr != -1 && nr != 19) {
        std::cerr <<  "sixad-3in1::read(fd) - not a 3in1 keymote (nr = " << nr << ")" << std::endl;
        return 1;
    }

    open_log("sixad-3in1");

    memset(&settings, 0, sizeof(device_settings));

    settings.led.enabled = 0;
    settings.joystick.enabled = 0;
    settings.remote.enabled = 0;
    settings.input.enabled = 1;
    settings.input.key_select = 29;
    settings.input.key_l3 = 0;
    settings.input.key_r3 = 0;
    settings.input.key_start = 56;
    settings.input.key_up = 103;
    settings.input.key_right = 106;
    settings.input.key_down = 108;
    settings.input.key_left = 105;
    settings.input.key_l2 = 102;
    settings.input.key_r2 = 107;
    settings.input.key_l1 = 67;
    settings.input.key_r1 = 87;
    settings.input.key_tri = 14;
    settings.input.key_cir = 273;
    settings.input.key_squ = 28;
    settings.input.key_cro = 272;
    settings.input.key_ps = 42;
    settings.input.axis_l_type = 3;
    settings.input.axis_l_up = 1;
    settings.input.axis_l_right = 0;
    settings.input.axis_l_down = 0;
    settings.input.axis_l_left = 0;
    settings.input.axis_r_type = 3;
    settings.input.axis_r_up = 8;
    settings.input.axis_r_right = 6;
    settings.input.axis_r_down = 0;
    settings.input.axis_r_left = 0;
    settings.input.axis_speed = 9;
    settings.input.use_lr3 = 1;
    settings.rumble.enabled = 0;

    ufd = uinput_open(DEV_TYPE_3IN1, "3in1", settings);

    if (ufd->js != 0 || ufd->mk == 0) {
        syslog(LOG_ERR, "Error! something  is not right...");
        return 1;
    }

    syslog(LOG_INFO, "Connected 'Brooklyn 3in1 KeyMote'");

    int b0, b1, lx, ly, rx, ry;
    int kUp, kDown, kLeft, kRight;
    int last_b1 = 0;
    int last_ib0 = 0;
    int last_ib1 = 0;
    int last_lx = 0;
    int last_ly = 0;
    int last_rx = 0;
    int last_ry = 0;
    int last_kUp = 0;
    int last_kDown = 0;
    int last_kLeft = 0;
    int last_kRight = 0;
    bool lr3_axis = true;
    bool lr3_buttons = true;
    int rw_timer = 0;

    while (true) {
        nr=read(fd, buf, sizeof(buf));

        if (nr == 19) {
          // read successful
          b0 = buf[0];
          b1 = buf[1];
          lx = buf[3] - 128;
          ly = buf[4] - 128;
          rx = buf[5] - 128;
          ry = buf[6] - 128;
          kRight = buf[7];
          kLeft  = buf[8];
          kUp    = buf[9];
          kDown  = buf[10];

        } else {
          if (errno != EAGAIN) {
            std::cerr <<  "sixad-3in1::read(fd, buf) - failed to read from device" << std::endl;
            break;
          }

          b0 = last_ib0;
          b1 = last_ib1;
          lx = last_lx;
          ly = last_ly;
          rx = last_rx;
          ry = last_ry;
          kRight = last_kRight;
          kLeft  = last_kLeft;
          kUp    = last_kUp;
          kDown  = last_kDown;
        }

        //lr3 enable/disable
        if ((b1 & KEYMOTE_KEY_L3) && b1 != last_b1)
          lr3_axis = !lr3_axis;

        if ((b1 & KEYMOTE_KEY_R3) && b1 != last_b1)
          lr3_buttons = !lr3_buttons;

        last_b1 = b1;

        //buttons
        if (lr3_buttons) {
            //part1
            if (last_ib0 != b0) {
                uinput_send(ufd->mk, EV_KEY, settings.input.key_l2, b0 & KEYMOTE_KEY_L2 ? 1 : 0);
                uinput_send(ufd->mk, EV_KEY, settings.input.key_r2, b0 & KEYMOTE_KEY_R2 ? 1 : 0);
                uinput_send(ufd->mk, EV_KEY, settings.input.key_l1, b0 & KEYMOTE_KEY_L1 ? 1 : 0);
                uinput_send(ufd->mk, EV_KEY, settings.input.key_r1, b0 & KEYMOTE_KEY_R1 ? 1 : 0);
                uinput_send(ufd->mk, EV_KEY, settings.input.key_tri, b0 & KEYMOTE_KEY_TRIANGLE ? 1 : 0);
                uinput_send(ufd->mk, EV_KEY, settings.input.key_cir, b0 & KEYMOTE_KEY_CIRCLE ? 1 : 0);
                uinput_send(ufd->mk, EV_KEY, settings.input.key_cro, b0 & KEYMOTE_KEY_CROSS ? 1 : 0);
                uinput_send(ufd->mk, EV_KEY, settings.input.key_squ, b0 & KEYMOTE_KEY_SQUARE ? 1 : 0);
            }
            //part2
            if (last_ib1 != b1) {
                uinput_send(ufd->mk, EV_KEY, settings.input.key_select, b1 & KEYMOTE_KEY_SELECT ? 1 : 0);
                uinput_send(ufd->mk, EV_KEY, settings.input.key_start, b1 & KEYMOTE_KEY_START ? 1 : 0);
            }
            uinput_send(ufd->mk, EV_KEY, settings.input.key_up, kUp ? 1 : 0);
            uinput_send(ufd->mk, EV_KEY, settings.input.key_right, kRight ? 1 : 0);
            uinput_send(ufd->mk, EV_KEY, settings.input.key_down, kDown ? 1 : 0);
            uinput_send(ufd->mk, EV_KEY, settings.input.key_left, kLeft ? 1 : 0);
        }

        //axis
        if (lr3_axis) {
          bool rw_do;

          if (rw_timer%(settings.input.axis_speed*2) == 0)
            rw_do = true;
          else
            rw_do = false;

          uinput_send(ufd->mk, EV_REL, REL_X, lx/4/settings.input.axis_speed);
          uinput_send(ufd->mk, EV_REL, REL_Y, ly/4/settings.input.axis_speed);

          if (rw_do) {
            rx = rx/20;
            ry = ry/20;
            ry = -ry; //Inverted

            uinput_send(ufd->mk, EV_REL, REL_HWHEEL, rx);
            uinput_send(ufd->mk, EV_REL, REL_WHEEL, ry);
          }
        }

        last_ib0 = b0;
        last_ib1 = b1;
        last_lx = lx;
        last_ly = ly;
        last_rx = rx;
        last_ry = ry;
        last_kUp = kUp;
        last_kDown = kDown;
        last_kLeft = kLeft;
        last_kRight = kRight;

        uinput_send(ufd->mk, EV_SYN, SYN_REPORT, 0);

        if (rw_timer > 0xff)
          rw_timer = 0;
        else
          rw_timer += 1;

        if (nr != 19)
          usleep(10000);
    }

    uinput_close(ufd->mk, 0);

    std::cerr <<  "sixad-3in1::read(buf) - connection has been broken" << std::endl;
    
    delete ufd;

    return 0;
}
