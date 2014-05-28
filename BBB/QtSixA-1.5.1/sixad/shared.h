/*
 * shared.h
 *
 * This file is part of the QtSixA, the Sixaxis Joystick Manager
 * Copyright 2008-2011 Filipe Coelho <falktx@gmail.com>
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

#ifndef SHARED_H
#define SHARED_H

struct dev_led {
    bool enabled;
    bool anim;
    bool auto_sel;
    int number;
};

struct dev_joystick {
    bool enabled;
    bool buttons;
    bool axis;
    bool sbuttons;
    bool accel;
    bool accon;
    bool speed;
    bool pos;
};

struct dev_remote {
    bool enabled;
    bool numeric;
    bool dvd;
    bool directional;
    bool multimedia;
};

struct dev_input {
    bool enabled;
    int key_select, key_l3, key_r3, key_start, key_up, key_right, key_down, key_left;
    int key_l2, key_r2, key_l1, key_r1, key_tri, key_cir, key_squ, key_cro, key_ps;
    int axis_l_type, axis_r_type, axis_speed;
    int axis_l_up, axis_l_right, axis_l_down, axis_l_left;
    int axis_r_up, axis_r_right, axis_r_down, axis_r_left;
    bool use_lr3;
};

struct dev_rumble {
    bool enabled;
    bool old_mode;
};

struct dev_timeout {
    bool enabled;
    int timeout;
};

struct device_settings {
    bool auto_disconnect;
    struct dev_led led;
    struct dev_joystick joystick;
    struct dev_remote remote;
    struct dev_input input;
    struct dev_rumble rumble;
    struct dev_timeout timeout;
};

bool was_active();
void set_active(int active);

bool io_canceled();
void sig_term(int sig);
void open_log(const char *app_name);

struct device_settings init_values(const char *mac);

int get_joystick_number();
void enable_sixaxis(int csk);

#endif // SHARED_H
