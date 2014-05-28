/*
 * sixaxis.cpp
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

#include "sixaxis.h"
#include "uinput.h"
#include "shared.h"

#include <syslog.h>
#include <sys/socket.h>
#include <unistd.h>

double dt, rc_dd, alpha_dd, rc_d, alpha_d, rc, alpha;

struct state {
    double time;
    int ax, ay, az;       // Raw accelerometer data
    double ddx, ddy, ddz; // Acceleration
    double dx, dy, dz;    // Speed
    double x, y, z;       // Position
};

struct state prev;
struct state newH;
struct timeval tv;

int b1, b2, b3, lx, ly, rx, ry, acx, acy, acz, gyro, last_b1;
int up, right, down, left, l2, r2, l1, r1, tri, cir, cro, squ;
int posX, posY, posZ, accX, accY, accZ, velX, velY, velZ;
bool lr3_axis = true;
bool lr3_buttons = true;
int rw_timer = 0;

int last_jb1 = 0;
int last_jb2 = 0;
int last_jb3 = 0;
int last_ib1 = 0;
int last_ib2 = 0;
int last_ib3 = 0;

void do_joystick(int fd, unsigned char* buf, struct dev_joystick joystick)
{
    newH.time = tv.tv_sec + tv.tv_usec*1e-6;
    newH.ax = buf[42]<<8 | buf[43];
    newH.ay = buf[44]<<8 | buf[45];
    newH.az = buf[46]<<8 | buf[47];
    if ( ! prev.time ) {
        prev.time = newH.time;
        prev.ax = newH.ax;
        prev.ay = newH.ay;
        prev.az = newH.az;
    }
    dt = newH.time - prev.time; //(time constants were recuced by half)
    if (joystick.accon) {
        rc_dd = 1.0;  // Time constant for highpass filter on acceleration
        alpha_dd = rc_dd / (rc_dd+dt);
        newH.ddx = alpha_dd*(prev.ddx + (newH.ax-prev.ax)*0.01);
        newH.ddy = alpha_dd*(prev.ddy + (newH.ay-prev.ay)*0.01);
        newH.ddz = alpha_dd*(prev.ddz - (newH.az-prev.az)*0.01);
    }
    if (joystick.speed) {
        rc_d = 1.0;  // Time constant for highpass filter on speed
        alpha_d = rc_d / (rc_d+dt);
        newH.dx = alpha_d*(prev.dx + newH.ddx*dt);
        newH.dy = alpha_d*(prev.dy + newH.ddy*dt);
        newH.dz = alpha_d*(prev.dz + newH.ddz*dt);
    }
    if (joystick.pos) {
        rc = 0.5;  // Time constant for highpass filter on position
        alpha = rc / (rc+dt);
        newH.x = alpha*(prev.x + newH.dx*dt);
        newH.y = alpha*(prev.y + newH.dy*dt);
        newH.z = alpha*(prev.z + newH.dz*dt);
    }
    prev = newH;

    b1 = buf[3];
    b2 = buf[4];
    b3 = buf[5];
    lx = buf[7] - 128;
    ly = buf[8] - 128;
    rx = buf[9] - 128;
    ry = buf[10] - 128;
    acx = - (buf[42]<<8 | buf[43]); //reversed
    acy = buf[44]<<8 | buf[45];
    acz = buf[46]<<8 | buf[47];
    gyro = 0; // FIXME - What is the gyro suppose to do?
    up = buf[15];
    right = buf[16];
    down = buf[17];
    left = buf[18];
    l2 = buf[19];
    r2 = buf[20];
    l1 = buf[21];
    r1 = buf[22];
    tri = buf[23];
    cir = buf[24];
    cro = buf[25];
    squ = buf[26];
    posX = (int)(newH.x*1000);
    posY = (int)(newH.y*1000);
    posZ = (int)(newH.z*1000);
    accX = (int)(newH.ddx*1000);
    accY = (int)(newH.ddy*1000);
    accZ = (int)(newH.ddz*1000);
    velX = (int)(newH.dx*1000);
    velY = (int)(newH.dy*1000);
    velZ = (int)(newH.dz*1000);

    //deadzones
    if (lx > -10 && lx < 10) lx = 0;
    if (ly > -10 && ly < 10) ly = 0;
    if (rx > -11 && rx < 11) rx = 0;
    if (ry > -11 && ry < 11) ry = 0;
    if (acx < -508 && acx > -516) acx = -512; //acx is reversed
    if (acy > 508 && acy < 516) acy = 512;
    if (acz > 508 && acz < 516) acz = 512;
    if (posX > -30 && posX < 30) posX = 0;
    if (posY > -30 && posY < 30) posY = 0;
    if (posZ > -30 && posZ < 30) posZ = 0;
    if (accX > -30 && accX < 30) accX = 0;
    if (accY > -30 && accY < 30) accY = 0;
    if (accZ > -30 && accZ < 30) accZ = 0;
    if (velX > -30 && velX < 30) velX = 0;
    if (velY > -30 && velY < 30) velY = 0;
    if (velZ > -30 && velZ < 30) velZ = 0;

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

    //axis
    if (joystick.axis) {
        uinput_send(fd, EV_ABS, 0, lx);
        uinput_send(fd, EV_ABS, 1, ly);
        uinput_send(fd, EV_ABS, 2, rx);
        uinput_send(fd, EV_ABS, 3, ry);

        if (lx != 0 || ly != 0 || rx != 0 || ry != 0) {
          set_active(true);
        }
    }

    //accelerometer RAW
    if (joystick.accel) {
        uinput_send(fd, EV_ABS, 4, acx);
        uinput_send(fd, EV_ABS, 5, acy);
        uinput_send(fd, EV_ABS, 6, acz);
        uinput_send(fd, EV_ABS, 7, gyro);
    }

    //buttons (sensible, as axis)
    if (joystick.sbuttons) {
        uinput_send(fd, EV_ABS, 8, up);
        uinput_send(fd, EV_ABS, 9, right);
        uinput_send(fd, EV_ABS, 10, down);
        uinput_send(fd, EV_ABS, 11, left);
        uinput_send(fd, EV_ABS, 12, l2);
        uinput_send(fd, EV_ABS, 13, r2);
        uinput_send(fd, EV_ABS, 14, l1);
        uinput_send(fd, EV_ABS, 15, r1);
        uinput_send(fd, EV_ABS, 16+AXIS_PADDING, tri);
        uinput_send(fd, EV_ABS, 17+AXIS_PADDING, cir);
        uinput_send(fd, EV_ABS, 18+AXIS_PADDING, cro);
        uinput_send(fd, EV_ABS, 19+AXIS_PADDING, squ);

        if (up > 0 || right > 0 || down > 0 || left > 0 || l2 > 0 || r2 > 0 || l1 > 0 || r1 > 0 || tri > 0 || cir > 0 || cro > 0 || squ > 0 ) {
          set_active(true);
        }
    }

    //acceleration
    if (joystick.accon) {
        uinput_send(fd, EV_ABS, 20+AXIS_PADDING, accX);
        uinput_send(fd, EV_ABS, 21+AXIS_PADDING, accY);
        uinput_send(fd, EV_ABS, 22+AXIS_PADDING, accZ);
    }

    //speed
    if (joystick.speed) {
        uinput_send(fd, EV_ABS, 23+AXIS_PADDING, velX);
        uinput_send(fd, EV_ABS, 24+AXIS_PADDING, velY);
        uinput_send(fd, EV_ABS, 25+AXIS_PADDING, velZ);
    }

    //position
    if (joystick.pos) {
        uinput_send(fd, EV_ABS, 26+AXIS_PADDING, posX);
        uinput_send(fd, EV_ABS, 27+AXIS_PADDING, posY);
        uinput_send(fd, EV_ABS, 28+AXIS_PADDING, posZ);
    }

    last_jb1 = b1;
    last_jb2 = b2;
    last_jb3 = b3;

    uinput_send(fd, EV_SYN, SYN_REPORT, 0);
}

void do_input(int fd, unsigned char* buf, struct dev_input input)
{
    b1 = buf[3];
    b2 = buf[4];
    b3 = buf[5];
    lx = buf[7] - 128;
    ly = buf[8] - 128;
    rx = buf[9] - 128;
    ry = buf[10] - 128;

    //deadzones
    if (lx > -10 && lx < 10) lx = 0;
    if (ly > -10 && ly < 10) ly = 0;
    if (rx > -11 && rx < 11) rx = 0;
    if (ry > -11 && ry < 11) ry = 0;

    //lr3 enable/disable
    if ((b1 & SIXAXIS_KEY_L3) && b1 != last_b1)
      lr3_axis = !lr3_axis;

    if ((b1 & SIXAXIS_KEY_R3) && b1 != last_b1)
      lr3_buttons = !lr3_buttons;

    last_b1 = b1;

    //buttons
    if (!input.use_lr3 || (input.use_lr3 && lr3_buttons)) {
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
    }

    //axis
    if (!input.use_lr3 || (input.use_lr3 && lr3_axis)) {
      int rel;
      bool rw_do;

      if (rw_timer%(input.axis_speed*2) == 0)
        rw_do = true;
      else
        rw_do = false;

      if (input.axis_l_type == INPUT_TYPE_KEYS)
      {
          uinput_send(fd, EV_KEY, input.axis_l_right, (lx > 100));
          uinput_send(fd, EV_KEY, input.axis_l_left, (lx < -100));
          uinput_send(fd, EV_KEY, input.axis_l_up, (ly > 100));
          uinput_send(fd, EV_KEY, input.axis_l_down, (ly < -100));
      }
      else if (input.axis_l_type == INPUT_TYPE_MOUSE)
      {
          rel = input.axis_l_right;
          if (rel == REL_X || rel == REL_Y) {
            uinput_send(fd, EV_REL, rel, lx/4/input.axis_speed);
          } else if (rw_do && (rel == REL_WHEEL || rel == REL_HWHEEL)) {
            lx = lx/20;
            if (rel == REL_WHEEL) lx = -lx; //Inverted
            uinput_send(fd, EV_REL, rel, lx);
          }

          rel = input.axis_l_up;
          if (rel == REL_X || rel == REL_Y) {
            uinput_send(fd, EV_REL, rel, ly/4/input.axis_speed);
          } else if (rw_do && (rel == REL_WHEEL || rel == REL_HWHEEL)) {
            ly = ly/20;
            if (rel == REL_WHEEL) ly = -ly; //Inverted
            uinput_send(fd, EV_REL, rel, ly);
          }
      }

      if (input.axis_r_type == INPUT_TYPE_KEYS)
      {
          uinput_send(fd, EV_KEY, input.axis_r_right, (rx > 100));
          uinput_send(fd, EV_KEY, input.axis_r_left, (rx < -100));
          uinput_send(fd, EV_KEY, input.axis_r_up, (ry > 100));
          uinput_send(fd, EV_KEY, input.axis_r_down, (ry < -100));
      }
      else if (input.axis_r_type == INPUT_TYPE_MOUSE)
      {
          rel = input.axis_r_right;
          if (rel == REL_X || rel == REL_Y) {
            uinput_send(fd, EV_REL, rel, rx/4/input.axis_speed);
          } else if (rw_do && (rel == REL_WHEEL || rel == REL_HWHEEL)) {
            rx = rx/20;
            if (rel == REL_WHEEL) rx = -rx; //Inverted
            uinput_send(fd, EV_REL, rel, rx);
          }

          rel = input.axis_r_up;
          if (rel == REL_X || rel == REL_Y) {
            uinput_send(fd, EV_REL, rel, ry/4/input.axis_speed);
          } else if (rw_do && (rel == REL_WHEEL || rel == REL_HWHEEL)) {
            ry = ry/20;
            if (rel == REL_WHEEL) ry = -ry; //Inverted
            uinput_send(fd, EV_REL, rel, ry);
          }
      }
    }

    if (b1 > 0 || b2 > 0 || b3 > 0 || lx != 0 || ly != 0 || rx != 0 || ry != 0) {
      set_active(true);
    }

    last_ib1 = b1;
    last_ib2 = b2;
    last_ib3 = b3;

    uinput_send(fd, EV_SYN, SYN_REPORT, 0);

    if (rw_timer > 0xff)
      rw_timer = 0;
    else
      rw_timer += 1;

}

void do_rumble(int csk, int led_n, int weak, int strong, int timeout)
{
    unsigned char buf[128];
    unsigned char setrumble[] = {
        0x52, /* HIDP_TRANS_SET_REPORT | HIDP_DATA_RTYPE_OUTPUT */
        0x01,
        0x00, 0x00, 0x00, 0x00, 0x00,   // rumble values [0x00, right-timeout, right-force, left-timeout, left-force]
        0x00, 0x00, 0x00, 0x00, 0x1E,   // 0x02=LED1 .. 0x10=LED4
        0xff, 0x27, 0x10, 0x00, 0x32,   // LED 4
        0xff, 0x27, 0x10, 0x00, 0x32,   // LED 3
        0xff, 0x27, 0x10, 0x00, 0x32,   // LED 2
        0xff, 0x27, 0x10, 0x00, 0x32,   // LED 1
        0x00, 0x00, 0x00, 0x00, 0x00
    };
    const unsigned char ledpattern[11] = {
        0x00,
        0x02, 0x04, 0x08, 0x10, // 1, 2, 3, 4
        0x12, 0x14, 0x18, 0x1A, // 5, 6, 7, 8
        0x1C, 0x1E  // 9, 10
    };

    // TESTING
    weak *= 10;
    strong *= 10;
    timeout *= 10;

    if (weak > 0xff) weak = 0xff;
    else if (weak < 0) weak = 0;
    if (strong > 0xff) strong = 0xff;
    else if (strong < 0) strong = 0;
    if (timeout > 0xff) timeout = 0xff;
    else if (timeout < 4) timeout = 4;

    setrumble[3] = setrumble[5] = timeout;
    setrumble[4] = weak;
    setrumble[6] = strong;

    //syslog(LOG_INFO, "Rumble Callback (%i|%i|%i)", weak, strong, timeout);

    setrumble[11] = ledpattern[led_n]; //keep old led
    send(csk, setrumble, sizeof(setrumble), 0);
    recv(csk, buf, sizeof(buf), 0); //MSG_DONTWAIT?
}

int set_sixaxis_led(int csk, struct dev_led led, int rumble)
{
    int i, led_n, led_number;
    unsigned char buf[128];
    unsigned char setleds[] = {
        0x52, /* HIDP_TRANS_SET_REPORT | HIDP_DATA_RTYPE_OUTPUT */
        0x01,
        0x00, 0x00, 0x00, 0x00, 0x00,	// rumble values [0x00, right-timeout, right-force, left-timeout, left-force]
        0x00, 0x00, 0x00, 0x00, 0x1E,	// 0x02=LED1 .. 0x10=LED4
        0xff, 0x27, 0x10, 0x00, 0x32,	// LED 4
        0xff, 0x27, 0x10, 0x00, 0x32,	// LED 3
        0xff, 0x27, 0x10, 0x00, 0x32,	// LED 2
        0xff, 0x27, 0x10, 0x00, 0x32,	// LED 1
        0x00, 0x00, 0x00, 0x00, 0x00
    };
    const unsigned char ledpattern[11] = {
        0x00,
        0x02, 0x04, 0x08, 0x10, // 1, 2, 3, 4
        0x12, 0x14, 0x18, 0x1A, // 5, 6, 7, 8
        0x1C, 0x1E  // 9, 10
    };

    if (led.enabled) {
        if (led.auto_sel)
            led_number = get_joystick_number();
        else
            led_number = led.number;

        if (led_number < 1) {
            led_n = 1;
        } else if (led_number > 10) {
            led_n = 10;
        } else {
            led_n = led_number;
        }
    } else
        led_n = 0;

    if (led.enabled && led.anim)
    {
        /* Sixaxis LED animation - Way Cool!! */
        if (rumble) setleds[3] = setleds[5] = 0xfe;
        for (i=0; i<4; i++) {  // repeat it 4 times
            if (rumble) setleds[4] = setleds[6] = 0xff;
            setleds[11] = ledpattern[1];
            send(csk, setleds, sizeof(setleds), 0);
            recv(csk, buf, sizeof(buf), 0);
            usleep(10000);
            setleds[11] = ledpattern[2];
            send(csk, setleds, sizeof(setleds), 0);
            recv(csk, buf, sizeof(buf), 0);
            usleep(5000);
            setleds[11] = ledpattern[3];
            send(csk, setleds, sizeof(setleds), 0);
            recv(csk, buf, sizeof(buf), 0);
            usleep(5000);
            setleds[11] = ledpattern[4];
            send(csk, setleds, sizeof(setleds), 0);
            recv(csk, buf, sizeof(buf), 0);
            usleep(10000);
            setleds[11] = ledpattern[3];
            send(csk, setleds, sizeof(setleds), 0);
            recv(csk, buf, sizeof(buf), 0);
            usleep(5000);
            setleds[11] = ledpattern[2];
            send(csk, setleds, sizeof(setleds), 0);
            recv(csk, buf, sizeof(buf), 0);
            usleep(5000);
        }
        /* 2nd part of animation (animate until LED reaches selected number) */
        if (led_n == 2 || led_n == 6 || led_n == 9)
        {
            setleds[11] = ledpattern[1];
            send(csk, setleds, sizeof(setleds), 0);
            recv(csk, buf, sizeof(buf), 0);
        }
        else if (led_n == 3 || led_n == 7)
        {
            setleds[11] = ledpattern[1];
            send(csk, setleds, sizeof(setleds), 0);
            recv(csk, buf, sizeof(buf), 0);
            usleep(10000);
            setleds[11] = ledpattern[2];
            send(csk, setleds, sizeof(setleds), 0);
            recv(csk, buf, sizeof(buf), 0);
        }
        else if (led_n == 4 || led_n == 8)
        {
            setleds[11] = ledpattern[1];
            send(csk, setleds, sizeof(setleds), 0);
            recv(csk, buf, sizeof(buf), 0);
            usleep(100000);
            setleds[11] = ledpattern[2];
            send(csk, setleds, sizeof(setleds), 0);
            recv(csk, buf, sizeof(buf), 0);
            usleep(50000);
            setleds[11] = ledpattern[3];
            send(csk, setleds, sizeof(setleds), 0);
            recv(csk, buf, sizeof(buf), 0);
        }
    }

    /* set LEDs (final) */
    setleds[11] = ledpattern[led_n];
    if (rumble) setleds[3] = setleds[4] = setleds[5] = setleds[6] = 0x00;
    send(csk, setleds, sizeof(setleds), 0);
    recv(csk, buf, sizeof(buf), 0);

    return led_n;
}
