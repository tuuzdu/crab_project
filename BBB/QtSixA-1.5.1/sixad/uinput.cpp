/*
 * uinput.cpp
 *
 * This file is part of the QtSixA, the Sixaxis Joystick Manager
 * Copyright 2008-2010 Filipe Coelho <falktx@gmail.com>
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

#include "uinput.h"
#include "shared.h"

#include <cstdio>
#include <fcntl.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>

struct uinput_fd *uinput_open(int DEV_TYPE, const char *mac, struct device_settings settings)
{
    const char *uinput_filename[] = { "/dev/uinput", "/dev/input/uinput", "/dev/misc/uinput" };
    uinput_fd *ufd = new uinput_fd;
    int i;

    if (settings.joystick.enabled) {
        for (i=0; i<3; i++) {
            if ((ufd->js = open(uinput_filename[i], O_RDWR)) >= 0) {
                break;
            }
        }
        if (ufd->js < 0) {
            syslog(LOG_ERR, "uinput_open()::open(uinput_filename[i], O_RDWR) - failed to open uinput (js)");
            goto error;
        }
    } else {
        ufd->js = 0;
    }

    if (settings.remote.enabled || settings.input.enabled) {
        for (i=0; i<3; i++) {
            if ((ufd->mk = open(uinput_filename[i], O_RDWR)) >= 0) {
                break;
            }
        }
        if (ufd->mk < 0) {
            syslog(LOG_ERR, "uinput_open()::open(uinput_filename[i], O_RDWR) - failed to open uinput (mk)");
            goto error;
        }
    } else {
        ufd->mk = 0;
    }

    char dev_name[512];
    struct uinput_user_dev dev, dev_mk;

    memset(&dev, 0, sizeof(dev));
    memset(&dev_mk, 0, sizeof(dev_mk));

    if (DEV_TYPE == DEV_TYPE_SIXAXIS) {
        strcpy(dev_name, "PLAYSTATION(R)3 Controller (");
        strcat(dev_name, mac);
        strcat(dev_name, ")");
        snprintf(dev.name, sizeof(dev.name), "%s", dev_name);
        dev.id.vendor = 0x054c;
        dev.id.product = 0x0268;
        dev.id.version = 0x0100;
        dev.id.bustype = BUS_VIRTUAL;
    } else if (DEV_TYPE == DEV_TYPE_REMOTE) {
        strcpy(dev_name, "PLAYSTATION(R)3 Remote (");
        strcat(dev_name, mac);
        strcat(dev_name, ")");
        snprintf(dev.name, sizeof(dev.name), "%s", dev_name);
        dev.id.vendor = 0x054c;
        dev.id.product = 0x0306;
        dev.id.version = 0x0100;
        dev.id.bustype = BUS_VIRTUAL;
    } else if (DEV_TYPE == DEV_TYPE_3IN1) {
        strcpy(dev_name, "Brooklyn 3in1 KeyMote");
        snprintf(dev.name, sizeof(dev.name), "%s", dev_name);
        dev.id.vendor = 0x062a;
        dev.id.product = 0x0818;
        dev.id.version = 0x0100;
        dev.id.bustype = BUS_VIRTUAL;
    } else {
        strcpy(dev_name, "Unkown Device (");
        strcat(dev_name, mac);
        strcat(dev_name, ")");
        snprintf(dev.name, sizeof(dev.name), "%s", dev_name);
        dev.id.vendor = 0x000;
        dev.id.product = 0x000;
        dev.id.version = 0x0000;
        dev.id.bustype = BUS_VIRTUAL;
    }

    dev_mk = dev;
    strcat(dev_name, " MK");
    snprintf(dev_mk.name, sizeof(dev_mk.name), "%s", dev_name);

    if (settings.joystick.enabled) {
        for (i=0; i<17; i++) {
            if (ioctl(ufd->js, UI_SET_KEYBIT, BTN_JOYSTICK + i) < 0) {
                syslog(LOG_ERR, "uinput_open()::ioctl(BTN_JOYSTICK) - failed to register button %i", i);
                goto error;
            }
        }

        // enable all axis and accelerometers
        int pos;
        for (i=0; i<29; i++) {
            pos = (i >= 16) ? i+AXIS_PADDING : i;
            if (i >= 0 && i <= 3) {// left & right axis
                dev.absmax[pos] = 127;
                dev.absmin[pos] = -127;
            } else if (i == 4) {  // Accelerometer X (reversed)
                dev.absmax[pos] = -402;
                dev.absmin[pos] = -622;
            } else if (i == 5) {  // Accelerometer Y
                dev.absmax[pos] = 622;
                dev.absmin[pos] = 402;
            } else if (i == 6) {  // Accelerometer Z
                dev.absmax[pos] = 622;
                dev.absmin[pos] = 402;
            } else if (i == 7) {  // Gyro - Does NOT work
                dev.absmax[pos] = 127;
                dev.absmin[pos] = -127;
            } else if (i >= 8 && i <= 19) {  // Buttons
                dev.absmax[pos] = 255;
                dev.absmin[pos] = -255;
            } else if (i >= 20 && i <= 22) { // Acceleration
                dev.absmax[pos] = 1250;
                dev.absmin[pos] = -1250;
            } else if (i >= 23 && i <= 25) { // Speed
                dev.absmax[pos] = 1250;
                dev.absmin[pos] = -1250;
            } else if (i >= 26 && i <= 28) { // Position
                dev.absmax[pos] = 1250;
                dev.absmin[pos] = -1250;
            } else {
                dev.absmax[pos] = 32767;
                dev.absmin[pos] = -32767;
            }

            if (ioctl(ufd->js, UI_SET_ABSBIT, pos) < 0) {
                syslog(LOG_ERR, "uinput_open()::ioctl(ABS_AXIS) - failed to register axis %i", pos);
                goto error;
            }
        }

      if (ioctl(ufd->js, UI_SET_EVBIT, EV_ABS) < 0) {
          syslog(LOG_ERR, "uinput_open()::ioctl(EV_ABS) - failed to set attribute");
          goto error;
      }
    }

    if (settings.input.enabled) {
        // enable all keys
        for (i=KEY_RESERVED+1; i<KEY_WIMAX; i++) {
            if (ioctl(ufd->mk, UI_SET_KEYBIT, i) < 0) {
                syslog(LOG_ERR, "uinput_open()::ioctl(BTN_KEYBOARD) - failed to register key %i", i);
                goto error;
            }
        }

        // enable all mouse clicks
        int MouseClicks[] = { BTN_MOUSE, BTN_LEFT, BTN_RIGHT, BTN_MIDDLE, BTN_SIDE, BTN_EXTRA, BTN_FORWARD, BTN_BACK, BTN_TASK };
        for (i=0; i<9; i++) {
            if (ioctl(ufd->mk, UI_SET_KEYBIT, MouseClicks[i]) < 0) {
                syslog(LOG_ERR, "uinput_open()::ioctl(BTN_MOUSE) - failed to register key %i", MouseClicks[i]);
                goto error;
            }
        }

        if (settings.input.axis_l_type || settings.input.axis_r_type) {
          // enable all mouse axis
          int MouseAxis[] = { REL_X, REL_Y, REL_WHEEL, REL_HWHEEL };
          for (i=0; i<4; i++) {
              if (ioctl(ufd->mk, UI_SET_RELBIT, MouseAxis[i]) < 0) {
                  syslog(LOG_ERR, "uinput_open()::ioctl(ABS_MOUSE) - failed to register axis %i", i);
                  goto error;
              }
           }

          // enable mouse axis
          if (ioctl(ufd->mk, UI_SET_EVBIT, EV_REL) < 0) {
              syslog(LOG_ERR, "uinput_open()::ioctl(EV_REL) - failed to set attribute");
              goto error;
          }
        }
    }

    if (DEV_TYPE == DEV_TYPE_SIXAXIS && settings.joystick.enabled && settings.rumble.enabled) {
        if (ioctl(ufd->js, UI_SET_FFBIT, FF_RUMBLE) < 0) {
            syslog(LOG_ERR, "uinput_open()::ioctl(FF_RUMBLE) - failed to set attribute");
            goto error;
        }
        if (ioctl(ufd->js, UI_SET_FFBIT, FF_PERIODIC) < 0) {
            syslog(LOG_ERR, "uinput_open()::ioctl(FF_PERIODIC) - failed to set attribute");
            goto error;
        }
        if (ioctl(ufd->js, UI_SET_FFBIT, FF_SQUARE) < 0) {
            syslog(LOG_ERR, "uinput_open()::ioctl(FF_SQUARE) - failed to set attribute");
            goto error;
        }
        if (ioctl(ufd->js, UI_SET_FFBIT, FF_TRIANGLE) < 0) {
            syslog(LOG_ERR, "uinput_open()::ioctl(FF_TRIANGLE) - failed to set attribute");
            goto error;
        }
        if (ioctl(ufd->js, UI_SET_FFBIT, FF_SINE) < 0) {
            syslog(LOG_ERR, "uinput_open()::ioctl(FF_SINE) - failed to set attribute");
            goto error;
        }
        dev.ff_effects_max = MAX_RUMBLE_EFFECTS;

        if (ioctl(ufd->js, UI_SET_EVBIT, EV_FF) < 0) {
            syslog(LOG_ERR, "uinput_open()::ioctl(EV_FF) - failed to set attribute");
            goto error;
        }
    }

    // Activate buttons and register device
    if (settings.joystick.enabled) {
        if (ioctl(ufd->js, UI_SET_EVBIT, EV_KEY) < 0) {
            syslog(LOG_ERR, "uinput_open()::ioctl(EV_KEY) - failed to set attribute (js)");
            goto error;
        }

        if (write(ufd->js, &dev, sizeof(dev)) != sizeof(dev)) {
            syslog(LOG_ERR, "uinput_open()::write(dev) - failed to set device information (js)");
            goto error;
        }

        if (ioctl(ufd->js, UI_DEV_CREATE) < 0) {
            syslog(LOG_ERR, "uinput_open()::ioctl(UI_DEV_CREATE) - failed to create device (js)");
            goto error;
        }
    }

    if (settings.remote.enabled || settings.input.enabled) {
        if (ioctl(ufd->mk, UI_SET_EVBIT, EV_KEY) < 0) {
            syslog(LOG_ERR, "uinput_open()::ioctl(EV_KEY) - failed to set attribute (mk)");
            goto error;
        }

        if (write(ufd->mk, &dev_mk, sizeof(dev_mk)) != sizeof(dev_mk)) {
            syslog(LOG_ERR, "uinput_open()::write(dev) - failed to set device information (mk)");
            goto error;
        }

        if (ioctl(ufd->mk, UI_DEV_CREATE) < 0) {
            syslog(LOG_ERR, "uinput_open()::ioctl(UI_DEV_CREATE) - failed to create device (mk)");
            goto error;
        }
    }

    return ufd;

error:
    ufd->js = -1;
    ufd->mk = -1;
    return ufd;
}

int uinput_close(int fd, int debug)
{
    if (ioctl(fd, UI_DEV_DESTROY) < 0) {
        if (debug) syslog(LOG_ERR, "uinput_close()::ioctl(UI_DEV_DESTROY) - failed to destroy device");
        return -1;
    } else {
        if (debug) syslog(LOG_INFO, "uinput_close()::ioctl(UI_DEV_DESTROY) - success!");
    }

    if (close(fd)) {
        if (debug) syslog(LOG_ERR, "uinput_close()::close(fd) - failed to close uinput");
        return -1;
    } else {
        if (debug) syslog(LOG_INFO, "uinput_close()::close(fd) - success!");
    }

    return 0;
}

int uinput_send(int fd, unsigned short type, unsigned short code, int value)
{
    struct input_event event;
    memset(&event, 0, sizeof(event));
    event.type = type;
    event.code = code;
    event.value = value;
    gettimeofday(&(event.time), NULL);

    if (write(fd, &event, sizeof(event)) != sizeof(event)) {
        syslog(LOG_ERR, "uinput_send::write(event) - failed to send uinput event (type %i, code %i, value %i)", type, code, value);
        return -1;
    }

    return 0;
}
