/*
 * sixad-sixaxis.cpp
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

#include "shared.h"
#include "remote.h"
#include "uinput.h"

#include <cstdlib>
#include <iostream>
#include <poll.h>
#include <signal.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <sys/socket.h>
#include <unistd.h>

int csk = 0;
int isk = 1;
int debug;
int timeout = 30;

volatile bool active = false;

struct uinput_fd *ufd;

static int get_time()
{
  timespec tp;
  if (!clock_gettime(CLOCK_MONOTONIC, &tp)) {
    return tp.tv_sec/60;
  } else {
    return -1;
  }
}

static void process_remote(struct device_settings settings, const char *mac, int modes)
{
    int br;
    bool msg = true;
    unsigned char buf[128];

    int last_time_action = get_time();

    while (!io_canceled()) {
        br = read(isk, buf, sizeof(buf));
        if (msg) {
            syslog(LOG_INFO, "Connected 'PLAYSTATION(R)3 Remote (%s)'", mac);
            msg = false;
        }

        if (settings.timeout.enabled) {
            int current_time = get_time();
            if (was_active()) {
                last_time_action = current_time;
                set_active(false);
            } else if (current_time-last_time_action >= settings.timeout.timeout) {
                syslog(LOG_INFO, "Remote was not in use, and timeout reached, disconneting...");
                sig_term(0);
                break;
            }
        }

        if (br < 0) {
            break;
        } else if (br==13 && buf[0]==0xa1 && buf[1]==0x01) { //only continue if we've got a Remote
            if (settings.joystick.enabled) do_joystick(ufd->js, buf, settings.joystick);
            if (settings.remote.enabled) do_remote(ufd->mk, buf, modes);
            if (settings.input.enabled) do_input(ufd->mk, buf, settings.input);
        } else {
            if (debug) syslog(LOG_ERR, "Non-Remote packet received and ignored (0x%02x|0x%02x|0x%02x)", buf[0], buf[1], buf[2]);
        }
    }

    if (debug) syslog(LOG_ERR, "Read loop was broken on the Remote process");
}

int main(int argc, char *argv[])
{
    struct pollfd p[3];
    struct timespec timeout;
    struct device_settings settings;
    struct sigaction sa;
    sigset_t sigs;
    short events;

    if (argc < 3) {
        std::cout << "Running " << argv[0] << " requires 'sixad'. Please run sixad instead" << std::endl;
        return 1;
    }

    const char *mac = argv[1];
    debug = atoi(argv[2]);

    open_log("sixad-remote");
    settings = init_values(mac);
    settings.joystick.axis = false;
    settings.joystick.sbuttons = false;
    settings.joystick.accel = false;
    settings.joystick.speed = false;
    settings.joystick.pos = false;;
    settings.led.enabled = false;
    settings.rumble.enabled = false;

    ufd = uinput_open(DEV_TYPE_REMOTE, mac, settings);

    if (ufd->js < 0 || ufd->mk < 0) {
        return 1;
    } else if (ufd->js == 0 && ufd->mk == 0) {
        syslog(LOG_ERR, "remote config has no joystick or input mode selected - please choose one!");
        return 1;
    }

    int modes = 0;
    if (settings.remote.numeric) modes |= REMOTE_KEYMODE_NUMBERIC;
    if (settings.remote.dvd) modes |= REMOTE_KEYMODE_DVD;
    if (settings.remote.directional) modes |= REMOTE_KEYMODE_DIRECTIONAL;
    if (settings.remote.multimedia) modes |= REMOTE_KEYMODE_MULTIMEDIA;

    sigfillset(&sigs);
//    sigdelset(&sigs, SIGCHLD);
//    sigdelset(&sigs, SIGPIPE);
//    sigdelset(&sigs, SIGTERM);
//    sigdelset(&sigs, SIGINT);
//    sigdelset(&sigs, SIGHUP);

    memset(&sa, 0, sizeof(sa));
    sa.sa_flags = SA_NOCLDSTOP;

    sa.sa_handler = sig_term;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);

    sa.sa_handler = SIG_IGN;
    sigaction(SIGCHLD, &sa, NULL);
    sigaction(SIGPIPE, &sa, NULL);

    if (debug) syslog(LOG_INFO, "Press any to activate");

    p[0].fd = 0;
    p[0].events = POLLIN | POLLERR | POLLHUP;

    p[1].fd = 1;
    p[1].events = POLLIN | POLLERR | POLLHUP;

    p[2].fd = ufd->mk ? ufd->mk : ufd->js;
    p[2].events = POLLIN | POLLERR | POLLHUP;

    while (!io_canceled()) {
        int i, idx = 3;
        for (i = 0; i < idx; i++)
            p[i].revents = 0;

        timeout.tv_sec = 1;
        timeout.tv_nsec = 0;

        if (ppoll(p, idx, &timeout, &sigs) < 1)
            continue;

        if (p[1].revents & POLLIN) {
            process_remote(settings, mac, modes);
        }

        events = p[0].revents | p[1].revents | p[2].revents;

        if (events & (POLLERR | POLLHUP)) {
            break;
        }
    }

    if (debug) syslog(LOG_INFO, "Closing uinput...");

    if (settings.joystick.enabled) {
        uinput_close(ufd->js, debug);
    }
    if (settings.remote.enabled || settings.input.enabled) {
        uinput_close(ufd->mk, debug);
    }
    
    delete ufd;

    shutdown(isk, SHUT_RDWR);
    shutdown(csk, SHUT_RDWR);

    if (debug) syslog(LOG_INFO, "Done");

    return 0;
}
