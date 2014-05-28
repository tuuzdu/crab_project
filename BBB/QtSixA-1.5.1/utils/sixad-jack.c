/*
 * sixad-jack.c - Read a Sixaxis jsX device and use it to control jack (midi and transport)
 *
 * This file is part of the QtSixA, the Sixaxis Joystick Manager
 * Copyright 2008-10 by falkTX.
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

#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <jack/jack.h>
#include <jack/midiport.h>
#include <jack/ringbuffer.h>
#include <linux/input.h>
#include <linux/joystick.h>

int debug = 0;

jack_client_t* client;
jack_port_t* output_port;
jack_nframes_t nframes;
jack_position_t position;
jack_transport_state_t state;
void* port_buffer;

int oct, black_keys;
int *axis;
int isPoly = 0;
int axis_prev_velocity[24];

unsigned char axis_is_checked[24];
unsigned char axis_prev_action[24];
unsigned char button_prev_val[24];

struct js_event js;

static int process_callback(jack_nframes_t nframes, void *arg)
{
    int i, ax, axis_note, axis_velocity;
    void* port_buffer = jack_port_get_buffer(output_port, nframes);
    unsigned char* buffer;
    unsigned char axis_playing[24];
    jack_midi_clear_buffer(port_buffer);

    for (i=0; i<nframes; i++)
    {
        for (ax=0; ax<20; ax++)
        {

            if (!isPoly)
            {
              buffer = jack_midi_event_reserve(port_buffer, i, 3);
              buffer[2] = 0x00; //as much poly as the host can get
              buffer[1] = 0x7F; //enable poly
              buffer[0] = 0xB0; //control mode
              isPoly = 1;
              if (debug) printf("Polyphonic mode enabled\n");
            }

            if (black_keys)
            {
              if (ax==0) axis_note = -2; //dummy
              else if (ax==1) axis_note = 0x02; //leftY (Modulation)
              else if (ax==2) axis_note = 0x0A; //rightH (Pan)
              else if (ax==3) axis_note = -2; //dummy
              else if (ax==4) axis_note = 0x10; //Acc X (Misc 1) 0x10
              else if (ax==5) axis_note = 0x11; //Acc Y (Misc 2) 0x11
              else if (ax==6) axis_note = 0x12; //Acc Z (Misc 3) 0x12
              else if (ax==7) axis_note = -2; //gyro (doesn't work)
              else if (ax==8) axis_note = 61+(12*oct);  //up
              else if (ax==9) axis_note = 62+(12*oct);  //right
              else if (ax==10) axis_note = 63+(12*oct); //down
              else if (ax==11) axis_note = 60+(12*oct); //left
              else if (ax==12) axis_note = 65+(12*oct); //l2
              else if (ax==13) axis_note = 66+(12*oct); //r2
              else if (ax==14) axis_note = 64+(12*oct); //l1
              else if (ax==15) axis_note = 67+(12*oct); //r1
              else if (ax==16) axis_note = 69+(12*oct); //triangle
              else if (ax==17) axis_note = 70+(12*oct); //circle
              else if (ax==18) axis_note = 71+(12*oct); //cross
              else if (ax==19) axis_note = 68+(12*oct); //square
            }
            else
            {
              if (ax==0) axis_note = -2; //dummy
              else if (ax==1) axis_note = 0x02; //leftY (Modulation)
              else if (ax==2) axis_note = 0x0A; //rightH (Pan)
              else if (ax==3) axis_note = -2; //dummy
              else if (ax==4) axis_note = 0x10; //Acc X (Misc 1) 0x10
              else if (ax==5) axis_note = 0x11; //Acc Y (Misc 2) 0x11
              else if (ax==6) axis_note = 0x12; //Acc Z (Misc 3) 0x12
              else if (ax==7) axis_note = -2; //gyro (doesn't work)
              else if (ax==8) axis_note = 62+(12*oct);  //up
              else if (ax==9) axis_note = 64+(12*oct);  //right
              else if (ax==10) axis_note = 65+(12*oct); //down
              else if (ax==11) axis_note = 60+(12*oct); //left
              else if (ax==12) axis_note = 69+(12*oct); //l2
              else if (ax==13) axis_note = 71+(12*oct); //r2
              else if (ax==14) axis_note = 67+(12*oct); //l1
              else if (ax==15) axis_note = 72+(12*oct); //r1
              else if (ax==16) axis_note = 76+(12*oct); //triangle
              else if (ax==17) axis_note = 77+(12*oct); //circle
              else if (ax==18) axis_note = 79+(12*oct); //cross
              else if (ax==19) axis_note = 74+(12*oct); //square
            }

            if (ax==1)
              axis_velocity = abs(axis[ax]/0xff) - 1; // modulation
            else if (ax==2)
              axis_velocity = ((axis[ax]/0xff)/2) + 63; // pan
            else if (ax<7)
              axis_velocity = ((axis[ax]/0xff)/2) + 63; // other controls
            else if (ax>=8 && ax<=19) { // keys
              axis_velocity = (axis[ax]/0xff) - 1;
              if (axis_velocity > 0) {
                axis_velocity = axis_velocity*(6.0-logf(axis_velocity));
                if (axis_velocity < 0)
                  axis_velocity = 0;
                else if (axis_velocity > 127)
                  axis_velocity = 127;
              }
            } else
              axis_velocity = (axis[ax]/0xff) - 1; // all the others

            if (axis_velocity == -1) axis_velocity = 0;

            if (axis[ax])
                axis_playing[ax] = 1;
            else
                axis_playing[ax] = 0;

            if (axis_prev_action[ax] != axis_playing[ax])
            {
                axis_is_checked[ax] = 0;
            }

            if (!axis_is_checked[ax])
            {

                if (axis_playing[ax])
                {
                    buffer = jack_midi_event_reserve(port_buffer, i, 3);
                    buffer[2] = axis_velocity;
                    buffer[1] = axis_note;
                    if (ax>7)
                      buffer[0] = 0x90; //note-on
                    else
                      buffer[0] = 0xB0; //control/mode
                    axis_prev_action[ax] = 1;
                    if (debug && (ax<4 || ax>7) && axis_note>0) printf("PLAY: axis %02i; velocity is %03i; playing ? %i\n", ax, axis_velocity, axis_playing[ax]);
                }
                else
                {
                    buffer = jack_midi_event_reserve(port_buffer, i, 3);
                    buffer[2] = axis_velocity;
                    buffer[1] = axis_note;
                    if (ax>7)
                      buffer[0] = 0x80;//note-off
                    else
                      buffer[0] = 0xB0; //control/mode
                    axis_prev_action[ax] = 0;
                    if (debug && (ax<4 || ax>7) && axis_note>0) printf("STOP: axis %02i; velocity is %03i; playing ? %i\n", ax, axis_velocity, axis_playing[ax]);
                }
                axis_is_checked[ax] = 1;
            }
            else if (axis_prev_velocity[ax] != axis[ax])
            {
                buffer = jack_midi_event_reserve(port_buffer, i, 3);
                buffer[2] = axis_velocity;
                buffer[1] = axis_note;
                if (ax>7)
                    buffer[0] = 0xA0; //aftertouch
                else
                    buffer[0] = 0xB0; //control/mode
                if (debug && (ax<4 || ax>7) && axis_note>0) printf("CTRL: axis %02i; velocity is %03i; playing ? %i\n", ax, axis_velocity, axis_playing[ax]);
            }

            axis_prev_velocity[ax] = axis[ax];
        }
    }

    return 0;
}

static int xrun_callback(void *arg)
{
    if (debug) printf("CALL: xrun callback\n");
    return 0;
}

static void do_jack(jack_client_t* client, char *button, int bk)
{
    black_keys = bk;

    // Jack Transport
    if (button[0] && (button[0] != button_prev_val[0])) //select :: Change octave
    {
        if (oct == 0) {
          oct = 1; //C6
          if (debug) printf("STAT: Changed base octave to C6\n");
        } else if (oct == 1) {
          oct = 2; //C7
          if (debug) printf("STAT: Changed base octave to C7\n");
        } else if (oct == 2) {
          oct = -2; //C3
          if (debug) printf("STAT: Changed base octave to C3\n");
        } else if (oct == -2) {
          oct = -1; //C4
          if (debug) printf("STAT: Changed base octave to C4\n");
        } else {
          oct = 0; //C5
          if (debug) printf("STAT: Changed base octave to C5\n");
        }
    }
    else if (button[3] && (button[3] != button_prev_val[3])) //start :: Start/Stop Transport
    {
        state = jack_transport_query(client, NULL);
        if (state) {
          jack_transport_stop(client);
          if (debug) printf("STAT: Transport stopped\n");
        } else {
          jack_transport_start(client);
          if (debug) printf("STAT: Transport started\n");
        }
    }

    if (button[16] && (button[16] != button_prev_val[16])) //PS :: Panic button
    {
        if (debug) printf("STAT: Panic!\n");
        jack_transport_stop(client);
        jack_transport_locate(client, 0);
        jack_transport_stop(client);
        int h;
        for (h=8; h<20; h++) {
            axis[h] = 0;
            axis_prev_velocity[h] = 0;
        }
    }
    else if (button[1]) //L3 :: Move transport backwards
    {
        jack_transport_query(client, &position);
        int prevJackFrame = position.frame - 100000;
        if (prevJackFrame < 0) prevJackFrame = 0;
        jack_transport_locate(client, prevJackFrame);
        if (button[1] != button_prev_val[1])
          if (debug) printf("STAT: Transport backwards...\n");
    }
    else if (button[1] != button_prev_val[1])
    {
          if (debug) printf("STAT: Transport normal\n");
    }
    else if (button[2]) //R3 :: Move transport forwards
    {
        jack_transport_query(client, &position);
        int nextJackFrame = position.frame + 100000;
        jack_transport_locate(client, nextJackFrame);
        if (button[2] != button_prev_val[2])
          if (debug) printf("STAT: Transport forwards...\n");
    }
    else if (button[2] != button_prev_val[2])
    {
          if (debug) printf("STAT: Transport normal\n");
    }

    button_prev_val[0] = button[0];
    button_prev_val[1] = button[1];
    button_prev_val[2] = button[2];
    button_prev_val[3] = button[3];
    button_prev_val[16] = button[16];

}

int main(int argc, char **argv)
{
    int fd, black_keys;
    char *button;
    unsigned char axes = 2;
    unsigned char buttons = 2;

    if (argc < 2) {
        printf("Usage: %s [-black] <jsX device>\n", argv[0]);
        return 1;
    }

    black_keys = 0;
    if (argc == 3) {
      if (argv[1][0] == '-' && argv[1][1] == 'b') {
          black_keys = 1;
          if (debug) printf("Using black keys\n");
        }
    }

    if (black_keys)
      fd = open(argv[2], O_RDONLY);
    else
      fd = open(argv[1], O_RDONLY);

    if (fd < 0) {
        perror("open(argv[x])");
        return 1;
    }

    client = jack_client_open("sixad-jack", JackNullOption, 0);

    if (client == NULL) {
        perror("jack_client_open()");
        return 1;
    }

    nframes = jack_get_buffer_size(client);
    output_port = jack_port_register(client, "midi_out", JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput, 0);
    jack_set_process_callback(client, process_callback, 0);
    jack_set_xrun_callback(client, xrun_callback, 0);

    if (jack_activate(client)) {
        perror("jack_activate()");
        return 1;
    }

    ioctl(fd, JSIOCGAXES, &axes);
    ioctl(fd, JSIOCGBUTTONS, &buttons);

    axis = calloc(axes, sizeof(int));
    button = calloc(buttons, sizeof(char));

    int h;
    for (h=0; h<20; h++) {
      axis_prev_action[h] = 0;
      axis_prev_velocity[h] = 0;
    }
    oct = 0;

    while (1) {

        if (read(fd, &js, sizeof(struct js_event)) != sizeof(struct js_event)) {
            perror("read(stdin)");
            jack_client_close(client);
            return 1;
        }

        switch (js.type & ~JS_EVENT_INIT) {
        case JS_EVENT_BUTTON:
            button[js.number] = js.value;
            break;
        case JS_EVENT_AXIS:
            axis[js.number] = js.value;
            break;
        }

        do_jack(client, button, black_keys);

    }

    jack_client_close(client);

    return 0;
}
