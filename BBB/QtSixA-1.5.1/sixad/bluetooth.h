/*
 * bluetooth.h
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

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>

#define L2CAP_PSM_HIDP_CTRL 0x11
#define L2CAP_PSM_HIDP_INTR 0x13

void do_search(int ctl, bdaddr_t *bdaddr, int debug);
void do_connect(int ctl, bdaddr_t *src, bdaddr_t *dst, int debug);

int l2cap_listen(const bdaddr_t *bdaddr, unsigned short psm, int lm, int backlog);
void l2cap_accept(int ctl, int csk, int isk, int debug, int legacy);
int l2cap_connect(bdaddr_t *src, bdaddr_t *dst, unsigned short psm);

void hid_server(int ctl, int csk, int isk, int debug, int legacy);
int create_device(int ctl, int csk, int isk);

int get_sdp_device_info(const bdaddr_t *src, const bdaddr_t *dst, struct hidp_connadd_req *req);
void epox_endian_quirk(unsigned char *data, int size);

#endif // BLUETOOTH_H
