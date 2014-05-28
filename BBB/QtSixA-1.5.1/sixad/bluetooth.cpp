/*
 * bluetooth.cpp
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

#include "bluetooth.h"
#include "shared.h"

#include <cstdlib>
#include <cerrno>
#include <iostream>
#include <poll.h>
#include <signal.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <bluetooth/hidp.h>
#include <bluetooth/sdp.h>
#include <bluetooth/sdp_lib.h>
#include <bluetooth/hci_lib.h>

void do_search(int ctl, bdaddr_t *bdaddr, int debug)
{
        inquiry_info *info = NULL;
        bdaddr_t src, dst;
        int i, dev_id, num_rsp, length, flags;
        char addr[18];
        uint8_t _class[3];

        ba2str(bdaddr, addr);
        dev_id = hci_devid(addr);
        if (dev_id < 0) {
                dev_id = hci_get_route(NULL);
                hci_devba(dev_id, &src);
        } else
                bacpy(&src, bdaddr);

        length  = 8;    /* ~10 seconds */
        num_rsp = 0;
        flags   = IREQ_CACHE_FLUSH;

        if (debug) syslog(LOG_INFO, "Searching...");

        num_rsp = hci_inquiry(dev_id, length, num_rsp, NULL, &info, flags);

        for (i = 0; i < num_rsp; i++) {
                memcpy(_class, (info+i)->dev_class, 3);
                
                if (debug) syslog(LOG_INFO, "Got device %02X | %02X | %02X", _class[0], _class[1], _class[2]);

                if (_class[1] == 0x25 && (_class[2] == 0x00 || _class[2] == 0x01)) {
                        bacpy(&dst, &(info+i)->bdaddr);
                        ba2str(&dst, addr);

                        if (debug) syslog(LOG_INFO, "Connecting to device %s", addr);
                        do_connect(ctl, &src, &dst, debug);
                }
        }

        bt_free(info);

        if (num_rsp <= 0) {
                if (debug) syslog(LOG_ERR, "No devices in range or visible");
        }
}

void do_connect(int ctl, bdaddr_t *src, bdaddr_t *dst, int debug)
{
        struct hidp_connadd_req req;
        uint16_t uuid = HID_SVCLASS_ID;
        int csk, isk, err;

        memset(&req, 0, sizeof(req));

        err = get_sdp_device_info(src, dst, &req);

        if (err < 0) {
                syslog(LOG_ERR, "Can't get device information");
                return;
        }

        if (uuid == HID_SVCLASS_ID && req.vendor == 0x054c && req.product == 0x0306) {
            csk = l2cap_connect(src, dst, L2CAP_PSM_HIDP_CTRL);
            if (csk < 0) {
                    syslog(LOG_ERR, "Can't create HID control channel");
                    return;
            }

            isk = l2cap_connect(src, dst, L2CAP_PSM_HIDP_INTR);
            if (isk < 0) {
                    syslog(LOG_ERR, "Can't create HID interrupt channel");
                    close(csk);
                    return;
            }

            if (debug) syslog(LOG_INFO, "Will initiate Remote now");

            dup2(isk, 1);
            close(isk);
            dup2(csk, 0);
            close(csk);

            char bda[18];
            ba2str(dst, bda);

            char cmd[64];
            strcpy(cmd, "/usr/sbin/sixad-remote ");
            strcat(cmd, bda);
            strcat(cmd, " ");
            strcat(cmd, debug ? "1" : "0");

            if (!system(cmd)) {
                syslog(LOG_INFO, "cannot exec '%s'", cmd);
            }

        } else {
            syslog(LOG_ERR, "device ID failed -> %04i, 0x%03X:0x%03X", uuid, req.vendor, req.product);
        }
}

int l2cap_listen(const bdaddr_t *bdaddr, unsigned short psm, int lm, int backlog)
{
    struct sockaddr_l2 addr;
    struct l2cap_options opts;
    int sk;

    if ((sk = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP)) < 0)
        return -1;

    memset(&addr, 0, sizeof(addr));
    addr.l2_family = AF_BLUETOOTH;
    bacpy(&addr.l2_bdaddr, bdaddr);
    addr.l2_psm = htobs(psm);

    if (bind(sk, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        close(sk);
        return -1;
    }

    setsockopt(sk, SOL_L2CAP, L2CAP_LM, &lm, sizeof(lm));

    memset(&opts, 0, sizeof(opts));
    opts.imtu = 64;
    opts.omtu = HIDP_DEFAULT_MTU;
    opts.flush_to = 0xffff;

    setsockopt(sk, SOL_L2CAP, L2CAP_OPTIONS, &opts, sizeof(opts));

    if (listen(sk, backlog) < 0) {
        close(sk);
        return -1;
    }

    return sk;
}

void l2cap_accept(int ctl, int csk, int isk, int debug, int legacy)
{
    struct sockaddr_l2 addr;
    struct hidp_connadd_req req;
    socklen_t addrlen;
    bdaddr_t addr_src, addr_dst;
    int ctrl_socket, intr_socket, err;

    memset(&addr, 0, sizeof(addr));
    memset(&req, 0, sizeof(req));
    addrlen = sizeof(addr);

    if ((ctrl_socket = accept(csk, (struct sockaddr *)&addr, &addrlen)) < 0) {
        syslog(LOG_ERR, "unable to accept control stream");
        return;
    }
    bacpy(&addr_dst, &addr.l2_bdaddr);

    if (getsockname(ctrl_socket, (struct sockaddr *)&addr, &addrlen) < 0) {
        syslog(LOG_ERR, "unable to get socket name from control stream");
        return;
    }
    bacpy(&addr_src, &addr.l2_bdaddr);

    if ((intr_socket = accept(isk, (struct sockaddr *)&addr, &addrlen)) < 0) {
        syslog(LOG_ERR, "unable to accept info stream");
        close(ctrl_socket);
        return;
    }

    if (bacmp(&addr_dst, &addr.l2_bdaddr)) {
        syslog(LOG_ERR, "intr and ctrl streams from different hosts - rejecting both");
        close(ctrl_socket);
        close(intr_socket);
        return;
    }

#ifdef GASIA_GAMEPAD_HACKS
    req.vendor  = 0x054c;
    req.product = 0x0268;
    req.version = 0x0100;
    req.parser  = 0x0100;

    strcpy(req.name, "Gasia Gamepad experimental driver");
#else
    get_sdp_device_info(&addr_src, &addr_dst, &req);
#endif

    if (!legacy && req.vendor == 0x054c && req.product == 0x0268) {
        if (debug) syslog(LOG_INFO, "Will initiate Sixaxis now");

        // New proccess for sixad-sixaxis
        pid_t pid = fork();

        if (pid == 0) {
//             close(ctl);
//             close(csk);
//             close(isk);

            dup2(ctrl_socket, 0);
            close(ctrl_socket);
            dup2(intr_socket, 1);
            close(intr_socket);

            char bda[18];
            ba2str(&addr_dst, bda);

            const char* uinput_sixaxis_cmd = "/usr/sbin/sixad-sixaxis";
            const char* debug_mode = debug ? "1" : "0";

            const char* argv[] = { uinput_sixaxis_cmd, bda, debug_mode, NULL };
            char* envp[] = { NULL };

            if (execve(argv[0], (char* const*)argv, envp) < 0) {
                syslog(LOG_INFO, "cannot exec %s", uinput_sixaxis_cmd);
                close(1);
                close(0);
            }
        }

    } else {
        if (debug) syslog(LOG_INFO, "Creating new device using the default driver...");
        err = create_device(ctl, ctrl_socket, intr_socket);
        if (err < 0)
            syslog(LOG_ERR, "HID create error %d (%s)", errno, strerror(errno));
        close(intr_socket);
        close(ctrl_socket);
    }
    return;
}

int l2cap_connect(bdaddr_t *src, bdaddr_t *dst, unsigned short psm)
{
        struct sockaddr_l2 addr;
        struct l2cap_options opts;
        int sk;

        if ((sk = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP)) < 0)
                return -1;

        memset(&addr, 0, sizeof(addr));
        addr.l2_family  = AF_BLUETOOTH;
        bacpy(&addr.l2_bdaddr, src);

        if (bind(sk, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
                close(sk);
                return -1;
        }

        memset(&opts, 0, sizeof(opts));
        opts.imtu = HIDP_DEFAULT_MTU;
        opts.omtu = HIDP_DEFAULT_MTU;
        opts.flush_to = 0xffff;

        setsockopt(sk, SOL_L2CAP, L2CAP_OPTIONS, &opts, sizeof(opts));

        memset(&addr, 0, sizeof(addr));
        addr.l2_family  = AF_BLUETOOTH;
        bacpy(&addr.l2_bdaddr, dst);
        addr.l2_psm = htobs(psm);

        if (connect(sk, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
                close(sk);
                return -1;
        }

        return sk;
}

void hid_server(int ctl, int csk, int isk, int debug, int legacy)
{
    struct pollfd p[2];
    struct timespec timeout;
    short events;
    sigset_t sigs;

    sigfillset(&sigs);
    sigdelset(&sigs, SIGCHLD);
    sigdelset(&sigs, SIGPIPE);
    sigdelset(&sigs, SIGTERM);
    sigdelset(&sigs, SIGINT);
    sigdelset(&sigs, SIGHUP);

    if (debug) syslog(LOG_INFO, "Server mode active, will start search now");

    p[0].fd = csk;
    p[0].events = POLLIN | POLLERR | POLLHUP;

    p[1].fd = isk;
    p[1].events = POLLIN | POLLERR | POLLHUP;

    while (!io_canceled()) {
        int i, idx = 2;

        for (i=0; i<idx; i++)
            p[i].revents = 0;

        timeout.tv_sec = 1;
        timeout.tv_nsec = 0;

        if (ppoll(p, idx, &timeout, &sigs) < 1)
            continue;

        events = p[0].revents | p[1].revents;

        if (events & POLLIN) {
            if (debug) syslog(LOG_INFO, "One event received");
            l2cap_accept(ctl, csk, isk, debug, legacy);
            if (debug) syslog(LOG_INFO, "One event proccessed");
        }

        if (events & (POLLERR | POLLHUP)) {
            if (debug) syslog(LOG_ERR, "Server mode loop was broken");
            break;
        }

    }
}

int create_device(int ctl, int csk, int isk)
{
     struct hidp_connadd_req req;
     struct sockaddr_l2 addr;
     socklen_t addrlen;
     bdaddr_t src, dst;
     char bda[18];
     int err;

     memset(&addr, 0, sizeof(addr));
     addrlen = sizeof(addr);

     if (getsockname(csk, (struct sockaddr *) &addr, &addrlen) < 0)
         return -1;

     bacpy(&src, &addr.l2_bdaddr);

     memset(&addr, 0, sizeof(addr));
     addrlen = sizeof(addr);

     if (getpeername(csk, (struct sockaddr *) &addr, &addrlen) < 0)
         return -1;

     bacpy(&dst, &addr.l2_bdaddr);

     memset(&req, 0, sizeof(req));
     req.ctrl_sock = csk;
     req.intr_sock = isk;
     req.flags     = 0;
     req.idle_to   = 1800;


#ifdef GASIA_GAMEPAD_HACKS
    req.vendor  = 0x054c;
    req.product = 0x0268;
    req.version = 0x0100;
    req.parser  = 0x0100;

    strcpy(req.name, "Gasia Gamepad experimental driver");

    err = 0;
#else
    err = get_sdp_device_info(&src, &dst, &req);
#endif

     if (err < 0)
         return err;
     else {
         ba2str(&dst, bda);
         syslog(LOG_INFO, "Connected %s (%s)", req.name, bda);
         if (req.vendor == 0x054c && req.product == 0x0268)
             enable_sixaxis(csk);
         err = ioctl(ctl, HIDPCONNADD, &req);
     }

  return 0;
}

int get_sdp_device_info(const bdaddr_t *src, const bdaddr_t *dst, struct hidp_connadd_req *req)
{
    struct sockaddr_l2 addr;
    socklen_t addrlen;
    bdaddr_t bdaddr;
    sdp_data_t *pdlist, *pdlist2;
    sdp_list_t *search, *attrid, *pnp_rsp, *hid_rsp;
    sdp_record_t *rec;
    sdp_session_t *sdp_session;
    uuid_t svclass;
    uint32_t range = 0x0000ffff;
    int err;

    sdp_session = sdp_connect(src, dst, SDP_RETRY_IF_BUSY | SDP_WAIT_ON_CLOSE);
    if (!sdp_session) {
        syslog(LOG_ERR, "unable to connect to sdp session");
        return -1;
    }

    sdp_uuid16_create(&svclass, PNP_INFO_SVCLASS_ID);
    search = sdp_list_append(NULL, &svclass);
    attrid = sdp_list_append(NULL, &range);

    err = sdp_service_search_attr_req(sdp_session, search, SDP_ATTR_REQ_RANGE, attrid, &pnp_rsp);

    sdp_list_free(search, NULL);
    sdp_list_free(attrid, NULL);

    sdp_uuid16_create(&svclass, HID_SVCLASS_ID);
    search = sdp_list_append(NULL, &svclass);
    attrid = sdp_list_append(NULL, &range);

    err = sdp_service_search_attr_req(sdp_session, search, SDP_ATTR_REQ_RANGE, attrid, &hid_rsp);

    sdp_list_free(search, NULL);
    sdp_list_free(attrid, NULL);

    memset(&addr, 0, sizeof(addr));
    addrlen = sizeof(addr);

    if (getsockname(sdp_session->sock, (struct sockaddr *) &addr, &addrlen) < 0)
            bacpy(&bdaddr, src);
    else
            bacpy(&bdaddr, &addr.l2_bdaddr);

    sdp_close(sdp_session);

    if (err || !hid_rsp) {
        syslog(LOG_ERR, "unable to get device information");
        return -1;
    }

    if (pnp_rsp) {
        rec = (sdp_record_t *) pnp_rsp->data;

        pdlist = sdp_data_get(rec, 0x0201);
        req->vendor = pdlist ? pdlist->val.uint16 : 0x0000;

        pdlist = sdp_data_get(rec, 0x0202);
        req->product = pdlist ? pdlist->val.uint16 : 0x0000;

        pdlist = sdp_data_get(rec, 0x0203);
        req->version = pdlist ? pdlist->val.uint16 : 0x0000;

        sdp_record_free(rec);
    }

    rec = (sdp_record_t *) hid_rsp->data;

    pdlist = sdp_data_get(rec, 0x0101);
    pdlist2 = sdp_data_get(rec, 0x0102);
    if (pdlist) {
            if (pdlist2) {
                    if (strncmp(pdlist->val.str, pdlist2->val.str, 5)) {
                            strncpy(req->name, pdlist2->val.str, sizeof(req->name) - 1);
                            strcat(req->name, " ");
                    }
                    strncat(req->name, pdlist->val.str,
                                    sizeof(req->name) - strlen(req->name));
            } else
                    strncpy(req->name, pdlist->val.str, sizeof(req->name) - 1);
    } else {
            pdlist2 = sdp_data_get(rec, 0x0100);
            if (pdlist2)
                    strncpy(req->name, pdlist2->val.str, sizeof(req->name) - 1);
    }

    pdlist = sdp_data_get(rec, 0x0201);
    req->parser = pdlist ? pdlist->val.uint16 : 0x0100;

    pdlist = sdp_data_get(rec, 0x0202);
    req->subclass = pdlist ? pdlist->val.uint8 : 0;

    pdlist = sdp_data_get(rec, 0x0203);
    req->country = pdlist ? pdlist->val.uint8 : 0;

    pdlist = sdp_data_get(rec, 0x0206);
    if (pdlist) {
            pdlist = pdlist->val.dataseq;
            pdlist = pdlist->val.dataseq;
            pdlist = pdlist->next;

            req->rd_data = (uint8_t*)malloc(pdlist->unitSize);
            if (req->rd_data) {
                    memcpy(req->rd_data, (unsigned char *) pdlist->val.str, pdlist->unitSize);
                    req->rd_size = pdlist->unitSize;
                    epox_endian_quirk(req->rd_data, req->rd_size);
            }
    }

    sdp_record_free(rec);

    return 0;
}

void epox_endian_quirk(unsigned char *data, int size)
{
        /* USAGE_PAGE (Keyboard)	05 07
         * USAGE_MINIMUM (0)		19 00
         * USAGE_MAXIMUM (65280)	2A 00 FF   <= must be FF 00
         * LOGICAL_MINIMUM (0)		15 00
         * LOGICAL_MAXIMUM (65280)	26 00 FF   <= must be FF 00
         */
        unsigned char pattern[] = { 0x05, 0x07, 0x19, 0x00, 0x2a, 0x00, 0xff,
                                                0x15, 0x00, 0x26, 0x00, 0xff };
        unsigned int i;

        if (!data)
                return;

        for (i = 0; i < size - sizeof(pattern); i++) {
                if (!memcmp(data + i, pattern, sizeof(pattern))) {
                        data[i + 5] = 0xff;
                        data[i + 6] = 0x00;
                        data[i + 10] = 0xff;
                        data[i + 11] = 0x00;
                }
        }
}
