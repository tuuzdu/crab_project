/*
 * hidraw-dump.c - program to dump information from hidraw devices (just like hcidump)
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

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>


int main(int argc, char **argv)
{
    unsigned char buf[128];
    int fd, nr, i, val, run;
    run = 0;

    if (argc < 2) {
        printf("Usage: %s /dev/hidrawX\n", argv[0]);
        return 1;
    }

    if ((fd = open(argv[1], O_RDONLY)) < 0) {
        perror("open(argv[1])");
        return 1;
    }

    while ( (nr=read(fd, buf, sizeof(buf))) ) {

        if ( nr < 0 ) {
            perror("read(stdin)");
            return 1;
        }

        for (i=0; i <= nr; i++) {
            val = buf[i];

            printf("%02X ", val);

            if (i == nr) {
                printf("\n \n");
            } else if (!i && !run) {
                run = 1;
            } else if (i % 15 == 0 && run) {
                printf("\n");
                run = 0;
            } else if ((i + 1) % 16 == 0) {
                printf("\n");
            }
        }
    }
    return 0;
}
