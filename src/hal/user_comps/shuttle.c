
//
// This is a userspace HAL driver for the ShuttleXpress and ShuttlePRO
// devices by Contour Design.
//
// Copyright 2011, 2016, 2021 Sebastian Kuzminsky <seb@highlab.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301-1307 USA
//


#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <linux/types.h>
#include <linux/hidraw.h>

#include "hal.h"



#ifndef HIDIOCGRAWNAME
#define HIDIOCGRAWNAME(len)     _IOC(_IOC_READ, 'H', 0x04, len)
#endif

#define Max(a, b)  ((a) > (b) ? (a) : (b))

#define MAX_BUTTONS 15


typedef struct {
    const char *name;
    uint16_t vendor_id;
    uint16_t product_id;
    int num_buttons;
    uint16_t button_mask[MAX_BUTTONS];
} contour_dev_t;



contour_dev_t contour_dev[] = {
    {
        .name = "shuttlexpress",
        .vendor_id = 0x0b33,
        .product_id = 0x0020,
        .num_buttons = 5,
        .button_mask = { 0x0010, 0x0020, 0x0040, 0x0080, 0x0100 }
    },
    {
        .name = "shuttlepro",
        .vendor_id = 0x05f3,
        .product_id = 0x0240,
        .num_buttons = 13,
        .button_mask = { 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080, 0x0100, 0x0200, 0x0400, 0x0800, 0x1000 }
    },
    {
        .name = "shuttleproV2",
        .vendor_id = 0x0b33,
        .product_id = 0x0030,
        .num_buttons = 15,
        .button_mask = { 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080, 0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000 }
    }
};

// each packet from the Shuttle* devices is this many bytes
#define PACKET_LEN 5


// the module name, and prefix for all HAL pins
char *modname = "shuttle";


int hal_comp_id;




// each Shuttle* device presents this interface to HAL
struct shuttle_hal {
    hal_bit_t *button[MAX_BUTTONS];
    hal_bit_t *button_not[MAX_BUTTONS];
    hal_s32_t *counts;        // accumulated counts from the jog wheel
    hal_float_t *spring_wheel_f;  // current position of the springy outer wheel, as a float from -1 to +1 inclusive
    hal_s32_t *spring_wheel_s32;  // current position of the springy outer wheel, as a s32 from -7 to +7 inclusive
};


struct shuttle {
    int fd;
    char *device_file;
    struct shuttle_hal *hal;
    int read_first_event;
    int prev_count;
    contour_dev_t *contour_type;
};


// this will become an array of all the Shuttle* devices we're using
struct shuttle **shuttle = NULL;
int num_devices = 0;




static void exit_handler(int sig) {
    (void)sig;
    printf("%s: exiting\n", modname);
    exit(0);
}


static void call_hal_exit(void) {
    hal_exit(hal_comp_id);
}




int read_update(struct shuttle *s) {
    int r;
    int8_t packet[PACKET_LEN];
    uint16_t button;

    r = read(s->fd, packet, PACKET_LEN);
    if (r < 0) {
        fprintf(stderr, "%s: error reading %s: %s\n", modname, s->device_file, strerror(errno));
        return -1;
    } else if (r == 0) {
        fprintf(stderr, "%s: EOF on %s\n", modname, s->device_file);
        return -1;
    }

    button = ((uint8_t)packet[4] << 8) | (uint8_t)packet[3];
    for (int i = 0; i < s->contour_type->num_buttons; i ++) {
        if (button & s->contour_type->button_mask[i]) {
            *s->hal->button[i] = 1;
        } else {
            *s->hal->button[i] = 0;
        }
        *s->hal->button_not[i] = !*s->hal->button[i];
    }

    {
        int curr_count = packet[1];

        if (s->read_first_event == 0) {
            *s->hal->counts = 0;
            s->prev_count = curr_count;
            s->read_first_event = 1;
        } else {
            int diff_count = curr_count - s->prev_count;
            if (diff_count > 128) diff_count -= 256;
            if (diff_count < -128) diff_count += 256;
            *s->hal->counts += diff_count;
            s->prev_count = curr_count;
        }
    }

    *s->hal->spring_wheel_s32 = packet[0];
    *s->hal->spring_wheel_f = packet[0] / 7.0;

    return 0;
}


struct shuttle *check_for_shuttle(char *dev_filename) {
    struct shuttle *s;
    struct hidraw_devinfo devinfo;
    char name[100];
    int r;

    printf("%s: checking %s\n", modname, dev_filename);

    s = (struct shuttle *)calloc(1, sizeof(struct shuttle));
    if (s == NULL) {
        fprintf(stderr, "%s: out of memory!\n", modname);
        return NULL;
    }

    s->device_file = dev_filename;

    s->fd = open(s->device_file, O_RDONLY);
    if (s->fd < 0) {
        fprintf(stderr, "%s: error opening %s: %s\n", modname, s->device_file, strerror(errno));
        if (errno == EACCES) {
            fprintf(stderr, "%s: make sure you have read permission on %s, read the shuttle(1) manpage for more info\n", modname, s->device_file);
        }
        goto fail0;
    }


    r = ioctl(s->fd, HIDIOCGRAWINFO, &devinfo);
    if (r < 0) {
        fprintf(stderr, "%s: error with ioctl HIDIOCGRAWINFO on %s: %s\n", modname, s->device_file, strerror(errno));
        goto fail1;
    }

    for (unsigned i = 0; i < sizeof(contour_dev)/sizeof(contour_dev_t); i ++) {
        if (devinfo.vendor != contour_dev[i].vendor_id) {
            continue;
        }

        if (devinfo.product != contour_dev[i].product_id) {
            continue;
        }

        s->contour_type = &contour_dev[i];
        break;
    }

    if (s->contour_type == NULL) {
        fprintf(stderr, "%s: dev %s is not a known Shuttle device\n", modname, s->device_file);
        goto fail1;
    }

    r = ioctl(s->fd, HIDIOCGRAWNAME(99), name);
    if (r < 0) {
        fprintf(stderr, "%s: error with ioctl HIDIOCGRAWNAME on %s: %s\n", modname, s->device_file, strerror(errno));
        goto fail1;
    }
    printf("%s: found %s on %s\n", modname, name, s->device_file);


    s->hal = (struct shuttle_hal *)hal_malloc(sizeof(struct shuttle_hal));
    if (s->hal == NULL) {
        fprintf(stderr, "%s: ERROR: unable to allocate HAL shared memory\n", modname);
        goto fail1;
    }

    for (int i = 0; i < s->contour_type->num_buttons; i ++) {
        r = hal_pin_bit_newf(HAL_OUT, &(s->hal->button[i]), hal_comp_id, "%s.%d.button-%d", modname, num_devices, i);
        if (r != 0) goto fail1;
        *s->hal->button[i] = 0;

        r = hal_pin_bit_newf(HAL_OUT, &(s->hal->button_not[i]), hal_comp_id, "%s.%d.button-%d-not", modname, num_devices, i);
        if (r != 0) goto fail1;
        *s->hal->button_not[i] = 1;
    }

    r = hal_pin_s32_newf(HAL_OUT, &(s->hal->counts), hal_comp_id, "%s.%d.counts", modname, num_devices);
    if (r != 0) goto fail1;

    r = hal_pin_float_newf(HAL_OUT, &(s->hal->spring_wheel_f), hal_comp_id, "%s.%d.spring-wheel-f", modname, num_devices);
    if (r != 0) goto fail1;

    r = hal_pin_s32_newf(HAL_OUT, &(s->hal->spring_wheel_s32), hal_comp_id, "%s.%d.spring-wheel-s32", modname, num_devices);
    if (r != 0) goto fail1;

    *s->hal->counts = 0;
    *s->hal->spring_wheel_f = 0.0;
    *s->hal->spring_wheel_s32 = 0;

    return s;


fail1:
    close(s->fd);

fail0:
    free(s);
    return NULL;
}




int main(int argc, char *argv[]) {
    int i;
    glob_t glob_buffer;

    char **names;
    int num_names;


    hal_comp_id = hal_init(modname);
    if (hal_comp_id < 1) {
        fprintf(stderr, "%s: ERROR: hal_init failed\n", modname);
        exit(1);
    }

    signal(SIGINT, exit_handler);
    signal(SIGTERM, exit_handler);
    atexit(call_hal_exit);


    // get the list of device filenames to check for Shuttle devices
    if (argc > 1) {
        // list of devices provided on the command line
        names = &argv[1];
        num_names = argc - 1;
    } else {
        // probe for /dev/hidraw*
        int r;

        r = glob("/dev/hidraw*", 0, NULL, &glob_buffer);
        if (r == GLOB_NOMATCH) {
            fprintf(stderr, "%s: no /dev/hidraw* found, is device plugged in?\n", modname);
            exit(1);
        } else if (r != 0) {
            fprintf(stderr, "%s: error with glob!\n", modname);
            exit(1);
        }
        names = glob_buffer.gl_pathv;
        num_names = glob_buffer.gl_pathc;

        // the pathnames we got from glob(3) are used in the shuttle array, so we intentionally dont call globfree(3)
    }


    // probe for Shuttle devices on all those device file names
    for (i = 0; i < num_names; i ++) {
        struct shuttle *s;
        s = check_for_shuttle(names[i]);
        if (s == NULL) continue;

        num_devices ++;
        shuttle = (struct shuttle **)realloc(shuttle, (num_devices * sizeof(struct shuttle *)));
        if (shuttle == NULL) {
            fprintf(stderr, "%s: out of memory!\n", modname);
            exit(1);
        }
        shuttle[num_devices - 1] = s;
    }


    if (num_devices == 0) {
        fprintf(stderr, "%s: no devices found\n", modname);
        exit(1);
    }


    hal_ready(hal_comp_id);


    // select on all the hidraw devices, process events from the active ones
    while (1) {
        fd_set readers;
        int max_fd;
        int i;
        int r;

        FD_ZERO(&readers);
        max_fd = -1;

        for (i = 0; i < num_devices; i ++) {
            FD_SET(shuttle[i]->fd, &readers);
            max_fd = Max(max_fd, shuttle[i]->fd);
        }

        r = select(max_fd + 1, &readers, NULL, NULL, NULL);
        if (r < 0) {
            if ((errno == EAGAIN) || (errno == EINTR)) continue;
            fprintf(stderr, "%s: error with select!\n", modname);
            exit(1);
        }

        for (i = 0; i < num_devices; i ++) {
            if (FD_ISSET(shuttle[i]->fd, &readers)) {
                r = read_update(shuttle[i]);
                if (r < 0) {
                    exit(1);
                }
            }
        }
    }

    exit(0);
}

