//    Copyright 2014 Jeff Epler
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#include <errno.h>
#include <fcntl.h>
#include <linux/ppdev.h>
#include <map>
#include <rtapi.h>
#include <rtapi_parport.h>
#include "rtapi_uspace.hh"
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

struct portinfo {
    int port_id;
    unsigned short base;
    unsigned short base_hi;
};

typedef std::map<unsigned short, portinfo> ParportMap;

static ParportMap parports;

static void map_parports() {
    for(int i=0; i<16; i++) {
        const char path_template[] = "/proc/sys/dev/parport/parport%d/base-addr";
        char path[sizeof(path_template)]; // noting that we stop before 100, so it'll fit
        snprintf(path, sizeof(path), path_template, i);

        FILE *f = fopen(path, "r");
        if(!f) {
            if(errno != ENOENT)
                rtapi_print_msg(RTAPI_MSG_ERR, "fopen(%s): %s\n", path, strerror(errno));
            continue;
        }
        struct portinfo pi;
        pi.port_id = i;
        if(fscanf(f, "%hd %hd", &pi.base, &pi.base_hi) != 2) {
            rtapi_print_msg(RTAPI_MSG_ERR, "Failed to parse base-addr for port #%d\n", i);
            fclose(f);
            continue;
        }
        fclose(f);

        parports[i] = pi;
        parports[pi.base] = pi;
    }
}

int rtapi_parport_get(const char * /*mod_name*/, rtapi_parport_t *port, unsigned short base, unsigned short base_hi, unsigned int modes) {
    WITH_ROOT;

    memset(port, 0, sizeof(*port));
    port->fd = -1;

    if(parports.empty()) map_parports();
    ParportMap::iterator pi = parports.find(base);
    if(pi == parports.end()) {
        if(base < 16){
            rtapi_print_msg(RTAPI_MSG_ERR, "Linux parallel port %d not found\n", base);
            return -ENOENT;}
        rtapi_print_msg(RTAPI_MSG_ERR, "No parport registered at 0x%x. "
                    "This is not always an error. Continuing.\n", base);
        port->base = base;
        port->base_hi = base_hi;
        return 0;
    } else {
        port->base = pi->second.base;
        port->base_hi = pi->second.base_hi;
        const char port_template[] = "/dev/parport%d";

        char port_path[sizeof(port_template)];
        snprintf(port_path, sizeof(port_path), port_template, pi->second.port_id);
        port->fd = open(port_path, O_RDWR);

        if(port->fd < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "open(%s): %s\n", port_path, strerror(errno));
            return -errno;
        }

        if(ioctl(port->fd, PPCLAIM) < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "ioctl(%s, PPCLAIM): %s\n", port_path, strerror(errno));
            close(port->fd);
            port->fd = -1;
            return -errno;
        }

        int ppmodes = ~0;

        if(ioctl(port->fd, PPGETMODES, &modes) < 0) {
            rtapi_print_msg(RTAPI_MSG_WARN, "ioctl(%s, PPGETMODES): %s\n", port_path, strerror(errno));
        }

        if((modes & ppmodes) != modes)
        {
            rtapi_print_msg(RTAPI_MSG_WARN,
                "PARPORT: linux parport %s does not support mode %x.\n"
                "PARPORT: continuing anyway.\n",
                port_path, modes);
        }

    }
    return 0;
}

void rtapi_parport_release(rtapi_parport_t *port) {
    close(port->fd);
    port->fd = -1;
}
