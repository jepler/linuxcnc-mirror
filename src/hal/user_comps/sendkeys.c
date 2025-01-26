//
//   Copyright (C) 2021 Andy Pugh
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
//   This program is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU General Public License for more details.
//
//   You should have received a copy of the GNU General Public License
//   along with this program; if not, write to the Free Software
//   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

#include "rtapi.h"
#include "hal.h"
#include <linux/uinput.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_NAME 12

static int comp_id;

typedef struct{
    hal_u32_t *keycode;
    hal_s32_t *current_event;
    hal_bit_t *init;
    hal_bit_t **trigger;
    hal_u32_t *event;
} sendkeys_hal;

typedef struct {
    int num_codes;
    int num_events;
    int num_triggers;
    bool inited;
    int fd;
    bool *prev;
    hal_u32_t oldcode;
} sendkeys_param;

typedef struct {
    sendkeys_hal *hal;
    sendkeys_param *param;
    int num_insts;
} sendkeys ;

static sendkeys *me;

void emit(int fd, int type, int code, int val)
{
   struct input_event ie;

   ie.type = type;
   ie.code = code;
   ie.value = val;
   if (write(fd, &ie, sizeof(ie)) != sizeof(ie)) { perror("write(input_event)"); }
}

static void exit_handler(int sig) {
    (void)sig;
    printf("sendkeys: exiting\n");
    exit(0);
}

static void call_hal_exit(void) {
    hal_exit(comp_id);
}

int init(int argc, char* argv[]){
    int *codes;
    int *pins;
    int i, j;
    comp_id = hal_init("sendkeys");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "sendkeys: ERROR: hal_init() failed\n");
        return -1;
    }

    me = hal_malloc(sizeof(sendkeys));
    codes = malloc(sizeof(int));
    pins = malloc(sizeof(int));
    // This is all because RTAPI_MP_ARRAY_** macros do not appear to work in userspace
    for (i = 1; i < argc; i++){
        static int index = -1;
        static int type = -1;
        void *v1, *v2;
        int ptr = 0;
        bool shift = 0;
        if (strncmp(argv[i], "config=", 7) == 0) type = 1, ptr = 6;
        if (strncmp(argv[i], "names=", 6) == 0) type = 2, ptr = 5;
        switch (type) {
        case 1: // config
            while (argv[i][ptr]){
                switch (argv[i][ptr]){
                case '=':
                case ' ':
                case ',':
                    index++;
                    v1 = realloc(codes, (index + 1) * sizeof(int));
                    v2 = realloc(pins, (index + 1) * sizeof(int));
                    if (!v1 || !v2) {
                        free(v1 ? v1 : codes);
                        free(v2 ? v2 : pins);
                        rtapi_print_msg(RTAPI_MSG_ERR, "sendkeys.N.keycode error\n");
                        return -ENOMEM;
                    } else {
                        codes=v1;
			pins=v2;
                    }
                    codes[index] = 0;
                    pins[index] = 0;
                    shift = 0;
                    break;
                case 's':
                case 'S':
                    shift = 0;
                    break;
                case 't':
                case 'T':
                    shift = 1;
                    break;
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                    if (shift){
                    pins[index] = pins[index] * 10 + argv[i][ptr] - '0';
                    } else {
                    codes[index] = codes[index] * 10 + argv[i][ptr] - '0';
                    }
                    break;
                }
                ptr++;;
            }
            me->num_insts = index + 1;
            break;
        case 2: //  Parsing of "names" would be here, but is not enabled yet
            while (argv[i][ptr]){
                ptr++;
            }
            break;
        }
    }
    me->hal = (sendkeys_hal*)hal_malloc(me->num_insts * sizeof(sendkeys_hal));
    me->param = malloc(me->num_insts * sizeof(sendkeys_param));
    for (i = 0; i < me->num_insts; i++){
        sendkeys_hal* hal = &(me->hal[i]);
        sendkeys_param* param = &(me->param[i]);
        if (hal_pin_u32_newf(HAL_IN, &(hal->keycode), comp_id,
        "sendkeys.%i.keycode", i) < 0) {
        free(codes);
        free(pins);
        rtapi_print_msg(RTAPI_MSG_ERR, "sendkeys.N.keycode error\n");
        return -ENOMEM;}
        if (hal_pin_s32_newf(HAL_OUT, &(hal->current_event), comp_id,
        "sendkeys.%i.current-event", i) < 0) {
        free(codes);
        free(pins);
        rtapi_print_msg(RTAPI_MSG_ERR, "sendkeys.N.current-event error\n");
        return -ENOMEM;}
        if (hal_pin_bit_newf(HAL_IN, &(hal->init), comp_id,
        "sendkeys.%i.init", i) < 0) {
        free(codes);
        free(pins);
        rtapi_print_msg(RTAPI_MSG_ERR, "sendkeys.N.init error\n");
        return -ENOMEM;}
        // event params
        rtapi_print_msg(RTAPI_MSG_DBG, "instance %i events %i triggers %i\n", i, codes[i], pins[i]);
        param->num_triggers = pins[i];
        param->num_codes = codes[i];
        param->num_events = codes[i] + pins[i];
        hal->event = hal_malloc(param->num_events  * sizeof(hal_u32_t*));
        for (j = 0; j < param->num_codes; j++){
            if (hal_param_u32_newf(HAL_RW, &(hal->event[j]), comp_id,
                    "sendkeys.%i.scan-event-%02i", i, j) < 0) {
                free(codes);
                free(pins);
                rtapi_print_msg(RTAPI_MSG_ERR, "sendkeys.N.scan-event-%02i error\n", j);
                return -ENOMEM;}
        }
        for (j = 0; j < param->num_triggers; j++){
            if (hal_param_u32_newf(HAL_RW, &(hal->event[j + param->num_codes]), comp_id,
                    "sendkeys.%i.pin-event-%02i", i, j) < 0) {
                free(codes);
                free(pins);
                rtapi_print_msg(RTAPI_MSG_ERR, "sendkeys.N.pin-event-%02i error\n", j);
                return -ENOMEM;}
        }
        // trigger pins
        hal->trigger = hal_malloc(param->num_triggers * sizeof(hal_bit_t*));
        param->prev = malloc(param->num_triggers * sizeof(hal_bit_t));
        for (j = 0; j < param->num_triggers; j++){
            if (hal_pin_bit_newf(HAL_IN, &(hal->trigger[j]), comp_id,
                    "sendkeys.%i.trigger-%02i", i, j) < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "sendkeys.N.trigger-%02i error\n",j);
                free(codes);
                free(pins);
                return -ENOMEM;}
            param->prev[j] = 0;
        }
    }
    free(codes);
    free(pins);
    return 0;
}

int main(int argc, char* argv[]) {
    int i, j;
    
    signal(SIGINT, exit_handler);
    signal(SIGTERM, exit_handler);
    atexit(call_hal_exit);

    //parse configs and create pins
    init(argc, argv);
    hal_ready(comp_id);
    
    while (1) {
        for (i = 0; i < me->num_insts; i++){
            sendkeys_hal* hal = &(me->hal[i]);
            sendkeys_param* param = &(me->param[i]);
            if ((*hal->keycode & 0xC0) == 0x80){
                *hal->current_event = (*hal->keycode & 0x3f);
            } else {
                *hal->current_event = -1;
            }
            // The event codes are written to pins in HAL after the 
            // component is loaded.
            // We need to set up the keyboard based on the events selected
            // so there is a secondary init here.
            if (! *hal->init) continue;
            if (*hal->init && ! param->inited) {
                param->fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
                if (param->fd < 0) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                        "Cannot open /dev/uinput. Suggest chmod 666 /dev/uinput\n");
                    abort();
                }
                ioctl(param->fd, UI_SET_EVBIT, EV_KEY);
                for (j = 0; j < param->num_events; j++){
                    if (hal->event[j] > 0 && hal->event[j] < KEY_MAX){
                        ioctl(param->fd, UI_SET_KEYBIT, hal->event[j]);
                        rtapi_print("SET_EVBIT %i\n", hal->event[j]);
                    }
                }
                struct uinput_user_dev uidev;
                memset(&uidev, 0, sizeof(uidev));
                strcpy(uidev.name, "linuxcnc-hal");
                uidev.id.bustype = BUS_USB;
                uidev.id.vendor  = 0x1;
                uidev.id.product = 0x1;
                uidev.id.version = 1;

                if (write(param->fd, &uidev, sizeof(uidev)) != sizeof(uidev)) { perror("write(uinput_user_dev)"); abort(); }
                if (ioctl(param->fd, UI_DEV_CREATE) < 0) { perror("ioctl(UI_DEV_CREATE)"); abort(); }
                param->inited = 1;
            }
            if (*hal->keycode != param->oldcode) {
                /* Key press, report the event, send key release, and report again*/
                if ((int)(*hal->keycode & 0x3F) > param->num_events) continue;
                if (hal->event[*hal->keycode & 0x3F] == 0) continue;
                if ((*hal->keycode & 0xC0) == 0xC0){ // keydown
                    emit(param->fd, EV_KEY, hal->event[*hal->keycode & 0x3F], 1);
                    emit(param->fd, EV_SYN, SYN_REPORT, 0);
                } else if ((*hal->keycode & 0xC0) == 0x80){ // keyup
                    emit(param->fd, EV_KEY, hal->event[*hal->keycode & 0x3F], 0);
                    emit(param->fd, EV_SYN, SYN_REPORT, 0);
                }
                param->oldcode = *hal->keycode;
            }
            for (j = 0; j < param->num_triggers; j++){
                if (param->prev[j] != *hal->trigger[j]){
                    if (*hal->trigger[j]){ // keydown
                        emit(param->fd, EV_KEY, hal->event[param->num_codes + j], 1);
                        emit(param->fd, EV_SYN, SYN_REPORT, 0);
                    } else { // keyup
                        emit(param->fd, EV_KEY, hal->event[param->num_codes + j], 0);
                        emit(param->fd, EV_SYN, SYN_REPORT, 0);
                    }
                    param->prev[j] = *hal->trigger[j];
                }
            }
        }
    }
}
