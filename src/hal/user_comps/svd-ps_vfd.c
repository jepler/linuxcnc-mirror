/*
    svd-ps_vfd.c

    This is a userspace program that interfaces the Soyan Power SVD-PS VFDs
    to the LinuxCNC HAL.

    Copyright (C) 2020 Tinic Uro
    Copyright (C) 2017 Sebastian Kuzminsky

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation, version 2.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301-1307 USA.
*/

#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <modbus.h>

#include "hal.h"
#include "rtapi.h"


// If a Modbus transaction fails, retry this many times before giving up.
#define NUM_MODBUS_RETRIES 5


typedef struct {
    hal_float_t *period;

    hal_float_t *speed_cmd;
    hal_float_t *freq_cmd;
    hal_bit_t *at_speed;

    hal_bit_t	*spindle_on;

    hal_u32_t *modbus_errors;
} haldata_t;

haldata_t *haldata;

int hal_comp_id;


typedef struct {
    int address;  // 0-0xffff for a register, -1 for end-of-list
    const char *name;

    // Multiply the uint16 value read by this multiplier to get the human-
    // readable value.
    float multiplier;

    hal_float_t *hal_pin;
} modbus_register_t;

int num_modbus_registers = 0;
modbus_register_t *modbus_register;


static int done;
char *modname = "svd-ps_vfd";

float motor_max_speed = 0.0;
float max_freq = 0.0;
float min_freq = 0.0;

int baud;

static struct option long_options[] = {
    {"device", 1, 0, 'd'},
    {"rate", 1, 0, 'r'},
    {"bits", 1, 0, 'b'},
    {"parity", 1, 0, 'p'},
    {"stopbits", 1, 0, 's'},
    {"target", 1, 0, 't'},
    {"verbose", 0, 0, 'v'},
    {"help", 0, 0, 'h'},
    {"motor-max-speed", 1, 0, 'S'},
    {"max-frequency", 1, 0, 'F'},
    {"min-frequency", 1, 0, 'f'},
    {0,0,0,0}
};

static char *option_string = "d:r:b:p:s:t:vhS:F:f:";

static char *bitstrings[] = {"5", "6", "7", "8", NULL};

static char *paritystrings[] = {"even", "odd", "none", NULL};
static char paritychars[] = {'E', 'O', 'N'};

static char *ratestrings[] = {"1200", "2400", "4800", "9600", "19200", "38400", NULL};
static char *stopstrings[] = {"1", "2", NULL};


static void quit(int sig) {
    (void)sig;
    done = 1;
}


int match_string(char *string, char **matches) {
    size_t len;
    int which, match;
    which=0;
    match=-1;
    if ((matches==NULL) || (string==NULL)) return -1;
    len = strlen(string);
    while (matches[which] != NULL) {
        if ((!strncmp(string, matches[which], len)) && (len <= strlen(matches[which]))) {
            if (match>=0) return -1;        // multiple matches
            match=which;
        }
        ++which;
    }
    return match;
}


void usage(int argc, char **argv) {
    (void)argc;
    printf("Usage:  %s [ARGUMENTS]\n", argv[0]);
    printf(
        "\n"
        "This program interfaces the Soyan Power SVD-PS VFD to the LinuxCNC HAL.\n"
        "\n"
        "Required arguments:\n"
        "    -S, --motor-max-speed SPEED\n"
        "        The motor's max speed in RPM.\n"
        "    -F, --max-frequency F\n"
        "        This is the maximum output frequency of the VFD in Hz.\n"
        "    -f, --min-frequency F\n"
        "        This is the minimum output frequency of the VFD in Hz.\n"
        "\n"
        "Optional arguments:\n"
        "    -d, --device PATH (default /dev/ttyS0)\n"
        "        Set the name of the serial device to use.\n"
        "    -r, --rate RATE (default 9600)\n"
        "        Set baud rate to RATE.  It is an error if the rate is not one of\n"
        "        the following: 1200, 2400, 4800, 9600, 19200, 38400\n"
        "    -b, --bits BITS (default 8)\n"
        "        Set number of data bits to BITS, must be between 5 and 8 inclusive.\n"
        "    -p, --parity PARITY (default none)\n"
        "        Set serial parity to one of 'even', 'odd', or 'none'.\n"
        "    -s, --stopbits {1,2} (default 2)\n"
        "        Set serial stop bits to 1 or 2.\n"
        "    -t, --target TARGET (default 1)\n"
        "        Set Modbus target (slave) number.  This must match the device\n"
        "        number you set on the SVD-PS VFD.\n"
        "    -v, --verbose\n"
        "        Turn on verbose mode.\n"
        "    -h, --help\n"
        "        Show this help.\n"
    );
}


void svd_modbus_sleep(void) {
    float seconds_per_bit = 1.0 / (float)baud;
    useconds_t useconds_per_bit = seconds_per_bit * 1000 * 1000;
    useconds_t delay = useconds_per_bit * 8 * 5;
    usleep(delay);
}


int set_motor_on_forward(modbus_t *mb) {
    int r;
    uint16_t addr = 0x2000;
    uint16_t val = 1;

    for (int retries = 0; retries <= NUM_MODBUS_RETRIES; retries ++) {
        svd_modbus_sleep();
        r = modbus_write_register(mb, addr, val);
        if (r == 1) {
            return 0;
        }
        fprintf(stderr, "%s: error writing %u to register 0x%04x: %s\n", __func__, val, addr, modbus_strerror(errno));
        *haldata->modbus_errors = *haldata->modbus_errors + 1;
    }
    return -1;
}

int set_motor_off(modbus_t *mb) {
    int r;
    uint16_t addr = 0x2000;
    uint16_t val = 5;

    for (int retries = 0; retries <= NUM_MODBUS_RETRIES; retries ++) {
        svd_modbus_sleep();
        r = modbus_write_register(mb, addr, val);
        if (r == 1) {
            return 0;
        }
        fprintf(stderr, "%s: error writing %u to register 0x%04x: %s\n", __func__, val, addr, modbus_strerror(errno));
        *haldata->modbus_errors = *haldata->modbus_errors + 1;
    }
    return -1;
}


int set_motor_frequency(modbus_t *mb, float freq) {
    int r;
    uint16_t addr = 0x1000;
    uint16_t val;  // 100 * the frequency percentage
    val = (freq / max_freq) * 10000;

    for (int retries = 0; retries <= NUM_MODBUS_RETRIES; retries ++) {
        svd_modbus_sleep();
        r = modbus_write_register(mb, addr, val);
        if (r == 1) {
            return 0;
        }
        fprintf(stderr, "%s: error writing %u to register 0x%04x: %s\n", __func__, val, addr, modbus_strerror(errno));
        *haldata->modbus_errors = *haldata->modbus_errors + 1;
    }
    return -1;
}


int read_modbus_register(modbus_t *mb, modbus_register_t *reg) {
    uint16_t data;
    int r;

    for (int retries = 0; retries <= NUM_MODBUS_RETRIES; retries ++) {
        svd_modbus_sleep();
        r = modbus_read_registers(mb, reg->address, 1, &data);
        if (r == 1) {
            *reg->hal_pin = data * reg->multiplier;
            return 0;
        }
        fprintf(stderr, "%s: error reading %s (register 0x%04x): %s\n", __func__, reg->name, reg->address, modbus_strerror(errno));
        *haldata->modbus_errors = *haldata->modbus_errors + 1;
    }
    return -1;
}


void read_modbus_registers(modbus_t *mb) {
    for (int i = 0; i < num_modbus_registers; i ++) {
        (void)read_modbus_register(mb, &modbus_register[i]);
    }
}


modbus_register_t *add_modbus_register(modbus_t *mb, int address, const char *pin_name, float multiplier) {
    int r;
    modbus_register_t *reg;

    reg = &modbus_register[num_modbus_registers];

    r = hal_pin_float_newf(HAL_OUT, &reg->hal_pin, hal_comp_id, "%s.%s", modname, pin_name);
    if (r != 0) {
        return NULL;
    }

    reg->address = address;
    reg->name = pin_name;
    reg->multiplier = multiplier;

    read_modbus_register(mb, reg);

    num_modbus_registers ++;

    return reg;
}


int main(int argc, char **argv) {
    char *device;
    int bits;
    char parity;
    int stopbits;
    int verbose;

    int retval = 0;
    modbus_t *mb;
    int slave;
    struct timespec period_timespec;
    char *endarg;
    int opt;
    int argindex, argvalue;

    modbus_register_t *speed_fb_reg;

    done = 0;

    // assume that nothing is specified on the command line
    device = "/dev/ttyS0";
    baud = 9600;
    bits = 8;
    parity = 'N';
    stopbits = 2;

    verbose = 0;

    /* slave / register info */
    slave = 1;

    // process command line options
    while ((opt = getopt_long(argc, argv, option_string, long_options, NULL)) != -1) {
        switch (opt) {
            case 'd':   // device name, default /dev/ttyS0
                // could check the device name here, but we'll leave it to the library open
                if (strlen(optarg) > FILENAME_MAX) {
                    printf("ERROR: device node name is too long: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                device = strdup(optarg);
                break;

            case 'r':   // Baud rate, 9600 default
                argindex=match_string(optarg, ratestrings);
                if (argindex<0) {
                    printf("ERROR: invalid baud rate: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                baud = atoi(ratestrings[argindex]);
                break;

            case 'b':   // serial data bits, probably should be 8 (and defaults to 8)
                argindex=match_string(optarg, bitstrings);
                if (argindex<0) {
                    printf("ERROR: invalid number of bits: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                bits = atoi(bitstrings[argindex]);
                break;

            case 'p':   // parity, should be a string like "even", "odd", or "none"
                argindex=match_string(optarg, paritystrings);
                if (argindex<0) {
                    printf("ERROR: invalid parity: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                parity = paritychars[argindex];
                break;

            case 's':   // stop bits, defaults to 2
                argindex=match_string(optarg, stopstrings);
                if (argindex<0) {
                    printf("ERROR: invalid number of stop bits: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                stopbits = atoi(stopstrings[argindex]);
                break;

            case 't':   // target number (MODBUS ID), default 1
                argvalue = strtol(optarg, &endarg, 10);
                if ((*endarg != '\0') || (argvalue < 1) || (argvalue > 254)) {
                    printf("ERROR: invalid slave number: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                slave = argvalue;
                break;

            case 'v':
                verbose = 1;
                break;

            case 'h':
                usage(argc, argv);
                exit(0);
                break;

            case 'S':
                motor_max_speed = strtof(optarg, &endarg);
                if ((*endarg != '\0') || (motor_max_speed == 0.0)) {
                    printf("%s: ERROR: invalid motor max speed: %s\n", modname, optarg);
                    exit(1);
                }
                break;

            case 'F':
                max_freq = strtof(optarg, &endarg);
                if ((*endarg != '\0') || (max_freq == 0.0)) {
                    printf("%s: ERROR: invalid max freq: %s\n", modname, optarg);
                    exit(1);
                }
                break;

            case 'f':
                min_freq = strtof(optarg, &endarg);
                if ((*endarg != '\0') || (min_freq == 0.0)) {
                    printf("%s: ERROR: invalid min freq: %s\n", modname, optarg);
                    exit(1);
                }
                break;

            default:
                usage(argc, argv);
                exit(1);
                break;
        }
    }

    if (motor_max_speed == 0.0) {
        printf("%s: must specify --motor-max-speed\n", modname);
        exit(1);
    }

    if (max_freq == 0.0) {
        printf("%s: must specify --max-frequency\n", modname);
        exit(1);
    }

    if (min_freq == 0.0) {
        printf("%s: must specify --min-frequency\n", modname);
        exit(1);
    }

    if (min_freq > max_freq) {
        printf("%s: min frequency (%f) must be less than max frequency (%f)\n", modname, min_freq, max_freq);
        exit(1);
    }

    {
        struct sigaction sa;
        int r;

        sa.sa_handler = quit;
        sigemptyset(&sa.sa_mask);

        // libmodbus sometimes segfaults if its system calls get
        // interrupted by signals.  This works around it by automatically
        // restarting the system call instead of having it return EINTR.
        sa.sa_flags = SA_RESTART;

        r = sigaction(SIGINT, &sa, NULL);
        if (r != 0) {
            printf("failed to set SIGINT handler: %s\n", strerror(errno));
            exit(1);
        }
        r = sigaction(SIGTERM, &sa, NULL);
        if (r != 0) {
            printf("failed to set SIGTERM handler: %s\n", strerror(errno));
            exit(1);
        }
    }

    printf("%s: device='%s', baud=%d, parity='%c', bits=%d, stopbits=%d, address=%d\n",
           modname, device, baud, parity, bits, stopbits, slave);

    mb = modbus_new_rtu(device, baud, parity, bits, stopbits);
    if (mb == NULL) {
        printf("%s: ERROR: couldn't open Modbus serial device: %s\n", modname, modbus_strerror(errno));
        goto out_noclose;
    }

    {
        struct timeval t;

        // Set the response timeout.
        t.tv_sec = 0;
        t.tv_usec = 30 * 1000;
#if (LIBMODBUS_VERSION_CHECK(3, 1, 2))
        modbus_set_response_timeout(mb, t.tv_sec, t.tv_usec);
#else
        modbus_set_response_timeout(mb, &t);
#endif

        // Disable the byte timeout so it just waits for the complete
        // response timeout instead.
#if (LIBMODBUS_VERSION_CHECK(3, 1, 2))
        t.tv_sec = 0;
        t.tv_usec = 0;
        modbus_set_byte_timeout(mb, t.tv_sec, t.tv_usec);
#else
        t.tv_sec = -1;
        modbus_set_byte_timeout(mb, &t);
#endif
    }

    retval = modbus_connect(mb);
    if (retval != 0) {
        printf("%s: ERROR: couldn't open serial device: %s\n", modname, modbus_strerror(errno));
        goto out_noclose;
    }

    modbus_set_debug(mb, verbose);

    modbus_set_slave(mb, slave);

    /* create HAL component */
    hal_comp_id = hal_init(modname);
    if (hal_comp_id < 0) {
        printf("%s: ERROR: hal_init failed\n", modname);
        retval = hal_comp_id;
        goto out_close;
    }

    haldata = (haldata_t *)hal_malloc(sizeof(haldata_t));
    if (haldata == NULL) {
        printf("%s: ERROR: unable to allocate shared memory\n", modname);
        retval = -1;
        goto out_closeHAL;
    }

    retval = hal_pin_float_newf(HAL_IN, &(haldata->period), hal_comp_id, "%s.period-seconds", modname);
    if (retval != 0) goto out_closeHAL;

    retval = hal_pin_float_newf(HAL_IN, &(haldata->speed_cmd), hal_comp_id, "%s.speed-cmd", modname);
    if (retval != 0) goto out_closeHAL;

    retval = hal_pin_float_newf(HAL_OUT, &(haldata->freq_cmd), hal_comp_id, "%s.freq-cmd", modname);
    if (retval != 0) goto out_closeHAL;

    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->at_speed), hal_comp_id, "%s.at-speed", modname);
    if (retval != 0) goto out_closeHAL;

    retval = hal_pin_bit_newf(HAL_IN, &(haldata->spindle_on), hal_comp_id, "%s.spindle-on", modname);
    if (retval != 0) goto out_closeHAL;

    retval = hal_pin_u32_newf(HAL_OUT, &(haldata->modbus_errors), hal_comp_id, "%s.modbus-errors", modname);
    if (retval != 0) goto out_closeHAL;

    *haldata->period = 0.1;

    *haldata->freq_cmd = 0.0;
    *haldata->at_speed = 0;

    *haldata->modbus_errors = 0;

    modbus_register = (modbus_register_t *)hal_malloc(20 * sizeof(modbus_register_t));
    if (modbus_register == NULL) {
        printf("%s: ERROR: unable to allocate memory\n", modname);
        retval = -1;
        goto out_closeHAL;
    }
    if (add_modbus_register(mb, 0x1001, "freq-fb", 0.01) == NULL) {
        goto out_closeHAL;
    }
    if (add_modbus_register(mb, 0x1002, "dc-bus-voltage", 0.1) == NULL) {
        goto out_closeHAL;
    }
    if (add_modbus_register(mb, 0x1003, "output-voltage", 1.0) == NULL) {
        goto out_closeHAL;
    }
    if (add_modbus_register(mb, 0x1004, "output-current", 1.0) == NULL) {
        goto out_closeHAL;
    }
    speed_fb_reg = add_modbus_register(mb, 0x1005, "speed-fb", 1.0);
    if (speed_fb_reg == NULL) {
        goto out_closeHAL;
    }
    if (add_modbus_register(mb, 0x1006, "output-power", 1.0) == NULL) {
        goto out_closeHAL;
    }
    if (add_modbus_register(mb, 0x1009, "input-terminal", 1) == NULL) {
        goto out_closeHAL;
    }
    if (add_modbus_register(mb, 0x100a, "AI1", 0.001) == NULL) {
        goto out_closeHAL;
    }
    if (add_modbus_register(mb, 0x100b, "AI2", 0.001) == NULL) {
        goto out_closeHAL;
    }

    // Activate HAL component
    hal_ready(hal_comp_id);

    while (done == 0) {
        if (*haldata->period < 0.001) *haldata->period = 0.001;
        if (*haldata->period > 2.0) *haldata->period = 2.0;
        period_timespec.tv_sec = (time_t)(*haldata->period);
        period_timespec.tv_nsec = (long)((*haldata->period - period_timespec.tv_sec) * 1000000000l);
        nanosleep(&period_timespec, NULL);

        read_modbus_registers(mb);

        if (*haldata->spindle_on) {
	    set_motor_on_forward(mb);
            *haldata->freq_cmd = (*haldata->speed_cmd / motor_max_speed) * max_freq;
            set_motor_frequency(mb, *haldata->freq_cmd);

            if ((fabs(*haldata->speed_cmd - *speed_fb_reg->hal_pin) / *haldata->speed_cmd) < 0.02) {
                *haldata->at_speed = 1;
            } else {
                *haldata->at_speed = 0;
            }
        } else {
            set_motor_off(mb);
            *haldata->at_speed = 0;
            *haldata->freq_cmd = 0.0;
        }

    }

    usleep(10 * 1000);
    set_motor_off(mb);

    retval = 0;	/* if we get here, then everything is fine, so just clean up and exit */
out_closeHAL:
    hal_exit(hal_comp_id);
out_close:
    modbus_close(mb);
    modbus_free(mb);
out_noclose:
    return retval;
}
