/*
    gs2_vfd.c
    Copyright (C) 2013 Sebastian Kuzminsky
    Copyright (C) 2009 John Thornton
    Copyright (C) 2007, 2008 Stephen Wille Padnos, Thoth Systems, Inc.

    Based on a work (test-modbus program, part of libmodbus) which is
    Copyright (C) 2001-2005 Stéphane Raimbault <stephane.raimbault@free.fr>

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation, version 2.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301-1307  USA.


    This is a userspace program that interfaces the Automation Direct
    GS2 VFD to the LinuxCNC HAL.

*/

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <getopt.h>
#include "rtapi.h"
#include "hal.h"
#include <modbus.h>

/* Read Registers:
	0x2100 = status word 1
	0x2101 = status word 2
	0x2102 = frequency command
	0x2103 = actual frequency
	0x2104 = output current
	0x2105 = DC bus voltage
	0x2106 = actual output voltage
	0x2107 = actual RPM
	0x2108 + 0x2109 = scale freq (not sure what this actually is - it's the same as 0x2103)
	0x210A = power factor.  Not sure of the units (1/10 or 1/100)
	0x210B = load percentage
	0x210C = Firmware revision (never saw anything other than 0 here)
	total of 13 registers		*/
#define START_REGISTER_R	0x2100
#define NUM_REGISTERS_R		13
/* write registers:
	0x91A = Speed reference, in 1/10Hz increments
	0x91B = RUN command, 0=stop, 1=run
	0x91C = direction, 0=forward, 1=reverse
	0x91D = serial fault, 0=no fault, 1=fault (maybe can stop with this?)
	0x91E = serial fault reset, 0=no reset, 1 = reset fault
	total of 5 registers */
#define START_REGISTER_W	0x091A
#define NUM_REGISTERS_W		5


#define GS2_REG_STOP_METHOD                             0x0100
#define GS2_REG_STOP_METHOD__RAMP_TO_STOP               0
#define GS2_REG_STOP_METHOD__COAST_TO_STOP              1

#define GS2_REG_ACCELERATION_TIME_1                     0x0101

#define GS2_REG_DECELERATION_TIME_1                     0x0102

#define GS2_REG_OVER_VOLTAGE_STALL_PREVENTION           0x0605
#define GS2_REG_OVER_VOLTAGE_STALL_PREVENTION__ENABLE   0
#define GS2_REG_OVER_VOLTAGE_STALL_PREVENTION__DISABLE  1


/* Modbus slave data struct */
typedef struct {
	int slave;		/* slave address */
	int read_reg_start;	/* starting read register number */
	int read_reg_count;	/* number of registers to read */
	int write_reg_start;	/* starting write register number */
	int write_reg_count;	/* number of registers to write */
} slavedata_t;

/* HAL data struct */
typedef struct {
  hal_s32_t	*stat1;		// status words from the VFD.  Maybe split these out sometime
  hal_s32_t	*stat2;
  hal_float_t	*freq_cmd;	// frequency command
  hal_float_t	*freq_out;	// actual output frequency
  hal_float_t	*curr_out;	// output current
  hal_float_t	*DCBusV;	//
  hal_float_t	*outV;
  hal_float_t	*RPM;
  hal_float_t	*scale_freq;
  hal_float_t	*power_factor;
  hal_float_t	*load_pct;
  hal_s32_t	*FW_Rev;
  hal_s32_t	errorcount;
  hal_float_t	looptime;
  hal_float_t	speed_tolerance;
  hal_s32_t	retval;
  hal_bit_t		*at_speed;		// when drive freq_cmd == freq_out and running
  hal_bit_t		*is_stopped;	// when drive freq out is 0
  hal_float_t	*speed_command;		// speed command input
  hal_float_t	motor_hz;		// speeds are scaled in Hz, not RPM
  hal_float_t	motor_RPM;		// nameplate RPM at default Hz
  hal_bit_t	*spindle_on;		// spindle 1=on, 0=off
  hal_bit_t	*spindle_fwd;		// direction, 0=fwd, 1=rev
  hal_bit_t *spindle_rev;		// on when in rev and running
  hal_bit_t	*err_reset;		// reset errors when 1
  hal_s32_t ack_delay;		// number of read/writes before checking at-speed

  hal_bit_t	old_run;		// so we can detect changes in the run state
  hal_bit_t	old_dir;
  hal_bit_t	old_err_reset;
  hal_bit_t    *ena_gs2comp;    // gs2 component enable pin
  hal_bit_t    *isInitialized;    // initialized status pin
} haldata_t;

static int done;
char *modname = "gs2_vfd";

static struct option long_options[] = {
    {"bits", 1, 0, 'b'},
    {"device", 1, 0, 'd'},
    {"debug", 0, 0, 'g'},
    {"help", 0, 0, 'h'},
    {"name", 1, 0, 'n'},
    {"parity", 1, 0, 'p'},
    {"rate", 1, 0, 'r'},
    {"stopbits", 1, 0, 's'},
    {"target", 1, 0, 't'},
    {"verbose", 0, 0, 'v'},
    {"accel-seconds", required_argument, NULL, 'A'},
    {"decel-seconds", required_argument, NULL, 'D'},
    {"braking-resistor", no_argument, NULL, 'R'},
    {"disable", no_argument, NULL, 'X'},
    {0,0,0,0}
};

static char *option_string = "gb:d:hn:p:r:s:t:vA:D:RX";

static char *bitstrings[] = {"5", "6", "7", "8", NULL};

// The old libmodbus (v2?) used strings to indicate parity, the new one
// (v3.0.1) uses chars.  The gs2_vfd driver gets the string indicating the
// parity to use from the command line, and I don't want to change the
// command-line usage.  The command-line argument string must match an
// entry in paritystrings, and the index of the matching string is used as
// the index to the parity character for the new libmodbus.
static char *paritystrings[] = {"even", "odd", "none", NULL};
static char paritychars[] = {'E', 'O', 'N'};

static char *ratestrings[] = {"110", "300", "600", "1200", "2400", "4800", "9600",
    "19200", "38400", "57600", "115200", NULL};
static char *stopstrings[] = {"1", "2", NULL};

static void quit(int sig) {
    (void)sig;
    done = 1;
}

static int comm_delay = 0; // JET delay counter for at-speed

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


int gs2_set_accel_time(modbus_t *mb_ctx, float accel_time) {
    int data = accel_time * 10;
    int r;

    r = modbus_write_register(mb_ctx, GS2_REG_ACCELERATION_TIME_1, data);
    if (r != 1) {
        // Retry, test system always fails first communication
        r = modbus_write_register(mb_ctx, GS2_REG_ACCELERATION_TIME_1, data);
        if (r != 1) {
            fprintf(
                stderr,
                "failed to set register P0x%04x to 0x%04x (%d): %s\n",
                GS2_REG_ACCELERATION_TIME_1,
                data, data,
                strerror(errno)
            );
            return -1;
        }
    }

    return 0;
}


int gs2_set_decel_time(modbus_t *mb_ctx, float decel_time) {
    int data;
    int stop_method;
    int r;

    if (decel_time == 0.0) {
        stop_method = GS2_REG_STOP_METHOD__COAST_TO_STOP;
        decel_time = 20.0;
    } else {
        stop_method = GS2_REG_STOP_METHOD__RAMP_TO_STOP;
    }
    r = modbus_write_register(mb_ctx, GS2_REG_STOP_METHOD, stop_method);
    if (r != 1) {
        fprintf(
            stderr,
            "failed to set register P0x%04x to 0x%04x: %s\n",
            GS2_REG_STOP_METHOD,
            stop_method,
            strerror(errno)
        );
        return -1;
    }

    data = decel_time * 10;
    r = modbus_write_register(mb_ctx, GS2_REG_DECELERATION_TIME_1, data);
    if (r != 1) {
        fprintf(
            stderr,
            "failed to set register P0x%04x to 0x%04x (%d): %s\n",
            GS2_REG_DECELERATION_TIME_1,
            data, data,
            strerror(errno)
        );
        return -1;
    }

    return 0;
}


int gs2_set_braking_resistor(modbus_t *mb_ctx, int braking_resistor) {
    int data;
    int r;

    if (braking_resistor) {
        data = GS2_REG_OVER_VOLTAGE_STALL_PREVENTION__DISABLE;
    } else {
        data = GS2_REG_OVER_VOLTAGE_STALL_PREVENTION__ENABLE;
    }
    r = modbus_write_register(
        mb_ctx,
        GS2_REG_OVER_VOLTAGE_STALL_PREVENTION,
        data
    );
    if (r != 1) {
        fprintf(
            stderr,
            "failed to set register P0x%04x to 0x%04x: %s\n",
            GS2_REG_OVER_VOLTAGE_STALL_PREVENTION,
            data,
            strerror(errno)
        );
        return -1;
    }

    return 0;
}


typedef struct {
    uint8_t param_group, param_number;
    const char *name;
} gs2_reg;

gs2_reg gs2_register[] = {
    { 0x00, 0x00, "Motor Nameplate Voltage" },
    { 0x00, 0x01, "Motor Nameplate Amps" },
    { 0x00, 0x02, "Motor Base Frequency" },
    { 0x00, 0x03, "Motor Base RPM" },
    { 0x00, 0x04, "Motor Max RPM" },
    { 0x01, 0x00, "Stop Method" },
    { 0x01, 0x01, "Acceleration Time 1 (0.1 seconds)" },
    { 0x01, 0x02, "Deceleration Time 1 (0.1 seconds)" },
    { 0x01, 0x03, "Accel S-Curve" },
    { 0x01, 0x04, "Decel S-Curve" },
    { 0x01, 0x05, "Acceleration Time 2 (0.1 seconds)" },
    { 0x01, 0x06, "Deceleration Time 2 (0.1 seconds)" },
    { 0x02, 0x00, "Volts/Hertz Settings" },
    { 0x02, 0x05, "Mid-point Frequency" },
    { 0x02, 0x05, "Mid-point Voltage" },
    { 0x02, 0x06, "Minimum Output Frequency" },
    { 0x02, 0x07, "Minimum Output Voltage" },
    { 0x06, 0x05, "Over-Voltage Stall Prevention" },
    { 0x06, 0x06, "Auto Adjustable Accel/Decel" },
    { 0x09, 0x27, "Firmware Version" },
    { 0x09, 0x29, "GS Series Number" },
    { 0x09, 0x2a, "Manufacturer Model Information" },
    { 0x00, 0x00, NULL }  // NULL name mean "end of list"
};


void gs2_show_config(modbus_t *mb_ctx) {
    gs2_reg *reg;
    int r;

    for (reg = &gs2_register[0]; reg->name != NULL; reg ++) {
        int address;
        uint16_t data;

        address = (reg->param_group << 8) | reg->param_number;

        r = modbus_read_registers(mb_ctx, address, 1, &data);
        if (r != 1) {
            fprintf(
                stderr,
                "failed to read register P%d.%02d (%s)\n",
                reg->param_group,
                reg->param_number,
                reg->name
            );
            return;
        }
        printf(
            "P%d.%02d %s: 0x%04x (%d)\n",
            reg->param_group,
            reg->param_number,
            reg->name,
            data,
            data
        );
    }
}


int write_data(modbus_t *mb_ctx, slavedata_t *slavedata, haldata_t *haldata) {
//  int write_data[MAX_WRITE_REGS];
    int retval;
    hal_float_t hzcalc;

    if (haldata->motor_hz<10)
        haldata->motor_hz = 60;
    if ((haldata->motor_RPM < 600) || (haldata->motor_RPM > 5000))
        haldata->motor_RPM = 1800;
    hzcalc = haldata->motor_hz/haldata->motor_RPM;

    retval = modbus_write_register(
        mb_ctx,
        slavedata->write_reg_start,
        abs((int)(*(haldata->speed_command)*hzcalc*10))
    );

    if (*(haldata->spindle_on) != haldata->old_run) {
        if (*haldata->spindle_on){
            modbus_write_register(mb_ctx, slavedata->write_reg_start+1, 1);
            comm_delay=0;
        }
        else
            modbus_write_register(mb_ctx, slavedata->write_reg_start+1, 0);
        haldata->old_run = *(haldata->spindle_on);
    }
    if (*(haldata->spindle_fwd) != haldata->old_dir) {
        if (*haldata->spindle_fwd)
            modbus_write_register(mb_ctx, slavedata->write_reg_start+2, 0);
        else
            modbus_write_register(mb_ctx, slavedata->write_reg_start+2, 1);
        haldata->old_dir = *(haldata->spindle_fwd);
    }
    if (*(haldata->spindle_fwd) || !(*(haldata->spindle_on)))  // JET turn on and off rev based on the status of fwd
        *(haldata->spindle_rev) = 0;
    if (!(*haldata->spindle_fwd) && *(haldata->spindle_on))
        *(haldata->spindle_rev) = 1;
    if (*(haldata->err_reset) != haldata->old_err_reset) {
        if (*(haldata->err_reset))
            modbus_write_register(mb_ctx, slavedata->write_reg_start+4, 1);
        else
            modbus_write_register(mb_ctx, slavedata->write_reg_start+4, 0);
        haldata->old_err_reset = *(haldata->err_reset);
    }
    if (comm_delay < haldata->ack_delay){ // JET allow time for communications between drive and EMC
        comm_delay++;
    }
    if ((*haldata->spindle_on) && comm_delay == haldata->ack_delay){ // JET test for up to speed
        if ((*(haldata->freq_cmd))==(*(haldata->freq_out)))
            *(haldata->at_speed) = 1;
    }
    if (*(haldata->spindle_on)==0){ // JET reset at-speed
        *(haldata->at_speed) = 0;
    }
    haldata->retval = retval;
    return retval;
}

void usage(int argc, char **argv) {
    (void)argc;
    printf("Usage:  %s [options]\n", argv[0]);
    printf(
    "This is a userspace HAL program, typically loaded using the halcmd \"loadusr\" command:\n"
    "    loadusr gs2_vfd\n"
    "There are several command-line options.  Options that have a set list of possible values may\n"
    "    be set by using any number of characters that are unique.  For example, --rate 5 will use\n"
    "    a baud rate of 57600, since no other available baud rates start with \"5\"\n"
    "-b or --bits <n> (default 8)\n"
    "    Set number of data bits to <n>, where n must be from 5 to 8 inclusive\n"
    "-d or --device <path> (default /dev/ttyS0)\n"
    "    Set the name of the serial device node to use\n"
    "-v or --verbose\n"
    "    Turn on verbose mode.\n"
    "-g or --debug\n"
    "    Turn on debug mode.  This will cause all Modbus messages to be\n"
    "    printed in hex on the terminal.\n"
    "-n or --name <string> (default gs2_vfd)\n"
    "    Set the name of the HAL module.  The HAL comp name will be set to <string>, and all pin\n"
    "    and parameter names will begin with <string>.\n"
    "-p or --parity {even,odd,none} (default odd)\n"
    "    Set serial parity to even, odd, or none.\n"
    "-r or --rate <n> (default 38400)\n"
    "    Set baud rate to <n>.  It is an error if the rate is not one of the following:\n"
    "    110, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200\n"
    "-s or --stopbits {1,2} (default 1)\n"
    "    Set serial stop bits to 1 or 2\n"
    "-t or --target <n> (default 1)\n"
    "    Set MODBUS target (slave) number.  This must match the device number you set on the GS2.\n"
    "-A, --accel-seconds <n>\n"
    "    (default 10.0) Seconds to accelerate the spindle from 0 to Max RPM.\n"
    "-D, --decel-seconds <n>\n"
    "    (default 0.0) Seconds to decelerate the spindle from Max RPM to 0.\n"
    "    If set to 0.0 the spindle will be allowed to coast to a stop without\n"
    "    controlled deceleration.\n"
    "-R, --braking-resistor\n"
    "    This argument should be used when a braking resistor is installed on the\n"
    "    GS2 VFD (see Appendix A of the GS2 manual).  It disables deceleration\n"
    "    over-voltage stall prevention (see GS2 Modbus Parameter 6.05), allowing\n"
    "    the VFD to keep braking even in situations where the motor is regenerating\n"
    "    high voltage.  The regenerated voltage gets safely dumped into the\n"
    "    braking resistor.\n"
    "-X, --disable\n"
    "    Set this flag to disable the control by default (sets default value of 'enable' pin to 0)"
    );
}
int read_data(modbus_t *mb_ctx, slavedata_t *slavedata, haldata_t *hal_data_block) {
    uint16_t receive_data[MODBUS_MAX_READ_REGISTERS];	/* a little padding in there */
    int retval;

    /* can't do anything with a null HAL data block */
    if (hal_data_block == NULL)
        return -1;
    /* but we can signal an error if the other params are null */
    if ((mb_ctx==NULL) || (slavedata == NULL)) {
        hal_data_block->errorcount++;
        return -1;
    }
    retval = modbus_read_registers(mb_ctx, slavedata->read_reg_start,
                                slavedata->read_reg_count, receive_data);
    if (retval==slavedata->read_reg_count) {
        retval = 0;
        hal_data_block->retval = retval;
        *(hal_data_block->stat1) = receive_data[0];
        *(hal_data_block->stat2) = receive_data[1];
        *(hal_data_block->freq_cmd) = receive_data[2] * 0.1;
        *(hal_data_block->freq_out) = receive_data[3] * 0.1;
        if (receive_data[3]==0) {	// JET if freq out is 0 then the drive is stopped
            *(hal_data_block->is_stopped) = 1;
        } else {
            *(hal_data_block->is_stopped) = 0;
        }
        *(hal_data_block->curr_out) = receive_data[4] * 0.1;
        *(hal_data_block->DCBusV) = receive_data[5] * 0.1;
        *(hal_data_block->outV) = receive_data[6] * 0.1;
        *(hal_data_block->RPM) = receive_data[7];
        *(hal_data_block->scale_freq) = (receive_data[8] | (receive_data[9] << 16)) * 0.1;
        *(hal_data_block->power_factor) = receive_data[10];
        *(hal_data_block->load_pct) = receive_data[11] * 0.1;
        *(hal_data_block->FW_Rev) = receive_data[12];
    } else {
        hal_data_block->retval = retval;
        hal_data_block->errorcount++;
        retval = -1;
    }
    return retval;
}

int main(int argc, char **argv)
{
    int retval = 0;
    modbus_t *mb_ctx;
    haldata_t *haldata;
    slavedata_t slavedata;
    int slave;
    int hal_comp_id;
    struct timespec loop_timespec, remaining;
    int baud, bits, stopbits, verbose, debug;
    char *device, *endarg;
    char parity;
    int opt;
    int argindex, argvalue;
    int enabled;

    float accel_time = 10.0;
    float decel_time = 0.0;  // this means: coast to a stop, don't try to control deceleration time
    int braking_resistor = 0;


    done = 0;

    // assume that nothing is specified on the command line
    baud = 38400;
    bits = 8;
    stopbits = 1;
    debug = 0;
    verbose = 0;
    device = "/dev/ttyS0";
    parity = 'O';
    enabled = 1;

    /* slave / register info */
    slave = 1;
    slavedata.read_reg_start = START_REGISTER_R;
    slavedata.read_reg_count = NUM_REGISTERS_R;
    slavedata.write_reg_start = START_REGISTER_W;
    slavedata.write_reg_count = NUM_REGISTERS_W;

    // process command line options
    while ((opt=getopt_long(argc, argv, option_string, long_options, NULL)) != -1) {
        switch(opt) {
            case 'X':  // disable by default on startup
                enabled = 0;
                break;
            case 'b':   // serial data bits, probably should be 8 (and defaults to 8)
                argindex=match_string(optarg, bitstrings);
                if (argindex<0) {
                    printf("gs2_vfd: ERROR: invalid number of bits: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                bits = atoi(bitstrings[argindex]);
                break;
            case 'd':   // device name, default /dev/ttyS0
                // could check the device name here, but we'll leave it to the library open
                if (strlen(optarg) > FILENAME_MAX) {
                    printf("gs2_vfd: ERROR: device node name is too long: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                device = strdup(optarg);
                break;
            case 'g':
                debug = 1;
                break;
            case 'v':
                verbose = 1;
                break;
            case 'n':   // module base name
                if (strlen(optarg) > HAL_NAME_LEN-20) {
                    printf("gs2_vfd: ERROR: HAL module name too long: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                modname = strdup(optarg);
                break;
            case 'p':   // parity, should be a string like "even", "odd", or "none"
                argindex=match_string(optarg, paritystrings);
                if (argindex<0) {
                    printf("gs2_vfd: ERROR: invalid parity: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                parity = paritychars[argindex];
                break;
            case 'r':   // Baud rate, 38400 default
                argindex=match_string(optarg, ratestrings);
                if (argindex<0) {
                    printf("gs2_vfd: ERROR: invalid baud rate: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                baud = atoi(ratestrings[argindex]);
                break;
            case 's':   // stop bits, defaults to 1
                argindex=match_string(optarg, stopstrings);
                if (argindex<0) {
                    printf("gs2_vfd: ERROR: invalid number of stop bits: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                stopbits = atoi(stopstrings[argindex]);
                break;
            case 't':   // target number (MODBUS ID), default 1
                argvalue = strtol(optarg, &endarg, 10);
                if ((*endarg != '\0') || (argvalue < 1) || (argvalue > 254)) {
                    printf("gs2_vfd: ERROR: invalid slave number: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                slave = argvalue;
                break;
            case 'A':
                accel_time = strtof(optarg, &endarg);
                if (*endarg != '\0') {
                    printf("gs2_vfd: ERROR: invalid acceleration time: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                break;
            case 'D':
                decel_time = strtof(optarg, &endarg);
                if (*endarg != '\0') {
                    printf("gs2_vfd: ERROR: invalid deceleration time: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                break;
            case 'R':
                braking_resistor = 1;
                break;
            case 'h':
            default:
                usage(argc, argv);
                exit(0);
                break;
        }
    }

    printf("%s: device='%s', baud=%d, parity='%c', bits=%d, stopbits=%d, address=%d, enabled=%d\n",
           modname, device, baud, parity, bits, stopbits, slave, enabled);
    /* point TERM and INT signals at our quit function */
    /* if a signal is received between here and the main loop, it should prevent
            some initialization from happening */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);

    /* Assume 38.4k O-8-1 serial settings, device 1 */
    mb_ctx = modbus_new_rtu(device, baud, parity, bits, stopbits);
    if (mb_ctx == NULL) {
        printf("%s: ERROR: couldn't open Modbus serial device: %s\n", modname, modbus_strerror(errno));
        goto out_noclose;
    }

    /* the open has got to work, or we're out of business */
    if (((retval = modbus_connect(mb_ctx))!=0) || done) {
        printf("%s: ERROR: couldn't open serial device: %s\n", modname, modbus_strerror(errno));
        goto out_noclose;
    }

    modbus_set_debug(mb_ctx, debug);

    modbus_set_slave(mb_ctx, slave);

    // show the gs2 vfd configuration
    if (verbose) {
        gs2_show_config(mb_ctx);
    }

    /* create HAL component */
    hal_comp_id = hal_init(modname);
    if ((hal_comp_id < 0) || done) {
        printf("%s: ERROR: hal_init failed\n", modname);
        retval = hal_comp_id;
        goto out_close;
    }

    /* grab some shmem to store the HAL data in */
    haldata = (haldata_t *)hal_malloc(sizeof(haldata_t));
    if ((haldata == 0) || done) {
        printf("%s: ERROR: unable to allocate shared memory\n", modname);
        retval = -1;
        goto out_close;
    }

    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->stat1), hal_comp_id, "%s.status-1", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->stat2), hal_comp_id, "%s.status-2", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->freq_cmd), hal_comp_id, "%s.frequency-command", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->freq_out), hal_comp_id, "%s.frequency-out", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->curr_out), hal_comp_id, "%s.output-current", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->DCBusV), hal_comp_id, "%s.DC-bus-volts", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->outV), hal_comp_id, "%s.output-voltage", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->RPM), hal_comp_id, "%s.motor-RPM", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->scale_freq), hal_comp_id, "%s.scale-frequency", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->power_factor), hal_comp_id, "%s.power-factor", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_OUT, &(haldata->load_pct), hal_comp_id, "%s.load-percentage", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_s32_newf(HAL_OUT, &(haldata->FW_Rev), hal_comp_id, "%s.firmware-revision", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_param_s32_newf(HAL_RW, &(haldata->errorcount), hal_comp_id, "%s.error-count", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_param_float_newf(HAL_RW, &(haldata->looptime), hal_comp_id, "%s.loop-time", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_param_s32_newf(HAL_RW, &(haldata->retval), hal_comp_id, "%s.retval", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->at_speed), hal_comp_id, "%s.at-speed", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->is_stopped), hal_comp_id, "%s.is-stopped", modname); // JET
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_float_newf(HAL_IN, &(haldata->speed_command), hal_comp_id, "%s.speed-command", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->spindle_on), hal_comp_id, "%s.spindle-on", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->spindle_fwd), hal_comp_id, "%s.spindle-fwd", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->spindle_rev), hal_comp_id, "%s.spindle-rev", modname); //JET
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->err_reset), hal_comp_id, "%s.err-reset", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_param_float_newf(HAL_RW, &(haldata->speed_tolerance), hal_comp_id, "%s.tolerance", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_param_float_newf(HAL_RW, &(haldata->motor_hz), hal_comp_id, "%s.nameplate-HZ", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_param_float_newf(HAL_RW, &(haldata->motor_RPM), hal_comp_id, "%s.nameplate-RPM", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_param_s32_newf(HAL_RW, &(haldata->ack_delay), hal_comp_id, "%s.ack-delay", modname);
    if (retval!=0) goto out_closeHAL;
    /* define run (enable) pin and isInitialized */
    retval = hal_pin_bit_newf(HAL_IN, &(haldata->ena_gs2comp), hal_comp_id, "%s.enable", modname);
    if (retval!=0) goto out_closeHAL;
    retval = hal_pin_bit_newf(HAL_OUT, &(haldata->isInitialized), hal_comp_id, "%s.initialized", modname);
    if (retval!=0) goto out_closeHAL;

    /* make default data match what we expect to use */
    *(haldata->stat1) = 0;
    *(haldata->stat2) = 0;
    *(haldata->freq_cmd) = 0;
    *(haldata->freq_out) = 0;
    *(haldata->curr_out) = 0;
    *(haldata->DCBusV) = 0;
    *(haldata->outV) = 0;
    *(haldata->RPM) = 0;
    *(haldata->scale_freq) = 0;
    *(haldata->power_factor) = 0;
    *(haldata->load_pct) = 0;
    *(haldata->FW_Rev) = 0;
    haldata->errorcount = 0;
    haldata->looptime = 0.1;
    haldata->motor_RPM = 1730;
    haldata->motor_hz = 60;
    haldata->speed_tolerance = 0.01;
    haldata->ack_delay = 2;
    *(haldata->err_reset) = 0;
    *(haldata->spindle_on) = 0;
    *(haldata->spindle_fwd) = 1;
    *(haldata->spindle_rev) = 0;
    haldata->old_run = -1;		// make sure the initial value gets output
    haldata->old_dir = -1;
    haldata->old_err_reset = -1;
    *(haldata->ena_gs2comp) = enabled;  // command line override, defaults to "enabled" for compatibility
    *(haldata->isInitialized) = 0;

    // Activate HAL component
    hal_ready(hal_comp_id);

    /* here's the meat of the program.  loop until done (which may be never) */
    while (done==0) {

        /* don't want to scan too fast, and shouldn't delay more than a few seconds */
        if (haldata->looptime < 0.001) haldata->looptime = 0.001;
        if (haldata->looptime > 2.0) haldata->looptime = 2.0;
        loop_timespec.tv_sec = (time_t)(haldata->looptime);
        loop_timespec.tv_nsec = (long)((haldata->looptime - loop_timespec.tv_sec) * 1000000000l);
        nanosleep(&loop_timespec, &remaining);

        if(*(haldata->ena_gs2comp) == 0) {
             // Component not enabled, so do nothing and force uninitialized state
             if (*(haldata->isInitialized)) {
                *(haldata->spindle_on) = 0;
                // need to write to vfd in case we are here when it is being disabled
                write_data(mb_ctx, &slavedata, haldata);
                // debug printf below
                // printf("GS2: Disabling\n");
            }
            *(haldata->isInitialized) = 0;
        } else if (!*(haldata->isInitialized)) {
            // Initialize: configure the gs2 vfd based on command-line arguments
            if (gs2_set_accel_time(mb_ctx, accel_time) != 0) {
                continue;
            }
            if (gs2_set_decel_time(mb_ctx, decel_time) != 0) {
                continue;
            }
            if (gs2_set_braking_resistor(mb_ctx, braking_resistor) != 0) {
                continue;
            }
            // debug printf below
            // printf("GS2: Initialized\n");
            *(haldata->isInitialized) = 1;
        } else {
            // Enabled and initialized, so do read/write of Modbus
            read_data(mb_ctx, &slavedata, haldata);
            write_data(mb_ctx, &slavedata, haldata);
        }
    }

    retval = 0;	/* if we get here, then everything is fine, so just clean up and exit */
out_closeHAL:
    hal_exit(hal_comp_id);
out_close:
    modbus_close(mb_ctx);
    modbus_free(mb_ctx);
out_noclose:
    return retval;
}
