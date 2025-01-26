/********************************************************************
* Description: emcrsh.cc  (source for linuxcncrsh)
*   Extended telnet based LinuxCNC interface
*
*   Derived from a work by Fred Proctor & Will Shackleford
*   Further derived from work by jmkasunich
*
* Author: Eric H. Johnson
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2006-2008 All rights reserved.
*
* Last change:
********************************************************************/

#define _REENTRANT

#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <getopt.h>

#include "emcglb.h"		// EMC_NMLFILE, TRAJ_MAX_VELOCITY, etc.
#include "inifile.hh"		// INIFILE
#include "rcs_print.hh"
#include "timer.hh"             // etime()
#include "shcom.hh"             // NML Messaging functions
#include <rtapi_string.h>

/*
  Using linuxcncrsh:

  linuxcncrsh {-- --port <port number> --name <server name> --connectpw <password>
             --enablepw <password> --sessions <max sessions> --path <path>
             -ini<INI file>}

  With -- --port Waits for socket connections (Telnet) on specified socket, without port
            uses default port 5007.
  With -- --name <server name> Sets the server name to specified name for Hello.
  With -- --connectpw <password> Sets the connection password to 'password'. Default EMC
  With -- --enablepw <password> Sets the enable password to 'password'. Default EMCTOO
  With -- --sessions <max sessions> Sets the maximum number of simultaneous connextions
            to max sessions. Default is no limit (-1).
  With -- --path Sets the base path to program (G-Code) files, default is "../../nc_files/".
            Make sure to include the final slash (/).
  With -- -ini <INI file>, uses specified INI file instead of default emc.ini.

  There are six commands supported, Where the commands set and get contain LinuxCNC
  specific sub-commands based on the commands supported by linuxcncrsh, but where the
  usual prefix ( "emc_") is omitted. Commands and most parameters are not case sensitive.
  The exceptions are passwords, file paths and text strings.

  The supported commands are as follows:

  ==> HELLO <==

  Hello <password> <client> <version>
  If a valid password was entered the server will respond with

  HELLO ACK <Server Name> <Server Version>

  Where server name and server version are looked up from the implementation.
  if an invalid password or any other syntax error occurs then the server
  responds with:

  HELLO NAK

  ==> Get <==

  The get command includes one of the LinuxCNC sub-commands, described below and
  zero or more additional parameters.

  ==> Set <==

  The set command inclides one of the LinuxCNC sub-commands, described below and
  one or more additional parameters.

  ==> Quit <==

  The quit command disconnects the associated socket connection.

  ==> Shutdown <==

  The shutdown command tells LinuxCNC to shutdown before quitting the connection. This
  command may only be issued if the Hello has been successfully negotiated and the
  connection has control of the CNC (see enable sub-command below). This command
  has no parameters.

  ==> Help <==

  The help command will return help information in text format over the telnet
  connection. If no parameters are specified, it will itemize the available commands.
  If a command is specified, it will provide usage information for the specified
  command. Help will respond regardless of whether a "Hello" has been
  successfully negotiated.


  LinuxCNC sub-commands:

  echo on | off
  With get will return the current echo state, with set, sets the echo
  state. When echo is on, all commands will be echoed upon receipt. This
  state is local to each connection.

  verbose on | off
  With get will return the current verbose state, with set, sets the
  verbose state. When in verbose mode is on, all set commands return
  positive acknowledgement in the form SET <COMMAND> ACK. In addition,
  text error messages will be issued when in verbose mode. This state
  is local to each connection.

  enable <pwd> | off
  With get will return On or Off to indicate whether the current connection
  is enabled to perform control functions. With set and a valid password,
  the current connection is enabled for control functions. "OFF" may not
  be used as a password and disables control functions for this connection.

  config [TBD]

  comm_mode ascii | binary
  With get, will return the current communications mode. With set, will
  set the communications mode to the specified mode. The binary protocol
  is TBD.

  comm_prot <version no>
  With get, returns the current protocol version used by the server,
  with set, sets the server to use the specified protocol version,
  provided it is lower than or equal to the highest version number
  supported by the server implementation.

  INIFILE
  Returns the path and file name of the current configuration inifile.

  plat
  Returns the platform for which this was compiled, e.g., linux_2_0_36

  ini <var> <section>
  Returns the string value of <var> in section <section>, in EMC_INIFILE

  debug {<new value>}
  With get, returns the integer value of EMC_DEBUG, in LinuxCNC. Note that
  it may not be true that the local EMC_DEBUG variable here (in linuxcncrsh and
  the GUIs that use it) is the same as the EMC_DEBUG value in the LinuxCNC. This
  can happen if LinuxCNC is started from one INI file, and the GUI is started
  with another that has a different value for DEBUG.
  With set, sends a command to the LinuxCNC to set the new debug level,
  and sets the EMC_DEBUG global here to the same value. This will make
  the two values the same, since they really ought to be the same.

  wait_mode none | received | done
  Set the wait for commands to return to be right away (none), after the
  command was sent and received (received), or after the command was
  done (done).

  wait received | done
  Force a wait for the previous command to be received, or done. This lets
  you wait in the event that "wait_mode none" is in effect.

  set_timeout <timeout>
  Set the timeout for commands to return to <timeout>, in seconds. Timeout
  is a real number. If it's <= 0.0, it means wait forever. Default is 0.0,
  wait forever.

  update (none) | none | auto
  With no arg, forces an update of the LinuxCNC status. With "none", doesn't
  cause an automatic update of status with other emc_ words. With "auto",
  makes emc_ words automatically update status before they return values.

  error
  Returns the current LinuxCNC error string, or "ok" if no error.

  operator_display
  Returns the current LinuxCNC operator display string, or "ok" if none.

  operator_text
  Returns the current LinuxCNC operator text string, or "ok" if none.

  time
  Returns the time, in seconds, from the start of the epoch. This starting
  time depends on the platform.

  estop (none) | on | off
  With no arg, returns the estop setting as "on" or "off". Otherwise,
  sends an estop on or off command.

  machine (none) | on | off
  With no arg, returns the machine setting as "on" or "off". Otherwise,
  sends a machine on or off command.

  mode (none) | manual | auto | mdi
  With no arg, returns the mode setting as "manual", "auto", or "mdi".
  Otherwise, sends a mode manual, auto, or mdi command.

  mist (none) | on | off
  With no arg, returns the mist setting as "on" or "off". Otherwise,
  sends a mist on or off command.

  flood (none) | on | off
  With no arg, returns the flood setting as "on" or "off". Otherwise,
  sends a flood on or off command.

  spindle (none) | forward | reverse | increase | decrease | constant | off
  With no arg, returns the value of the spindle state as "forward",
  "reverse", "increase", "decrease", or "off". With arg, sends the spindle
  command. Note that "increase" and "decrease" will cause a speed change in
  the corresponding direction until a "constant" command is sent.

  brake (none) | on | off
  With no arg, returns the brake setting. Otherwise sets the brake.

  tool
  Returns the id of the currently loaded tool

  tool_offset
  Returns the currently applied tool length offset

  load_tool_table <file>
  Loads the tool table specified by <file>

  home 0 | 1 | 2 | ...
  Homes the indicated joint.

  jog_stop joint_number|axis_letter
  Stop the axis or joint jog

  jog joint_number|axis_letter speed
  Jog the indicated joint or axis at <speed>; sign of speed is direction

  jog_incr joint_number|axis_letter <speed> <incr>
  Jog the indicated joint or axis by increment <incr> at the <speed>; sign of
  speed is direction

  feed_override {<percent>}
  With no args, returns the current feed override, as a percent. With
  argument, set the feed override to be the percent value

  spindle_override {<percent>}
  With no args, returns the current spindle override, as a percent. With
  argument, set the spindle override to be the percent value

  abs_cmd_pos 0 | 1 | ...
  Returns double obj containing the XYZ-SXYZ commanded pos in abs coords,
  at given index, 0 = X, etc.

  abs_act_pos
  Returns double objs containing the XYZ-SXYZ actual pos in abs coords

  rel_cmd_pos 0 | 1 | ...
  Returns double obj containing the XYZ-SXYZ commanded pos in rel coords,
  at given index, 0 = X, etc., including tool length offset

  rel_act_pos
  Returns double objs containing the XYZ-SXYZ actual pos in rel coords,
  including tool length offset

  joint_pos
  Returns double objs containing the actual pos in absolute coords of individual
  joint/slider positions, excludes tool length offset

  pos_offset X | Y | Z | R | P | W
  Returns the position offset associated with the world coordinate provided

  joint_limit 0 | 1 | ...
  Returns "ok", "minsoft", "minhard", "maxsoft", "maxhard"

  joint_fault 0 | 1 | ...
  Returns "ok" or "fault"

  joint_homed 0 | 1 | ...
  Returns "homed", "not"

  mdi <string>
  Sends the <string> as an MDI command

  task_plan_init
  Initializes the program interpreter

  open <filename>
  Opens the named file

  run {<start line>}
  Without start line, runs the opened program from the beginning. With
  start line, runs from that line. A start line of -1 runs in verify mode.

  pause
  Pause program execution

  resume
  Resume program execution

  step
  Step the program one line

  program
  Returns the name of the currently opened program, or "none"

  program_line
  Returns the currently executing line of the program

  program_status
  Returns "idle", "running", or "paused"

  program_codes
  Returns the string for the currently active program codes

  joint_type <joint>
  Returns "linear", "angular", or "custom" for the type of the specified joint

  joint_units <joint>
  Returns "inch", "mm", "cm", or "deg", "rad", "grad", or "custom",
  for the corresponding native units of the specified axis. The type
  of the axis (linear or angular) is used to resolve which type of units
  are returned. The units are obtained heuristically, based on the
  EMC_AXIS_STAT::units numerical value of user units per mm or deg.
  For linear joints, something close to 0.03937 is deemed "inch",
  1.000 is "mm", 0.1 is "cm", otherwise it's "custom".
  For angular joints, something close to 1.000 is deemed "deg",
  PI/180 is "rad", 100/90 is "grad", otherwise it's "custom".

  program_units
  program_linear_units
  Returns "inch", "mm", "cm", or "none", for the corresponding linear
  units that are active in the program interpreter.

  program_angular_units
  Returns "deg", "rad", "grad", or "none" for the corresponding angular
  units that are active in the program interpreter.

  user_linear_units
  Returns "inch", "mm", "cm", or "custom", for the
  corresponding native user linear units of the LinuxCNC trajectory
  level. This is obtained heuristically, based on the
  EMC_TRAJ_STAT::linearUnits numerical value of user units per mm.
  Something close to 0.03937 is deemed "inch", 1.000 is "mm", 0.1 is
  "cm", otherwise it's "custom".

  user_angular_units
  Returns "deg", "rad", "grad", or "custom" for the corresponding native
  user angular units of the LinuxCNC trajectory level. Like with linear units,
  this is obtained heuristically.

  display_linear_units
  display_angular_units
  Returns "inch", "mm", "cm", or "deg", "rad", "grad", or "custom",
  for the linear or angular units that are active in the display.
  This is effectively the value of linearUnitConversion or
  angularUnitConversion, resp.

  linear_unit_conversion {inch | mm | cm | auto}
  With no args, returns the unit conversion active. With arg, sets the
  units to be displayed. If it's "auto", the units to be displayed match
  the program units.

  angular_unit_conversion {deg | rad | grad | auto}
  With no args, returns the unit conversion active. With arg, sets the
  units to be displayed. If it's "auto", the units to be displayed match
  the program units.

  probe_clear
  Clear the probe tripped flag.

  probe_tripped
  Has the probe been tripped since the last clear.

  probe_value
  Value of current probe signal. (read-only)

  probe
  Move toward a certain location. If the probe is tripped on the way stop
  motion, record the position and raise the probe tripped flag.

  teleop_enable
  Should motion run in teleop mode? (No args
  gets it, one arg sets it.)

  kinematics_type
  returns the type of kinematics functions used identity=1, serial=2,
  parallel=3, custom=4

  override_limits on | off
  If parameter is on, disables end of travel hardware limits to allow
  jogging off of a limit. If parameters is off, then hardware limits
  are enabled.

  optional_stop  none | 0 | 1
  returns state of optional setop, sets it or deactivates it (used to stop/continue on M1)

  <------------------------------------------------>

  To Do:

  1> Load / save connect and enable passwords to file.
  2> Implement commands to set / get passwords
  3> Get enable to tell peer connections which connection has control.
  4> Get shutdown to notify LinuxCNC to actually shutdown.
*/

// EMC_STAT *emcStatus;

typedef enum {
  cmdHello, cmdSet, cmdGet, cmdQuit, cmdShutdown, cmdHelp, cmdUnknown} cmdType;

typedef enum {
  scEcho, scVerbose, scEnable, scConfig, scCommMode, scCommProt, scIniFile,
  scPlat, scIni, scDebug, scWaitMode, scWait, scSetTimeout, scUpdate, scError,
  scOperatorDisplay, scOperatorText, scTime, scEStop, scMachine, scMode,
  scMist, scFlood, scSpindle, scBrake, scTool, scToolOffset,
  scLoadToolTable, scHome, scJogStop, scJog, scJogIncr, scFeedOverride,
  scAbsCmdPos, scAbsActPos, scRelCmdPos, scRelActPos, scJointPos, scPosOffset,
  scJointLimit, scJointFault, scJointHomed, scMDI, scTskPlanInit, scOpen, scRun,
  scPause, scResume, scStep, scAbort, scProgram, scProgramLine, scProgramStatus,
  scProgramCodes, scJointType, scJointUnits, scProgramUnits, scProgramLinearUnits,
  scProgramAngularUnits, scUserLinearUnits, scUserAngularUnits, scDisplayLinearUnits,
  scDisplayAngularUnits, scLinearUnitConversion,  scAngularUnitConversion, scProbeClear,
  scProbeTripped, scProbeValue, scProbe, scTeleopEnable, scKinematicsType, scOverrideLimits,
  scSpindleOverride, scOptionalStop, scSetWait, scUnknown
  } cmdTokenType;

typedef enum {
  rtNoError, rtHandledNoError, rtStandardError, rtCustomError, rtCustomHandledError
  } cmdResponseType;

typedef struct {
  int cliSock;
  char hostName[80];
  char version[8];
  bool linked;
  bool echo;
  bool verbose;
  bool enabled;
  int commMode;
  int commProt;
  char inBuf[256];
  char outBuf[4096];
  char progName[PATH_MAX];} connectionRecType;

int port = 5007;
int server_sockfd;
socklen_t server_len, client_len;
struct sockaddr_in server_address;
struct sockaddr_in client_address;
bool useSockets = true;
int tokenIdx;
const char *delims = " \n\r\0";
int enabledConn = -1;
char pwd[16] = "EMC\0";
char enablePWD[16] = "EMCTOO\0";
char serverName[24] = "EMCNETSVR\0";
int sessions = 0;
int maxSessions = -1;

const char *cmdTokens[] = {
  "ECHO", "VERBOSE", "ENABLE", "CONFIG", "COMM_MODE", "COMM_PROT", "INIFILE", "PLAT", "INI", "DEBUG",
  "WAIT_MODE", "WAIT", "TIMEOUT", "UPDATE", "ERROR", "OPERATOR_DISPLAY", "OPERATOR_TEXT",
  "TIME", "ESTOP", "MACHINE", "MODE", "MIST", "FLOOD",
  "SPINDLE", "BRAKE", "TOOL", "TOOL_OFFSET", "LOAD_TOOL_TABLE", "HOME",
  "JOG_STOP", "JOG", "JOG_INCR", "FEED_OVERRIDE", "ABS_CMD_POS", "ABS_ACT_POS",
  "REL_CMD_POS", "REL_ACT_POS", "JOINT_POS", "POS_OFFSET", "JOINT_LIMIT",
  "JOINT_FAULT", "JOINT_HOMED", "MDI", "TASK_PLAN_INIT", "OPEN", "RUN", "PAUSE",
  "RESUME", "STEP", "ABORT", "PROGRAM", "PROGRAM_LINE", "PROGRAM_STATUS", "PROGRAM_CODES",
  "JOINT_TYPE", "JOINT_UNITS", "PROGRAM_UNITS", "PROGRAM_LINEAR_UNITS", "PROGRAM_ANGULAR_UNITS",
  "USER_LINEAR_UNITS", "USER_ANGULAR_UNITS", "DISPLAY_LINEAR_UNITS", "DISPLAY_ANGULAR_UNITS",
  "LINEAR_UNIT_CONVERSION", "ANGULAR_UNIT_CONVERSION", "PROBE_CLEAR", "PROBE_TRIPPED",
  "PROBE_VALUE", "PROBE", "TELEOP_ENABLE", "KINEMATICS_TYPE", "OVERRIDE_LIMITS",
  "SPINDLE_OVERRIDE", "OPTIONAL_STOP", "SET_WAIT", ""};

const char *commands[] = {"HELLO", "SET", "GET", "QUIT", "SHUTDOWN", "HELP", ""};

struct option longopts[] = {
  {"help", 0, NULL, 'h'},
  {"port", 1, NULL, 'p'},
  {"name", 1, NULL, 'n'},
  {"sessions", 1, NULL, 's'},
  {"connectpw", 1, NULL, 'w'},
  {"enablepw", 1, NULL, 'e'},
  {"path", 1, NULL, 'd'},
  {0,0,0,0}};

/* format string to outputbuffer (will be presented to user as result of command) */
#define OUT(...) snprintf(context->outBuf, sizeof(context->outBuf), __VA_ARGS__)


static void thisQuit()
{
    EMC_NULL emc_null_msg;

    if (emcStatusBuffer != 0) {
	// wait until current message has been received
	emcCommandWaitReceived();
    }

    // clean up NML buffers

    if (emcErrorBuffer != 0) {
	delete emcErrorBuffer;
	emcErrorBuffer = 0;
    }

    if (emcStatusBuffer != 0) {
	delete emcStatusBuffer;
	emcStatusBuffer = 0;
	emcStatus = 0;
    }

    if (emcCommandBuffer != 0) {
	delete emcCommandBuffer;
	emcCommandBuffer = 0;
    }

    exit(0);
}

static int initSocket()
{
  int optval = 1;
  int err;

  server_sockfd = socket(AF_INET, SOCK_STREAM, 0);
  setsockopt(server_sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
  server_address.sin_family = AF_INET;
  server_address.sin_addr.s_addr = htonl(INADDR_ANY);
  server_address.sin_port = htons(port);
  server_len = sizeof(server_address);
  err = bind(server_sockfd, (struct sockaddr *)&server_address, server_len);
  if (err) {
      rcs_print_error("error initializing sockets: %s\n", strerror(errno));
      return err;
  }

  err = listen(server_sockfd, 5);
  if (err) {
      rcs_print_error("error listening on socket: %s\n", strerror(errno));
      return err;
  }

  // ignore SIGCHLD
  {
    struct sigaction act;
    act.sa_handler = SIG_IGN;
    sigemptyset(&act.sa_mask);
    act.sa_flags = 0;
    sigaction(SIGCHLD, &act, NULL);
  }

  return 0;
}

static void sigQuit(int /*sig*/)
{
    thisQuit();
}

static cmdTokenType lookupCommandToken(char *s)
{
  for (long unsigned int i = scEcho; i < sizeof(cmdTokens)/sizeof(char *); ++i) {
    if (strcmp(cmdTokens[i], s) == 0) return (cmdTokenType) i;
  }
  return scUnknown;
}

// initiate session
static int commandHello(connectionRecType *context)
{
  // get password token
  char *s = strtok(NULL, delims);
  if (!s) return -1;
  if (strcmp(s, pwd) != 0) return -1;

  // get announced client name
  s = strtok(NULL, delims);
  if (!s) return -1;
  if(rtapi_strlcpy(context->hostName, s, sizeof(context->hostName)) >= sizeof(context->hostName)) {
    return -1;
  }

  // get version string
  s = strtok(NULL, delims);
  if (!s) return -1;
  if(rtapi_strlcpy(context->version, s, sizeof(context->version)) >= sizeof(context->version)) {
    return -1;
  }

  // mark context as connected
  context->linked = true;
  printf("Connected to %s\n", context->hostName);
  return 0;
}


static int checkOnOff(char *s)
{
  if (!s) return -1;
  strupr(s);
  if (strcmp(s, "ON") == 0) return 0;
  if (strcmp(s, "OFF") == 0) return 1;
  return -1;
}

static int checkBinaryASCII(char *s)
{
  if (!s) return -1;
  strupr(s);
  if (strcmp(s, "ASCII") == 0) return 0;
  if (strcmp(s, "BINARY") == 0) return 1;
  return -1;
}

static int checkReceivedDoneNone(char *s)
{
  if (!s) return -1;
  strupr(s);
  if (strcmp(s, "RECEIVED") == 0) return 0;
  if (strcmp(s, "DONE") == 0) return 1;
  if (strcmp(s, "NONE") == 0) return 2;
  return -1;
}

static int checkNoneAuto(char *s)
{
  if (!s) return -1;
  strupr(s);
  if (strcmp(s, "NONE") == 0) return 0;
  if (strcmp(s, "AUTO") == 0) return 1;
  return -1;
}

static int checkManualAutoMDI(char *s)
{
  if (!s) return -1;
  strupr(s);
  if (strcmp(s, "MANUAL") == 0) return 0;
  if (strcmp(s, "AUTO") == 0) return 1;
  if (strcmp(s, "MDI") == 0) return 2;
  return -1;
}

static int checkSpindleCmd(char *s)
{
  if (!s) return -1;
  strupr(s);
  if (strcmp(s, "FORWARD") == 0) return 0;
  if (strcmp(s, "REVERSE") == 0) return 1;
  if (strcmp(s, "INCREASE") == 0) return 2;
  if (strcmp(s, "DECREASE") == 0) return 3;
  if (strcmp(s, "CONSTANT") == 0) return 4;
  if (strcmp(s, "OFF") == 0) return 5;
  return -1;
}

static int checkConversionStr(char *s)
{
  if (!s) return -1;
  strupr(s);
  if (strcmp(s, "INCH") == 0) return 0;
  if (strcmp(s, "MM") == 0) return 1;
  if (strcmp(s, "CM") == 0) return 2;
  if (strcmp(s, "AUTO") == 0) return 3;
  if (strcmp(s, "CUSTOM") == 0) return 4;
  return -1;
}

static int checkAngularConversionStr(char *s)
{
  if (!s) return -1;
  strupr(s);
  if (strcmp(s, "DEG") == 0) return 0;
  if (strcmp(s, "RAD") == 0) return 1;
  if (strcmp(s, "GRAD") == 0) return 2;
  if (strcmp(s, "AUTO") == 0) return 3;
  if (strcmp(s, "CUSTOM") == 0) return 4;
  return -1;
}

static cmdResponseType setEcho(connectionRecType *context)
{
 
   char *s = strtok(NULL, delims);
   switch (checkOnOff(s)) {
     case -1: return rtStandardError;
     case 0: context->echo = true; break;
     case 1: context->echo = false;
     }
   return rtNoError;
}

static cmdResponseType setVerbose(connectionRecType *context)
{
 
   char *s = strtok(NULL, delims);
   switch (checkOnOff(s)) {
     case -1: return rtStandardError;
     case 0: context->verbose = true; break;
     case 1: context->verbose = false;
     }
   return rtNoError;
}

static cmdResponseType setEnable(connectionRecType *context)
{

   char *s = strtok(NULL, delims);
   if (s && (strcmp(s, enablePWD) == 0) ) {
     enabledConn = context->cliSock;
     context->enabled = true;
     return rtNoError;
     }
   else
     if (checkOnOff(s) == 1) {
       context->enabled = false;
       enabledConn = -1;
       return rtNoError;
       }
     else return rtStandardError;
}

static cmdResponseType setConfig(connectionRecType *context)
{
  OUT("SET CONFIG not implemented");
  return rtCustomError;
}

static cmdResponseType setCommMode(connectionRecType *context)
{
  int ret;

  char *s = strtok(NULL, delims);
  ret = checkBinaryASCII(s);
  if (ret == -1) return rtStandardError;
  context->commMode = ret;
  return rtNoError;
}

static cmdResponseType setCommProt(connectionRecType *context)
{
  char *pVersion = strtok(NULL, delims);
  if (!pVersion) return rtStandardError;
  rtapi_strxcpy(context->version, pVersion);
  return rtNoError;
}

static cmdResponseType setDebug(connectionRecType * /*context*/)
{
  char *pLevel;
  int level;

  pLevel = strtok(NULL, delims);
  if (!pLevel) return rtStandardError;
  if (sscanf(pLevel, "%i", &level) < 1) return rtStandardError;
  else {
      if(sendDebug(level) != 0) return rtStandardError;
  }
  return rtNoError;
}

static cmdResponseType setWaitMode(connectionRecType *context)
{
  char *s = strtok(NULL, delims);
  switch (checkReceivedDoneNone(s)) {
    case -1: return rtStandardError;
    case 0: {
      emcWaitType = EMC_WAIT_RECEIVED;
      break;
    }
    case 1: {
      emcWaitType = EMC_WAIT_DONE;
      break;
    }
    case 2: {
      OUT("linuxcncrsh: 'set wait_mode' asked for 'NONE', but that setting has been removed as it may cause commands to be lost");
      return rtCustomError;
    }
  }
  return rtNoError;
}

/* compatibility wrapper to deprecate set_wait command token - @todo remove at some point */
static cmdResponseType setSetWait(connectionRecType *context)
{
  dprintf(context->cliSock, "WARNING: \"set_wait\" command is depreciated and will be removed in the future. Please use \"wait_mode\" instead.\n");
  return setWaitMode(context);
}

static cmdResponseType setMachine(connectionRecType * /*context*/)
{
   char *s = strtok(NULL, delims);
   switch (checkOnOff(s)) {
     case -1: return rtStandardError;
     case 0:
        if(sendMachineOn() != 0) return rtStandardError;
        break;
     case 1:
        if(sendMachineOff() != 0) return rtStandardError;
        break;
     }
   return rtNoError;
}

static cmdResponseType setEStop(connectionRecType * /*context*/)
{
    char *state = strtok(NULL, delims);
    switch (checkOnOff(state)) {
        case 0:
            if(sendEstop() != 0) return rtStandardError;
            break;

        case 1:
            if(sendEstopReset() != 0) return rtStandardError;
            break;

        default:
            return rtStandardError;
    }

    return rtNoError;
}

static cmdResponseType setWait(connectionRecType * /*context*/)
{
  char *s = strtok(NULL, delims);
  switch (checkReceivedDoneNone(s)) {
    case -1: return rtStandardError;
    case 0:
      if (emcCommandWaitReceived() != 0) return rtStandardError;
      break;
    case 1:
      if (emcCommandWaitDone() != 0) return rtStandardError;
      break;
    case 2: ;
    default: return rtStandardError;
    }
  return rtNoError;
}

static cmdResponseType setTimeout(connectionRecType * /*context*/)
{
  float Timeout;

  char *s = strtok(NULL, delims);
  if (!s) return rtStandardError;
  if (sscanf(s, "%f", &Timeout) < 1) return rtStandardError;
  emcTimeout = Timeout;
  return rtNoError;
}

static cmdResponseType setUpdate(connectionRecType * /*context*/)
{
  char *s = strtok(NULL, delims);
  switch (checkNoneAuto(s)) {
    case 0: emcUpdateType = EMC_UPDATE_NONE; break;
    case 1: emcUpdateType = EMC_UPDATE_AUTO; break;
    default: return rtStandardError;
    }
  return rtNoError;
}

static cmdResponseType setMode(connectionRecType * /*context*/)
{
  char *s = strtok(NULL, delims);
  switch (checkManualAutoMDI(s)) {
    case 0:
        if(sendManual() != 0) return rtStandardError;
        break;
    case 1:
        if(sendAuto() != 0) return rtStandardError;
        break;
    case 2:
        if(sendMdi() != 0) return rtStandardError;
        break;
    default: return rtStandardError;
    }
  return rtNoError;
}

static cmdResponseType setMist(connectionRecType * /*context*/)
{
   char *s = strtok(NULL, delims);
   switch (checkOnOff(s)) {
     case -1: return rtStandardError;
     case 0:
        if(sendMistOn() != 0) return rtStandardError;
        break;
     case 1:
        if(sendMistOff() != 0) return rtStandardError;
        break;
     }
   return rtNoError;
}

static cmdResponseType setFlood(connectionRecType * /*context*/)
{
   char *s = strtok(NULL, delims);
   switch (checkOnOff(s)) {
     case -1: return rtStandardError;
     case 0:
        if(sendFloodOn() != 0) return rtStandardError;
        break;
     case 1:
        if(sendFloodOff() != 0) return rtStandardError;
        break;
     }
   return rtNoError;
}

static cmdResponseType setSpindle(connectionRecType *context)
{
    // handle no spindle present
    if (emcStatus->motion.traj.spindles == 0) {
        OUT("no spindles configured");
        return rtCustomError;
    }

    // get cmd string: forward|reverse|increase|decrease|constant|off
    char *cmdStr = strtok(NULL, delims);
    // get spindle number string
    char *spindleStr = strtok(NULL, delims);
    // use first spindle by default
    int spindle = 0;
    // try to parse a spindle number
    if (spindleStr) {
        if (sscanf(spindleStr, "%d", &spindle) < 1) {
            OUT("failed to parse decimal: %s", spindleStr);
            return rtCustomError;
        }
        // validate
        if (spindle < -1 || spindle > emcStatus->motion.traj.spindles-1) {
            OUT(
                "invalid spindle: %d (valid: -1 - %d)",
                spindle, emcStatus->motion.traj.spindles
            );
            return rtCustomError;
        }
    }
    // walk all spindles
    for (int n = 0; n < emcStatus->motion.traj.spindles; n++){
        // process this spindle?
        if (n != spindle && spindle != -1)
            continue;

        switch (checkSpindleCmd(cmdStr)) {
            case 0:
                if(sendSpindleForward(n) != 0) return rtStandardError;
                break;

            case 1:
                if(sendSpindleReverse(n) != 0) return rtStandardError;
                break;

            case 2:
                if(sendSpindleIncrease(n) != 0) return rtStandardError;
                break;

            case 3:
                if(sendSpindleDecrease(n) != 0) return rtStandardError;
                break;

            case 4:
                if(sendSpindleConstant(n) != 0) return rtStandardError;
                break;

            case 5:
                if(sendSpindleOff(n) != 0) return rtStandardError;
                break;

            default:
                OUT(
                    "invalid command \"%s\" (valid: forward, "
                    "reverse, increase, decrease, constant, off)",
                    cmdStr
                );
                return rtCustomError;
        }
    }

	return rtNoError;
}

static cmdResponseType setBrake(connectionRecType *context)
{
    // handle no spindle present
    if (emcStatus->motion.traj.spindles == 0) {
        OUT("no spindles configured");
        return rtCustomError;
    }

    // get brake state string: on|off
    char *stateStr = strtok(NULL, delims);
    // get spindle number string
    char *spindleStr = strtok(NULL, delims);
    // use first spindle by default
    int spindle = 0;
    // try to parse a spindle number
    if (spindleStr) {
        if (sscanf(spindleStr, "%d", &spindle) < 1) {
            OUT("failed to parse decimal: %s", spindleStr);
            return rtCustomError;
        }
        // validate
        if (spindle < -1 || spindle > emcStatus->motion.traj.spindles-1) {
            OUT(
                "invalid spindle: %d (valid: -1 - %d)",
                spindle, emcStatus->motion.traj.spindles
            );
            return rtCustomError;
        }
    }
    // walk all spindles
    for (int n = 0; n < emcStatus->motion.traj.spindles; n++){
        // process this spindle?
        if (n != spindle && spindle != -1)
            continue;

        switch (checkOnOff(stateStr)) {
            case 0:
                if(sendBrakeEngage(n) != 0) return rtStandardError;
                break;

            case 1:
                if(sendBrakeRelease(n) != 0) return rtStandardError;
                break;

            default:
                OUT("invalid state: \"%s\" (valid: on, off)", stateStr);
                return rtCustomError;
        }
    }

    return rtNoError;
}

static cmdResponseType setLoadToolTable(connectionRecType * /*context*/)
{
  char *s = strtok(NULL, delims);
  if (!s) return rtStandardError;
  if (sendLoadToolTable(s) != 0) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setToolOffset(connectionRecType * /*context*/)
{
  int tool;
  float length, diameter;

  char *s = strtok(NULL, delims);
  if (!s) return rtStandardError;
  if (sscanf(s, "%d", &tool) < 1) return rtStandardError;
  s = strtok(NULL, delims);
  if (!s) return rtStandardError;
  if (sscanf(s, "%f", &length) < 1) return rtStandardError;
  s = strtok(NULL, delims);
  if (!s) return rtStandardError;
  if (sscanf(s, "%f", &diameter) < 1) return rtStandardError;

  if (sendToolSetOffset(tool, length, diameter) != 0) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setOverrideLimits(connectionRecType * /*context*/)
{
   char *s = strtok(NULL, delims);
   switch (checkOnOff(s)) {
     case -1: return rtStandardError;
     case 0: if (sendOverrideLimits(0) != 0) return rtStandardError; break;
     case 1: if (sendOverrideLimits(-1) != 0) return rtStandardError;
     }
   return rtNoError;
}

static cmdResponseType setMDI(connectionRecType * /*context*/)
{
  char *s = strtok(NULL, "\n\r\0");
  if (sendMdiCmd(s) !=0) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setHome(connectionRecType * /*context*/)
{
  int joint;

  char *jointStr = strtok(NULL, delims);
  if (!jointStr) return rtStandardError;
  if (sscanf(jointStr, "%d", &joint) < 1) return rtStandardError;
  // joint == -1 means "Home All", any other negative is wrong
  if ((joint < -1) || (joint > EMCMOT_MAX_JOINTS)) return rtStandardError;
  if(sendHome(joint) != 0) return rtStandardError;
  return rtNoError;
}

/* parse axis number or letter to axis number or -1 */
static int axisnumber(char *s) {
  int axis = -1; // Return all axes by default;
  if(!s)
    return axis;

  // try to parse axis number (0-5)
  if (sscanf(s, "%d", &axis) < 1) {
      // try to parse axis letter
      switch(s[0]) {
        case 'x': case 'X': axis = 0; break;
        case 'y': case 'Y': axis = 1; break;
        case 'z': case 'Z': axis = 2; break;
        case 'a': case 'A': case 'r': case 'R': axis = 3; break;
        case 'b': case 'B': case 'p': case 'P': axis = 4; break;
        case 'c': case 'C': case 'w': case 'W': axis = 5; break;
        default: axis = -1; break;
      }
  }
  return axis;
}

static cmdResponseType setJogStop(connectionRecType * /*context*/)
{
  int ja,jnum,jjogmode;
  char aletter;
  //parms:  jnum|aletter

  char *s = strtok(NULL, delims);
  if (!s) return rtStandardError;

  if (sscanf(s, "%d", &jnum) > 0) {
    ja = jnum;
    jjogmode = JOGJOINT;
  } else if (sscanf(s, "%c", &aletter) > 0) {
    ja = axisnumber(&aletter);
    jjogmode = JOGTELEOP;
  } else {
    return rtStandardError;
  }

  if (   (jjogmode == JOGJOINT)
      && ((ja < 0) || (ja > EMCMOT_MAX_JOINTS))
     ) return rtStandardError;

  if (sendJogStop(ja,jjogmode) != 0) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setJog(connectionRecType * /*context*/)
{
  int ja,jnum,jjogmode;
  char aletter;
  float speed;
  //parms:  jnum|aletter speed

  char *s = strtok(NULL, delims);
  if (!s) return rtStandardError;

  if (sscanf(s, "%d", &jnum) > 0) {
    ja = jnum;
    jjogmode = JOGJOINT;
  } else if (sscanf(s, "%c", &aletter) > 0) {
    ja = axisnumber(&aletter);
    jjogmode = JOGTELEOP;
  } else {
    return rtStandardError;
  }

  s = strtok(NULL, delims);
  if (!s) return rtStandardError;
  if (sscanf(s, "%f", &speed) < 1) return rtStandardError;

  if (sendJogCont(ja, jjogmode, speed) != 0) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setFeedOverride(connectionRecType * /*context*/)
{
  char *s = strtok(NULL, delims);
  if (!s)
    return rtStandardError;

  double percent;
  if (sscanf(s, "%lf", &percent) < 1)
    return rtStandardError;

  if(sendFeedOverride((percent) / 100.0) != 0)
    return rtStandardError;

  return rtNoError;
}

static cmdResponseType setJogIncr(connectionRecType * /*context*/)
{
  int jnum,ja,jjogmode;
  char aletter;
  float speed, incr;
  //parms:  jnum|aletter speed distance

  char *s = strtok(NULL, delims);
  if (!s) return rtStandardError;

  if (sscanf(s, "%d", &jnum) > 0) {
    ja = jnum;
    jjogmode = JOGJOINT;
  } else if (sscanf(s, "%c", &aletter) > 0) {
    ja = axisnumber(&aletter);
    jjogmode = JOGTELEOP;
  } else {
    return rtStandardError;
  }

  if (   (jjogmode == JOGJOINT)
      && ((ja < 0) || (ja > EMCMOT_MAX_JOINTS))
     ) return rtStandardError;

  s = strtok(NULL, delims);
  if (!s) return rtStandardError;

  if (sscanf(s, "%f", &speed) < 1) return rtStandardError;
  s = strtok(NULL, delims);

  if (!s) return rtStandardError;
  if (sscanf(s, "%f", &incr) < 1) return rtStandardError;

  if (sendJogIncr(ja, jjogmode, speed, incr) != 0) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setTaskPlanInit(connectionRecType * /*context*/)
{
  if (sendTaskPlanInit() != 0) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setOpen(connectionRecType *context)
{
  char *s = strtok(NULL, delims);
  if (!s) return rtStandardError;

  if(rtapi_strlcpy(context->progName, s, sizeof(context->progName)) >= sizeof(context->progName)) {
    OUT(
      "linuxcncrsh: 'set open' filename too long for context (got %lu bytes, max %lu)",
      (unsigned long) strlen(s),
      (unsigned long) sizeof(context->progName)
    );
    return rtCustomError;
  }

  if (sendProgramOpen(context->progName) != 0) return rtStandardError;
  
  return rtNoError;
}

static cmdResponseType setRun(connectionRecType * /*context*/)
{
  char *s = strtok(NULL, delims);
  if (!s) { // run from beginning
    if (sendProgramRun(0) != 0)
      return rtStandardError;
    return rtNoError;
  }

  // run from line number
  int lineNo;
  if (sscanf(s, "%d", &lineNo) < 1)
    return rtStandardError;

  if (sendProgramRun(lineNo) != 0)
    return rtStandardError;

  return rtNoError;
}

static cmdResponseType setPause(connectionRecType * /*context*/)
{
  if (sendProgramPause() != 0) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setResume(connectionRecType * /*context*/)
{
  if (sendProgramResume() != 0) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setStep(connectionRecType * /*context*/)
{
  if (sendProgramStep() != 0) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setAbort(connectionRecType * /*context*/)
{
  if (sendAbort() != 0) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setLinearUnitConversion(connectionRecType * /*context*/)
{
   char *s = strtok(NULL, delims);
   switch (checkConversionStr(s)) {
     case -1: return rtStandardError;
     case 0: linearUnitConversion = LINEAR_UNITS_INCH; break;
     case 1: linearUnitConversion = LINEAR_UNITS_MM; break;
     case 2: linearUnitConversion = LINEAR_UNITS_CM; break;
     case 3: linearUnitConversion = LINEAR_UNITS_AUTO; break;
     case 4: linearUnitConversion = LINEAR_UNITS_CUSTOM; break;
     }
   return rtNoError;
}

static cmdResponseType setAngularUnitConversion(connectionRecType * /*context*/)
{
   char *s = strtok(NULL, delims);
   switch (checkAngularConversionStr(s)) {
     case -1: return rtStandardError;
     case 0: angularUnitConversion = ANGULAR_UNITS_DEG; break;
     case 1: angularUnitConversion = ANGULAR_UNITS_RAD; break;
     case 2: angularUnitConversion = ANGULAR_UNITS_GRAD; break;
     case 3: angularUnitConversion = ANGULAR_UNITS_AUTO; break;
     case 4: angularUnitConversion = ANGULAR_UNITS_CUSTOM; break;
     }
   return rtNoError;
}

static cmdResponseType setTeleopEnable(connectionRecType * /*context*/)
{
   char *s = strtok(NULL, delims);
   switch (checkOnOff(s)) {
     case -1: return rtStandardError;
     case 0:
        if(sendSetTeleopEnable(1) != 0) return rtStandardError;
        break;
     case 1:
        if(sendSetTeleopEnable(0) != 0) return rtStandardError;
        break;
     }
   return rtNoError;
}

static cmdResponseType setProbe(connectionRecType * /*context*/)
{
  float x, y, z;

  char *s = strtok(NULL, delims);
  if (!s) return rtStandardError;
  fprintf(stderr,"0_probe %s\n",s);
  if (sscanf(s, "%f", &x) < 1) return rtStandardError;

  s = strtok(NULL, delims);
  if (!s) return rtStandardError;
  fprintf(stderr,"1_probe %s\n",s);
  if (sscanf(s, "%f", &y) < 1) return rtStandardError;

  s = strtok(NULL, delims);
  if (!s) return rtStandardError;
  fprintf(stderr,"2_probe %s\n",s);
  if (sscanf(s, "%f", &z) < 1) return rtStandardError;

  if(sendProbe(x, y, z) != 0) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setProbeClear(connectionRecType * /*context*/)
{
  if(sendClearProbeTrippedFlag() != 0) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setSpindleOverride(connectionRecType *context)
{
    // handle no spindle present
    if (emcStatus->motion.traj.spindles == 0) {
        OUT("no spindles configured");
        return rtCustomError;
    }

    // get override percentage string
    char *percentStr = strtok(NULL, delims);
    // get spindle number string
    char *spindleStr = strtok(NULL, delims);
    // use first spindle by default
    int spindle = 0;
    // try to parse a spindle number
    if (spindleStr) {
        if (sscanf(spindleStr, "%d", &spindle) < 1) {
            OUT("failed to parse decimal: %s", spindleStr);
            return rtCustomError;
        }
        // validate
        if (spindle < -1 || spindle > emcStatus->motion.traj.spindles-1) {
            OUT(
                "invalid spindle: %d (valid: -1 - %d)",
                spindle, emcStatus->motion.traj.spindles
            );
            return rtCustomError;
        }
    }
    // try to parse override percentage
    if(!percentStr) {
        OUT("missing parameter");
        return rtCustomError;
    }
	int percent;
    if(sscanf(percentStr, "%d", &percent) < 1) {
        OUT("parsing \"%s\" (%s)", percentStr, strerror(errno));
        return rtCustomError;
    }
    // validate
    if(percent < 0 || percent > 100) {
        OUT("invalid: %d (valid: 0-100)", percent);
        return rtCustomError;
    }

    // walk all spindles
    for (int n = 0; n < emcStatus->motion.traj.spindles; n++){
        // process this spindle?
        if (n != spindle && spindle != -1)
            continue;

        if(sendSpindleOverride(n, ((double) percent) / 100.0) != 0)
            return rtStandardError;
    }

	return rtNoError;
}

static cmdResponseType setOptionalStop(connectionRecType * /*context*/)
{
  int value;
  char *s = strtok(NULL, delims);
  sscanf(s, "%d", &value);
  if (sendSetOptionalStop(value) != 0) return rtStandardError;
  return rtNoError;
}

int commandSet(connectionRecType *context)
{
  cmdTokenType cmd;
  cmdResponseType ret = rtNoError;

  // parse cmd token
  char *tokenStr = strtok(NULL, delims);
  if (!tokenStr) {
    dprintf(context->cliSock, "SET NAK\r\n");
    return -1;
  }
  strupr(tokenStr);
  cmd = lookupCommandToken(tokenStr);
  if ((cmd >= scIniFile) && (context->cliSock != enabledConn)) {
    dprintf(context->cliSock, "SET %s NAK\r\n", tokenStr);
    return -1;
  }

  if ((cmd > scMachine) && (emcStatus->task.state != EMC_TASK_STATE::ON)) {
    //  Extra check in the event of an undetected change in Machine state resulting in
    //  sending a set command when the machine state is off. This condition is detected
    //  and appropriate error messages are generated, however erratic behavior has been
    //  seen when doing certain set commands when the Machine state is other than 'On
    dprintf(context->cliSock, "SET %s NAK\r\n", tokenStr);
    return -1;
  }

  switch (cmd) {
    case scEcho: ret = setEcho(context); break;
    case scVerbose: ret = setVerbose(context); break;
    case scEnable: ret = setEnable(context); break;
    case scConfig: ret = setConfig(context); break;
    case scCommMode: ret = setCommMode(context); break;
    case scCommProt: ret = setCommProt(context); break;
    case scIniFile: break;
    case scPlat: break;
    case scIni: break;
    case scDebug: ret = setDebug(context); break;
    case scSetWait: ret = setSetWait(context); break;     // remove this: deprecation alias for scWaitMode.
    case scWaitMode: ret = setWaitMode(context); break;
    case scWait: ret = setWait(context); break;
    case scSetTimeout: ret = setTimeout(context); break;
    case scUpdate: ret = setUpdate(context); break;
    case scError: ret = rtStandardError; break;
    case scOperatorDisplay: ret = rtStandardError; break;
    case scOperatorText: ret = rtStandardError; break;
    case scTime: ret = rtStandardError; break;
    case scEStop: ret = setEStop(context); break;
    case scMachine: ret = setMachine(context); break;
    case scMode: ret = setMode(context); break;
    case scMist: ret = setMist(context); break;
    case scFlood: ret = setFlood(context); break;
    case scSpindle: ret = setSpindle(context); break;
    case scBrake: ret = setBrake(context); break;
    case scTool: ret = rtStandardError; break;
    case scToolOffset: ret = setToolOffset(context); break;
    case scLoadToolTable: ret = setLoadToolTable(context); break;
    case scHome: ret = setHome(context); break;
    case scJogStop: ret = setJogStop(context); break;
    case scJog: ret = setJog(context); break;
    case scJogIncr: ret = setJogIncr(context); break;
    case scFeedOverride: ret = setFeedOverride(context); break;
    case scAbsCmdPos: ret = rtStandardError; break;
    case scAbsActPos: ret = rtStandardError; break;
    case scRelCmdPos: ret = rtStandardError; break;
    case scRelActPos: ret = rtStandardError; break;
    case scJointPos: ret = rtStandardError; break;
    case scPosOffset: ret = rtStandardError; break;
    case scJointLimit: ret = rtStandardError; break;
    case scJointFault: ret = rtStandardError; break;
    case scJointHomed: ret = rtStandardError; break;
    case scMDI: ret = setMDI(context); break;
    case scTskPlanInit: ret = setTaskPlanInit(context); break;
    case scOpen: ret = setOpen(context); break;
    case scRun: ret = setRun(context); break;
    case scPause: ret = setPause(context); break;
    case scResume: ret = setResume(context); break;
    case scAbort: ret = setAbort(context); break;
    case scStep: ret = setStep(context); break;
    case scProgram:ret = rtStandardError; break;
    case scProgramLine: ret = rtStandardError; break;
    case scProgramStatus: ret = rtStandardError; break;
    case scProgramCodes: ret = rtStandardError; break;
    case scJointType: ret = rtStandardError; break;
    case scJointUnits: ret = rtStandardError; break;
    case scProgramUnits:
    case scProgramLinearUnits: ret = rtStandardError; break;
    case scProgramAngularUnits: ret = rtStandardError; break;
    case scUserLinearUnits: ret = rtStandardError; break;
    case scUserAngularUnits: ret = rtStandardError; break;
    case scDisplayLinearUnits: ret = rtStandardError; break;
    case scDisplayAngularUnits: ret = rtStandardError; break;
    case scLinearUnitConversion: ret = setLinearUnitConversion(context); break;
    case scAngularUnitConversion: ret = setAngularUnitConversion(context); break;
    case scProbeClear: ret = setProbeClear(context); break;
    case scProbeTripped: ret = rtStandardError; break;
    case scProbeValue: ret = rtStandardError; break;
    case scProbe: ret = setProbe(context); break;
    case scTeleopEnable: ret = setTeleopEnable(context); break;    case scKinematicsType: ret = rtStandardError; break;
    case scOverrideLimits: ret = setOverrideLimits(context); break;
    case scSpindleOverride: ret = setSpindleOverride(context); break;
    case scOptionalStop: ret = setOptionalStop(context); break;
    case scUnknown: ret = rtStandardError;
  }

  switch (ret) {

    case rtNoError:
      if (context->verbose) {
        dprintf(context->cliSock, "SET %s ACK\r\n", tokenStr);
      }
      return -1;

    // Custom ok response already handled, take no action
    case rtHandledNoError:
      break;

    case rtStandardError:
      dprintf(context->cliSock, "SET %s NAK\r\n", tokenStr);
      return -1;

    // Custom error response entered in buffer
    case rtCustomError:
      dprintf(context->cliSock, "error: %s\r\n", context->outBuf);
      return -1;

    // Custom error response handled, take no action
    case rtCustomHandledError:
      return -1;
  }
  return 0;
}


static cmdResponseType getEcho(connectionRecType *context)
{
  if (context->echo) OUT("ECHO ON");
  else OUT("ECHO OFF");
  return rtNoError;
}

static cmdResponseType getVerbose(connectionRecType *context)
{
  if (context->verbose) OUT("VERBOSE ON");
  else OUT("VERBOSE OFF");
  return rtNoError;
}

static cmdResponseType getEnable(connectionRecType *context)
{
  if (context->cliSock == enabledConn)
    OUT("ENABLE ON");
  else OUT("ENABLE OFF");
  return rtNoError;
}

static cmdResponseType getConfig(connectionRecType *context)
{
  OUT("GET CONFIG not implemented");
  return rtCustomError;
}

static cmdResponseType getCommMode(connectionRecType *context)
{
  switch (context->commMode) {
    case 0: OUT("COMM_MODE ASCII"); break;
    case 1: OUT("COMM_MODE BINARY"); break;
    }
  return rtNoError;
}

static cmdResponseType getCommProt(connectionRecType *context)
{
  OUT("COMM_PROT %s", context->version);
  return rtNoError;
}

static cmdResponseType getDebug(connectionRecType *context)
{
  OUT("DEBUG %d", emcStatus->debug);
  return rtNoError;
}

static cmdResponseType getWaitMode(connectionRecType *context)
{
  switch (emcWaitType) {
    case EMC_WAIT_RECEIVED: OUT("WAIT_MODE RECEIVED"); break;
    case EMC_WAIT_DONE: OUT("WAIT_MODE DONE"); break;
    default: return rtStandardError;
  }
  return rtNoError;
}

/* compatibility wrapper to deprecate set_wait command token  - @todo remove at some point */
static cmdResponseType getSetWait(connectionRecType *context)
{
  dprintf(context->cliSock, "WARNING: \"set_wait\" command is depreciated and will be removed in the future. Please use \"wait_mode\" instead.\n");
  return getWaitMode(context);
}

static cmdResponseType getPlat(connectionRecType *context)
{
  OUT("PLAT Linux");
  return rtNoError;
}

static cmdResponseType getEStop(connectionRecType *context)
{
  if (emcStatus->task.state == EMC_TASK_STATE::ESTOP)
    OUT("ESTOP ON");
  else OUT("ESTOP OFF");
  return rtNoError;
}

static cmdResponseType getTimeout(connectionRecType *context)
{
  OUT("TIMEOUT %f", emcTimeout);
  return rtNoError;
}

static cmdResponseType getTime(connectionRecType *context)
{
  OUT("TIME %f", etime());
  return rtNoError;
}

static cmdResponseType getUpdate(connectionRecType *context)
{
    switch(emcUpdateType) {
        case EMC_UPDATE_NONE:
            OUT("UPDATE NONE");
            break;

        case EMC_UPDATE_AUTO:
            OUT("UPDATE AUTO");
            break;

        default:
            OUT("UPDATE UNKNOWN");
            break;
    }

    return rtNoError;
}

static cmdResponseType getError(connectionRecType *context)
{
  if (updateError() != 0) {
    OUT("emc_error: bad status from LinuxCNC");
    return rtCustomError;
  }

  if (error_string[0] == 0)
    OUT("ERROR OK");
  else {
    OUT("ERROR %s", error_string);
    error_string[0] = 0;
  }
  return rtNoError;
}

static cmdResponseType getOperatorDisplay(connectionRecType *context)
{
  if (updateError() != 0) {
    OUT("emc_operator_display: bad status from LinuxCNC");
    return rtCustomError;
  }

  if (operator_display_string[0] == 0)
    OUT("OPERATOR_DISPLAY OK");
  else {
    OUT("OPERATOR_DISPLAY %s", operator_display_string);
    operator_display_string[0] = 0;
  }
  return rtNoError;
}

static cmdResponseType getOperatorText(connectionRecType *context)
{
  if (updateError() != 0) {
    OUT("emc_operator_text: bad status from LinuxCNC");
    return rtCustomError;
  }

  if (operator_text_string[0] == 0)
    OUT("OPERATOR_TEXT OK");
  else {
    OUT("OPERATOR_TEXT %s", operator_text_string);
    operator_text_string[0] = 0;
  }
  return rtNoError;
}

static cmdResponseType getMachine(connectionRecType *context)
{
  if (emcStatus->task.state == EMC_TASK_STATE::ON)
    OUT("MACHINE ON");
  else
    OUT("MACHINE OFF");
  return rtNoError;
}

static cmdResponseType getMode(connectionRecType *context)
{
  switch (emcStatus->task.mode) {
    case EMC_TASK_MODE::MANUAL: OUT("MODE MANUAL"); break;
    case EMC_TASK_MODE::AUTO: OUT("MODE AUTO"); break;
    case EMC_TASK_MODE::MDI: OUT("MODE MDI"); break;
    default: OUT("MODE ?");
  }
  return rtNoError;
}

static cmdResponseType getMist(connectionRecType *context)
{
  if (emcStatus->io.coolant.mist == 1)
    OUT("MIST ON");
  else OUT("MIST OFF");
  return rtNoError;
}

static cmdResponseType getFlood(connectionRecType *context)
{
  if (emcStatus->io.coolant.flood == 1)
    OUT("FLOOD ON");
  else OUT("FLOOD OFF");
  return rtNoError;
}

static cmdResponseType getSpindle(connectionRecType *context)
{
    // handle no spindle present
    if (emcStatus->motion.traj.spindles == 0) {
        OUT("no spindles configured");
        return rtCustomError;
    }

    // get spindle number string
    char *spindleStr = strtok(NULL, delims);
    // use first spindle by default
    int spindle = 0;
    // try to parse a spindle number
    if (spindleStr) {
        if (sscanf(spindleStr, "%d", &spindle) < 1) {
            OUT("failed to parse decimal: %s", spindleStr);
            return rtCustomError;
        }
        // validate
        if (spindle < -1 || spindle > emcStatus->motion.traj.spindles-1) {
            OUT(
                "invalid spindle: %d (valid: -1 - %d)",
                spindle, emcStatus->motion.traj.spindles
            );
            return rtCustomError;
        }
    }

    // walk all spindles
    const char *pSpindleStr = "SPINDLE %d %s";
    for (int n = 0; n < emcStatus->motion.traj.spindles; n++){
        // process this spindle?
        if (n != spindle && spindle != -1)
            continue;

        if (emcStatus->motion.spindle[n].increasing > 0)
            OUT(pSpindleStr, n, "INCREASE");
        else if (emcStatus->motion.spindle[n].increasing < 0)
            OUT(pSpindleStr, n, "DECREASE");
        else if (emcStatus->motion.spindle[n].direction > 0)
            OUT(pSpindleStr, n, "FORWARD");
        else if (emcStatus->motion.spindle[n].direction < 0)
            OUT(pSpindleStr, n, "REVERSE");
        else OUT(pSpindleStr, n, "OFF");
    }

    return rtNoError;
}

static cmdResponseType getBrake(connectionRecType *context)
{
    // handle no spindle present
    if (emcStatus->motion.traj.spindles == 0) {
        OUT("no spindles configured");
        return rtCustomError;
    }

    // get spindle number string
    char *spindleStr = strtok(NULL, delims);
    // use first spindle by default
    int spindle = 0;
    // try to parse a spindle number
    if (spindleStr) {
        if (sscanf(spindleStr, "%d", &spindle) < 1) {
            OUT("failed to parse decimal: %s", spindleStr);
            return rtCustomError;
        }
        // validate
        if (spindle < -1 || spindle > emcStatus->motion.traj.spindles-1) {
            OUT(
                "invalid spindle: %d (valid: -1 - %d)",
                spindle, emcStatus->motion.traj.spindles
            );
            return rtCustomError;
        }
    }

    // walk all spindles
    const char *pBrakeStr = "BRAKE %s";
    for (int n = 0; n < emcStatus->motion.traj.spindles; n++){
        // process this spindle?
        if (n != spindle && spindle != -1)
            continue;

        if (emcStatus->motion.spindle[spindle].brake == 1)
            OUT(pBrakeStr, "ON");
        else
            OUT(pBrakeStr, "OFF");
    }

    return rtNoError;
}

static cmdResponseType getTool(connectionRecType *context)
{
  OUT("TOOL %d", emcStatus->io.tool.toolInSpindle);
  return rtNoError;
}

static cmdResponseType getToolOffset(connectionRecType *context)
{
  OUT("TOOL_OFFSET %f", emcStatus->task.toolOffset.tran.z);
  return rtNoError;
}

static cmdResponseType getAbsCmdPos(connectionRecType *context)
{
  char *axisStr = strtok(NULL, delims);
  int axis = axisnumber(axisStr);

  switch(axis) {

    case -1: // output all axes
      OUT(
        "ABS_CMD_POS %f %f %f %f %f %f",
        emcStatus->motion.traj.position.tran.x,
        emcStatus->motion.traj.position.tran.y,
        emcStatus->motion.traj.position.tran.z,
        emcStatus->motion.traj.position.a,
        emcStatus->motion.traj.position.b,
        emcStatus->motion.traj.position.c
      );
      break;

    case 0: // X
      OUT("ABS_CMD_POS X %f", emcStatus->motion.traj.position.tran.x);
      break;

    case 1: // Y
      OUT("ABS_CMD_POS Y %f", emcStatus->motion.traj.position.tran.y);
      break;

    case 2: // Z
      OUT("ABS_CMD_POS Z %f", emcStatus->motion.traj.position.tran.z);
      break;

    case 3: // A
      OUT("ABS_CMD_POS A %f", emcStatus->motion.traj.position.a);
      break;

    case 4: // B
      OUT("ABS_CMD_POS B %f", emcStatus->motion.traj.position.b);
      break;

    case 5: // C
      OUT("ABS_CMD_POS C %f", emcStatus->motion.traj.position.c);
      break;

  }
  return rtNoError;
}

static cmdResponseType getAbsActPos(connectionRecType *context)
{
  char *axisStr = strtok(NULL, delims);
  int axis = axisnumber(axisStr);

  switch(axis) {

    case -1: // output all axes
      OUT(
        "ABS_ACT_POS %f %f %f %f %f %f",
        emcStatus->motion.traj.actualPosition.tran.x,
        emcStatus->motion.traj.actualPosition.tran.y,
        emcStatus->motion.traj.actualPosition.tran.z,
        emcStatus->motion.traj.actualPosition.a,
        emcStatus->motion.traj.actualPosition.b,
        emcStatus->motion.traj.actualPosition.c
      );
      break;

    case 0: // X
      OUT("ABS_ACT_POS X %f", emcStatus->motion.traj.actualPosition.tran.x);
      break;

    case 1: // Y
      OUT("ABS_ACT_POS Y %f", emcStatus->motion.traj.actualPosition.tran.y);
      break;

    case 2: // Z
      OUT("ABS_ACT_POS Z %f", emcStatus->motion.traj.actualPosition.tran.z);
      break;

    case 3: // A
      OUT("ABS_ACT_POS A %f", emcStatus->motion.traj.actualPosition.a);
      break;

    case 4: // B
      OUT("ABS_ACT_POS B %f", emcStatus->motion.traj.actualPosition.b);
      break;

    case 5: // C
      OUT("ABS_ACT_POS C %f", emcStatus->motion.traj.actualPosition.c);
      break;

  }
  return rtNoError;
}

static cmdResponseType getRelCmdPos(connectionRecType *context)
{
  char *axisStr = strtok(NULL, delims);
  int axis = axisnumber(axisStr);

  switch(axis) {

    case -1: // output all axes
      OUT(
        "REL_CMD_POS %f %f %f %f %f %f",
        emcStatus->motion.traj.actualPosition.tran.x,
        emcStatus->motion.traj.actualPosition.tran.y,
        emcStatus->motion.traj.actualPosition.tran.z,
        emcStatus->motion.traj.actualPosition.a,
        emcStatus->motion.traj.actualPosition.b,
        emcStatus->motion.traj.actualPosition.c
      );
      break;

    case 0: // X
      OUT("REL_CMD_POS X %f", emcStatus->motion.traj.position.tran.x - emcStatus->task.g5x_offset.tran.x - emcStatus->task.g92_offset.tran.x);
      break;

    case 1: // Y
      OUT("REL_CMD_POS Y %f", emcStatus->motion.traj.position.tran.y - emcStatus->task.g5x_offset.tran.y - emcStatus->task.g92_offset.tran.y);
      break;

    case 2: // Z
      OUT("REL_CMD_POS Z %f", emcStatus->motion.traj.position.tran.z - emcStatus->task.g5x_offset.tran.z - emcStatus->task.g92_offset.tran.z);
      break;

    case 3: // A
      OUT("REL_CMD_POS A %f", emcStatus->motion.traj.position.a - emcStatus->task.g5x_offset.a - emcStatus->task.g92_offset.a);
      break;

    case 4: // B
      OUT("REL_CMD_POS B %f", emcStatus->motion.traj.position.b - emcStatus->task.g5x_offset.b - emcStatus->task.g92_offset.b);
      break;

    case 5: // C
      OUT("REL_CMD_POS C %f", emcStatus->motion.traj.position.c - emcStatus->task.g5x_offset.c - emcStatus->task.g92_offset.c);
      break;

  }
  return rtNoError;
}

static cmdResponseType getRelActPos(connectionRecType *context)
{
  char *axisStr = strtok(NULL, delims);
  int axis = axisnumber(axisStr);

  switch(axis) {

    case -1: // output all axes
      OUT(
        "REL_ACT_POS %f %f %f %f %f %f",
        emcStatus->motion.traj.actualPosition.tran.x,
        emcStatus->motion.traj.actualPosition.tran.y,
        emcStatus->motion.traj.actualPosition.tran.z,
        emcStatus->motion.traj.actualPosition.a,
        emcStatus->motion.traj.actualPosition.b,
        emcStatus->motion.traj.actualPosition.c
      );
      break;

    case 0: // X
      OUT("REL_ACT_POS X %f", emcStatus->motion.traj.actualPosition.tran.x - emcStatus->task.g5x_offset.tran.x - emcStatus->task.g92_offset.tran.x);
      break;

    case 1: // Y
      OUT("REL_ACT_POS Y %f", emcStatus->motion.traj.actualPosition.tran.y - emcStatus->task.g5x_offset.tran.y - emcStatus->task.g92_offset.tran.y);
      break;

    case 2: // Z
      OUT("REL_ACT_POS Z %f", emcStatus->motion.traj.actualPosition.tran.z - emcStatus->task.g5x_offset.tran.z - emcStatus->task.g92_offset.tran.z);
      break;

    case 3: // A
      OUT("REL_ACT_POS A %f", emcStatus->motion.traj.actualPosition.a - emcStatus->task.g5x_offset.a - emcStatus->task.g92_offset.a);
      break;

    case 4: // B
      OUT("REL_ACT_POS B %f", emcStatus->motion.traj.actualPosition.b - emcStatus->task.g5x_offset.b - emcStatus->task.g92_offset.b);
      break;

    case 5: // C
      OUT("REL_ACT_POS C %f", emcStatus->motion.traj.actualPosition.c - emcStatus->task.g5x_offset.c - emcStatus->task.g92_offset.c);
      break;

  }
  return rtNoError;
}

static cmdResponseType getJointPos(connectionRecType *context)
{
  int joint = -1; // Return all axes by default

  char *jointStr = strtok(NULL, delims);
  if (jointStr)
    joint = atoi(jointStr);

  if (joint == -1) {
    OUT(
      "JOINT_POS %f %f %f %f %f %f",
      emcStatus->motion.joint[0].input,
      emcStatus->motion.joint[1].input,
      emcStatus->motion.joint[2].input,
      emcStatus->motion.joint[3].input,
      emcStatus->motion.joint[4].input,
      emcStatus->motion.joint[5].input
    );
    return rtNoError;
  }

  OUT("JOINT_POS %d %f", joint, emcStatus->motion.joint[joint].input);
  return rtNoError;
}

static cmdResponseType getPosOffset(connectionRecType *context)
{
  char *axisStr = strtok(NULL, delims);
  int axis = axisnumber(axisStr);

  switch(axis) {

    case -1: // output all axes
      OUT(
        "POS_OFFSET %f %f %f %f %f %f",
        convertLinearUnits(emcStatus->task.g5x_offset.tran.x + emcStatus->task.g92_offset.tran.x),
        convertLinearUnits(emcStatus->task.g5x_offset.tran.y + emcStatus->task.g92_offset.tran.y),
        convertLinearUnits(emcStatus->task.g5x_offset.tran.z + emcStatus->task.g92_offset.tran.z),
        convertLinearUnits(emcStatus->task.g5x_offset.a + emcStatus->task.g92_offset.a),
        convertLinearUnits(emcStatus->task.g5x_offset.b + emcStatus->task.g92_offset.b),
        convertLinearUnits(emcStatus->task.g5x_offset.c + emcStatus->task.g92_offset.c)
      );
      break;

    case 0:
      OUT("POS_OFFSET X %f", convertLinearUnits(emcStatus->task.g5x_offset.tran.x + emcStatus->task.g92_offset.tran.x));
      break;

    case 1:
      OUT("POS_OFFSET Y %f", convertLinearUnits(emcStatus->task.g5x_offset.tran.y + emcStatus->task.g92_offset.tran.y));
      break;

    case 2:
      OUT("POS_OFFSET Z %f", convertLinearUnits(emcStatus->task.g5x_offset.tran.z + emcStatus->task.g92_offset.tran.z));
      break;

    case 3:
      OUT("POS_OFFSET A %f", convertLinearUnits(emcStatus->task.g5x_offset.a + emcStatus->task.g92_offset.a));
      break;

    case 4:
      OUT("POS_OFFSET B %f", convertLinearUnits(emcStatus->task.g5x_offset.b + emcStatus->task.g92_offset.b));
      break;

    case 5:
      OUT("POS_OFFSET C %f", convertLinearUnits(emcStatus->task.g5x_offset.c + emcStatus->task.g92_offset.c));
      break;
  }
  return rtNoError;
}

static cmdResponseType getJointLimit(connectionRecType *context)
{
  const char *pJointLimit = "JOINT_LIMIT";
  char buf[16];
  int joint, i;

  char *s = strtok(NULL, delims);
  if (!s) {
    rtapi_strxcpy(context->outBuf, pJointLimit);
    for (i=0; i<6; i++) {
      if (emcStatus->motion.joint[i].minHardLimit)
        rtapi_strxcpy(buf, " MINHARD");
      else
        if (emcStatus->motion.joint[i].minSoftLimit)
	  rtapi_strxcpy(buf, " MINSOFT");
	else
	  if (emcStatus->motion.joint[i].maxSoftLimit)
	    rtapi_strxcpy(buf, " MAXSOFT");
	  else
	    if (emcStatus->motion.joint[i].maxHardLimit)
	      rtapi_strxcpy(buf, " MAXHARD");
	    else rtapi_strxcpy(buf, " OK");
      rtapi_strxcat(context->outBuf, buf);
      }
    }
  else
    {
      joint = atoi(s);
      if (emcStatus->motion.joint[joint].minHardLimit)
        rtapi_strxcpy(buf, "MINHARD");
      else
        if (emcStatus->motion.joint[joint].minSoftLimit)
	  rtapi_strxcpy(buf, "MINSOFT");
	else
	  if (emcStatus->motion.joint[joint].maxSoftLimit)
	    rtapi_strxcpy(buf, "MAXSOFT");
	  else
	    if (emcStatus->motion.joint[joint].maxHardLimit)
	      rtapi_strxcpy(buf, "MAXHARD");
	    else rtapi_strxcpy(buf, "OK");
      OUT("%s %d %s", pJointLimit, joint, buf);
    }
  return rtNoError;
}

static cmdResponseType getJointFault(connectionRecType *context)
{
  const char *pJointFault = "JOINT_FAULT";
  char buf[16];
  int joint, i;

  char *s = strtok(NULL, delims);
  if (!s) {
    rtapi_strxcpy(context->outBuf, pJointFault);
    for (i=0; i<6; i++) {
      if (emcStatus->motion.joint[i].fault)
        rtapi_strxcat(context->outBuf, " FAULT");
      else rtapi_strxcat(context->outBuf, " OK");
      }
    }
  else {
      joint = atoi(s);
      if (emcStatus->motion.joint[joint].fault)
        rtapi_strxcpy(buf, "FAULT");
      else rtapi_strxcpy(buf, "OK");
      OUT("%s %d %s", pJointFault, joint, buf);
    }
  return rtNoError;
}

static cmdResponseType getOverrideLimits(connectionRecType *context)
{
  const char *pOverrideLimits = "OVERRIDE_LIMITS %d";

  OUT(pOverrideLimits, emcStatus->motion.joint[0].overrideLimits);
  return rtNoError;
}

static cmdResponseType getJointHomed(connectionRecType *context)
{
  const char *pJointHomed = "JOINT_HOMED";
  char buf[16];
  int joint, i;
  char *s = strtok(NULL, delims);
  if (!s) {
    rtapi_strxcpy(context->outBuf, pJointHomed);
    for (i=0; i<6; i++) {
      if (emcStatus->motion.joint[i].homed)
        rtapi_strxcat(context->outBuf, " YES");
      else rtapi_strxcat(context->outBuf, " NO");
      }
    }
  else {
      joint = atoi(s);
      if (emcStatus->motion.joint[joint].homed)
        rtapi_strxcpy(buf, "YES");
      else rtapi_strxcpy(buf, "NO");
      OUT("%s %d %s", pJointHomed, joint, buf);
    }
  return rtNoError;
}

static cmdResponseType getProgram(connectionRecType *context)
{
  if (emcStatus->task.file[0] != 0)
    OUT("PROGRAM %s", emcStatus->task.file);
  else
    OUT("PROGRAM NONE");
  return rtNoError;
}

static cmdResponseType getProgramLine(connectionRecType *context)
{
  int lineNo;

  if ((programStartLine< 0) || (emcStatus->task.readLine < programStartLine))
    lineNo = emcStatus->task.readLine;
  else
    if (emcStatus->task.currentLine > 0)
      if ((emcStatus->task.motionLine > 0) &&
        (emcStatus->task.motionLine < emcStatus->task.currentLine))
	  lineNo = emcStatus->task.motionLine;
      else lineNo = emcStatus->task.currentLine;
    else lineNo = 0;
  OUT("PROGRAM_LINE %d", lineNo);
  return rtNoError;
}

static cmdResponseType getProgramStatus(connectionRecType *context)
{
  switch (emcStatus->task.interpState) {
      case EMC_TASK_INTERP::READING:
      case EMC_TASK_INTERP::WAITING: OUT("PROGRAM_STATUS RUNNING"); break;
      case EMC_TASK_INTERP::PAUSED: OUT("PROGRAM_STATUS PAUSED"); break;
      default: OUT("PROGRAM_STATUS IDLE"); break;
    }
  return rtNoError;
}

static cmdResponseType getProgramCodes(connectionRecType *context)
{
  const char *pProgramCodes = "PROGRAM_CODES ";
  char buf[256];
  int code, i;

  buf[0] = 0;
  rtapi_strxcpy(context->outBuf, pProgramCodes);
  for (i=1; i<ACTIVE_G_CODES; i++) {
      code = emcStatus->task.activeGCodes[i];
      if (code == -1) continue;
      if (code % 10) snprintf(buf, sizeof(buf), "G%.1f ", (double) code / 10.0);
      else snprintf(buf, sizeof(buf), "G%d ", code / 10);
      rtapi_strxcat(context->outBuf, buf);
    }
  snprintf(buf, sizeof(buf), "F%.0f ", emcStatus->task.activeSettings[1]);
  rtapi_strxcat(context->outBuf, buf);
  snprintf(buf, sizeof(buf), "S%.0f", fabs(emcStatus->task.activeSettings[2]));
  rtapi_strxcat(context->outBuf, buf);
  return rtNoError;
}

static cmdResponseType getJointType(connectionRecType *context)
{
  const char *pJointType = "JOINT_TYPE";
  char buf[16];
  int joint, i;

  char *s = strtok(NULL, delims);
  if (!s) {
    rtapi_strxcpy(context->outBuf, pJointType);
    for (i=0; i<6; i++) {
      switch (emcStatus->motion.joint[i].jointType) {
        case EMC_LINEAR: rtapi_strxcat(context->outBuf, " LINEAR"); break;
	case EMC_ANGULAR: rtapi_strxcat(context->outBuf, " ANGULAR"); break;
	default: rtapi_strxcat(context->outBuf, "CUSTOM");
	}
      }
    }
  else {
      joint = atoi(s);
      switch (emcStatus->motion.joint[joint].jointType) {
        case EMC_LINEAR: rtapi_strxcpy(buf, " LINEAR"); break;
	case EMC_ANGULAR: rtapi_strxcpy(buf, " ANGULAR"); break;
	default: rtapi_strxcpy(buf, "CUSTOM");
	}
      OUT("%s %d %s", pJointType, joint, buf);
    }
  return rtNoError;
}

static cmdResponseType getJointUnits(connectionRecType *context)
{
  const char *pJointUnits = "JOINT_UNITS";
  char buf[16];
  int joint, i;

  char *s = strtok(NULL, delims);
  if (!s) {
    rtapi_strxcpy(context->outBuf, pJointUnits);
    for (i=0; i<6; i++) {
      switch (emcStatus->motion.joint[i].jointType) {
        case EMC_LINEAR:
	  if (CLOSE(emcStatus->motion.joint[i].units, 1.0, LINEAR_CLOSENESS))
	    rtapi_strxcat(context->outBuf, " MM");
	  else
	    if (CLOSE(emcStatus->motion.joint[i].units, INCH_PER_MM,
	      LINEAR_CLOSENESS)) rtapi_strxcat(context->outBuf, " INCH");
	    else
	      if (CLOSE(emcStatus->motion.joint[i].units, CM_PER_MM,
	        LINEAR_CLOSENESS)) rtapi_strxcat(context->outBuf, " CM");
	      else rtapi_strxcat(context->outBuf, " CUSTOM");
	  break;
	case EMC_ANGULAR:
	  if (CLOSE(emcStatus->motion.joint[i].units, 1.0, ANGULAR_CLOSENESS))
	    rtapi_strxcat(context->outBuf, " DEG");
	  else
  	    if (CLOSE(emcStatus->motion.joint[i].units, RAD_PER_DEG, ANGULAR_CLOSENESS))
	      rtapi_strxcat(context->outBuf, " RAD");
	    else
	      if (CLOSE(emcStatus->motion.joint[i].units, GRAD_PER_DEG, ANGULAR_CLOSENESS))
	        rtapi_strxcat(context->outBuf, " GRAD");
	      else rtapi_strxcat(context->outBuf, " CUSTOM");
	  break;
	default: rtapi_strxcat(context->outBuf, " CUSTOM");
	}
      }
    }
  else {
      joint = atoi(s);
      switch (emcStatus->motion.joint[joint].jointType) {
        case EMC_LINEAR:
	  if (CLOSE(emcStatus->motion.joint[joint].units, 1.0, LINEAR_CLOSENESS))
	    rtapi_strxcpy(buf, "MM");
	  else
	    if (CLOSE(emcStatus->motion.joint[joint].units, INCH_PER_MM,
	      LINEAR_CLOSENESS)) rtapi_strxcpy(buf, "INCH");
	    else
	      if (CLOSE(emcStatus->motion.joint[joint].units, CM_PER_MM,
	        LINEAR_CLOSENESS)) rtapi_strxcpy(buf, "CM");
	      else rtapi_strxcpy(buf, "CUSTOM");
	  break;
	case EMC_ANGULAR:
	  if (CLOSE(emcStatus->motion.joint[joint].units, 1.0, ANGULAR_CLOSENESS))
	    rtapi_strxcpy(buf, "DEG");
	  else
  	    if (CLOSE(emcStatus->motion.joint[joint].units, RAD_PER_DEG, ANGULAR_CLOSENESS))
	      rtapi_strxcpy(buf, "RAD");
	    else
	      if (CLOSE(emcStatus->motion.joint[joint].units, GRAD_PER_DEG, ANGULAR_CLOSENESS))
	        rtapi_strxcpy(buf, "GRAD");
	      else rtapi_strxcpy(buf, "CUSTOM");
	  break;
	default: rtapi_strxcpy(buf, "CUSTOM");
      OUT("%s %d %s", pJointUnits, joint, buf);
      }
    }
  return rtNoError;
}

static cmdResponseType getProgramLinearUnits(connectionRecType *context)
{
  switch (emcStatus->task.programUnits) {
    case CANON_UNITS_INCHES: OUT("PROGRAM_UNITS INCH"); break;
    case CANON_UNITS_MM: OUT("PROGRAM_UNITS MM"); break;
    case CANON_UNITS_CM: OUT("PROGRAM_UNITS CM"); break;
    default: OUT("PROGRAM_UNITS CUSTOM"); break;
    }
  return rtNoError;
}

static cmdResponseType getProgramAngularUnits(connectionRecType *context)
{
  OUT("PROGRAM_ANGULAR_UNITS DEG");
  return rtNoError;
}

static cmdResponseType getUserLinearUnits(connectionRecType *context)
{
  if (CLOSE(emcStatus->motion.traj.linearUnits, 1.0, LINEAR_CLOSENESS))
    OUT("USER_LINEAR_UNITS MM");
  else
    if (CLOSE(emcStatus->motion.traj.linearUnits, INCH_PER_MM, LINEAR_CLOSENESS))
      OUT("USER_LINEAR_UNITS INCH");
    else
      if (CLOSE(emcStatus->motion.traj.linearUnits, CM_PER_MM, LINEAR_CLOSENESS))
        OUT("USER_LINEAR_UNITS CM");
      else
        OUT("USER_LINEAR_UNITS CUSTOM");
  return rtNoError;
}

static cmdResponseType getUserAngularUnits(connectionRecType *context)
{
  if (CLOSE(emcStatus->motion.traj.angularUnits, 1.0, ANGULAR_CLOSENESS))
    OUT("USER_ANGULAR_UNITS DEG");
  else
    if (CLOSE(emcStatus->motion.traj.angularUnits, RAD_PER_DEG, ANGULAR_CLOSENESS))
      OUT("USER_ANGULAR_UNITS RAD");
    else
      if (CLOSE(emcStatus->motion.traj.angularUnits, GRAD_PER_DEG, ANGULAR_CLOSENESS))
        OUT("USER_ANGULAR_UNITS GRAD");
      else
        OUT("USER_ANGULAR_UNITS CUSTOM");
  return rtNoError;
}

static cmdResponseType getDisplayLinearUnits(connectionRecType *context)
{
  switch (linearUnitConversion) {
      case LINEAR_UNITS_INCH: OUT("DISPLAY_LINEAR_UNITS INCH"); break;
      case LINEAR_UNITS_MM: OUT("DISPLAY_LINEAR_UNITS MM"); break;
      case LINEAR_UNITS_CM: OUT("DISPLAY_LINEAR_UNITS CM"); break;
      case LINEAR_UNITS_AUTO:
        switch (emcStatus->task.programUnits) {
	        case CANON_UNITS_MM: OUT("DISPLAY_LINEAR_UNITS MM"); break;
	        case CANON_UNITS_INCHES: OUT("DISPLAY_LINEAR_UNITS INCH"); break;
	        case CANON_UNITS_CM: OUT("DISPLAY_LINEAR_UNITS CM"); break;
	        default: OUT("DISPLAY_LINEAR_UNITS CUSTOM");
	      }
        break;
      default: OUT("DISPLAY_LINEAR_UNITS CUSTOM");
    }
  return rtNoError;
}

static cmdResponseType getDisplayAngularUnits(connectionRecType *context)
{
  switch (angularUnitConversion) {
      case ANGULAR_UNITS_DEG: OUT("DISPLAY_ANGULAR_UNITS DEG"); break;
      case ANGULAR_UNITS_RAD: OUT("DISPLAY_ANGULAR_UNITS RAD"); break;
      case ANGULAR_UNITS_GRAD: OUT("DISPLAY_ANGULAR_UNITS GRAD"); break;
      case ANGULAR_UNITS_AUTO: OUT("DISPLAY_ANGULAR_UNITS DEG"); break;
      default: OUT("DISPLAY_ANGULAR_UNITS CUSTOM");
    }
  return rtNoError;
}

static cmdResponseType getLinearUnitConversion(connectionRecType *context)
{
  switch (linearUnitConversion) {
      case LINEAR_UNITS_INCH: OUT("LINEAR_UNIT_CONVERSION INCH"); break;
      case LINEAR_UNITS_MM: OUT("LINEAR_UNIT_CONVERSION MM"); break;
      case LINEAR_UNITS_CM: OUT("LINEAR_UNIT_CONVERSION CM"); break;
      case LINEAR_UNITS_AUTO: OUT("LINEAR_UNIT_CONVERSION AUTO"); break;
      default: OUT("LINEAR_UNIT_CONVERSION CUSTOM");
    }
  return rtNoError;
}

static cmdResponseType getAngularUnitConversion(connectionRecType *context)
{
  switch (angularUnitConversion) {
      case ANGULAR_UNITS_DEG: OUT("ANGULAR_UNIT_CONVERSION DEG"); break;
      case ANGULAR_UNITS_RAD: OUT("ANGULAR_UNIT_CONVERSION RAD"); break;
      case ANGULAR_UNITS_GRAD: OUT("ANGULAR_UNIT_CONVERSION GRAD"); break;
      case ANGULAR_UNITS_AUTO: OUT("ANGULAR_UNIT_CONVERSION AUTO"); break;
      default: OUT("ANGULAR_UNIT_CONVERSION CUSTOM");
    }
  return rtNoError;
}

static cmdResponseType getProbeValue(connectionRecType *context)
{
  OUT("PROBE_VALUE %d", emcStatus->motion.traj.probeval);
  return rtNoError;
}

static cmdResponseType getProbeTripped(connectionRecType *context)
{
  OUT("PROBE_TRIPPED %d", emcStatus->motion.traj.probe_tripped);
  return rtNoError;
}

static cmdResponseType getTeleopEnable(connectionRecType *context)
{
  if (emcStatus->motion.traj.mode == EMC_TRAJ_MODE::TELEOP)
    OUT("TELEOP_ENABLE ON");
  else OUT("TELEOP_ENABLE OFF");
  return rtNoError;
}

static cmdResponseType getKinematicsType(connectionRecType *context)
{
  OUT("KINEMATICS_TYPE %d", emcStatus->motion.traj.kinematics_type);
  return rtNoError;
}

static cmdResponseType getFeedOverride(connectionRecType *context)
{
  double percent = emcStatus->motion.traj.scale * 100.0;
  OUT("FEED_OVERRIDE %f", percent);
  return rtNoError;
}

static cmdResponseType getIniFile(connectionRecType *context)
{
  OUT("INIFILE %s", emc_inifile);
  return rtNoError;
}

static cmdResponseType getSpindleOverride(connectionRecType *context)
{
    // handle no spindle present
    if (emcStatus->motion.traj.spindles == 0) {
        OUT("no spindles configured");
        return rtCustomError;
    }

    // get spindle number string
    char *spindleStr = strtok(NULL, delims);
    // use first spindle by default
    int spindle = 0;
    // try to parse a spindle number
    if (spindleStr) {
        if (sscanf(spindleStr, "%d", &spindle) < 1) {
            OUT("failed to parse decimal: %s", spindleStr);
            return rtCustomError;
        }
        // validate
        if (spindle < -1 || spindle > emcStatus->motion.traj.spindles-1) {
            OUT(
               "invalid spindle: %d (valid: -1 - %d)",
                spindle, emcStatus->motion.traj.spindles
            );
            return rtCustomError;
        }
    }

    // walk all spindles
    for (int n = 0; n < emcStatus->motion.traj.spindles; n++){
        // process this spindle?
        if (n != spindle && spindle != -1)
            continue;

		  double percent = emcStatus->motion.spindle[n].spindle_scale * 100.0;
		  OUT("SPINDLE_OVERRIDE %d %f", n, percent);
  }

  return rtNoError;
}

static cmdResponseType getOptionalStop(connectionRecType *context)
{
  OUT("OPTIONAL_STOP %d", emcStatus->task.optional_stop_state);
  return rtNoError;
}

int commandGet(connectionRecType *context)
{
  const static char *setCmdNakStr = "GET %s NAK\r\n";
  cmdTokenType cmd;
  char *pch;
  cmdResponseType ret = rtNoError;

  pch = strtok(NULL, delims);
  if (!pch) {
    dprintf(context->cliSock, "GET NAK\r\n");
    return -1;
  }
  if (emcUpdateType == EMC_UPDATE_AUTO) updateStatus();
  strupr(pch);
  cmd = lookupCommandToken(pch);
  if (cmd > scIni)
    if (emcUpdateType == EMC_UPDATE_AUTO) updateStatus();
  switch (cmd) {
    case scEcho: ret = getEcho(context); break;
    case scVerbose: ret = getVerbose(context); break;
    case scEnable: ret = getEnable(context); break;
    case scConfig: ret = getConfig(context); break;
    case scCommMode: ret = getCommMode(context); break;
    case scCommProt: ret = getCommProt(context); break;
    case scIniFile: getIniFile(context); break;
    case scPlat: ret = getPlat(context); break;
    case scIni: break;
    case scDebug: ret = getDebug(context); break;
    case scSetWait: ret = getSetWait(context); break;         // remove this: deprecation alias for scWaitMode.
    case scWaitMode: ret = getWaitMode(context); break;
    case scWait: break;
    case scSetTimeout: ret = getTimeout(context); break;
    case scUpdate: ret = getUpdate(context); break;
    case scError: ret = getError(context); break;
    case scOperatorDisplay: ret = getOperatorDisplay(context); break;
    case scOperatorText: ret = getOperatorText(context); break;
    case scTime: ret = getTime(context); break;
    case scEStop: ret = getEStop(context); break;
    case scMachine: ret = getMachine(context); break;
    case scMode: ret = getMode(context); break;
    case scMist: ret = getMist(context); break;
    case scFlood: ret = getFlood(context); break;
    case scSpindle: ret = getSpindle(context); break;
    case scBrake: ret = getBrake(context); break;
    case scTool: ret = getTool(context); break;
    case scToolOffset: ret = getToolOffset(context); break;
    case scLoadToolTable: ret = rtStandardError; break;
    case scHome: ret = rtStandardError; break;
    case scJogStop: ret = rtStandardError; break;
    case scJog: ret = rtStandardError; break;
    case scJogIncr: ret = rtStandardError; break;
    case scFeedOverride: ret = getFeedOverride(context); break;
    case scAbsCmdPos: ret = getAbsCmdPos(context); break;
    case scAbsActPos: ret = getAbsActPos(context); break;
    case scRelCmdPos: ret = getRelCmdPos(context); break;
    case scRelActPos: ret = getRelActPos(context); break;
    case scJointPos: ret = getJointPos(context); break;
    case scPosOffset: ret = getPosOffset(context); break;
    case scJointLimit: ret = getJointLimit(context); break;
    case scJointFault: ret = getJointFault(context); break;
    case scJointHomed: ret = getJointHomed(context); break;
    case scMDI: ret = rtStandardError; break;
    case scTskPlanInit: ret = rtStandardError; break;
    case scOpen: ret = rtStandardError; break;
    case scRun: ret = rtStandardError; break;
    case scPause: ret = rtStandardError; break;
    case scResume: ret = rtStandardError; break;
    case scStep: ret = rtStandardError; break;
    case scAbort: ret = rtStandardError; break;
    case scProgram: ret = getProgram(context); break;
    case scProgramLine: ret = getProgramLine(context); break;
    case scProgramStatus: ret = getProgramStatus(context); break;
    case scProgramCodes: ret = getProgramCodes(context); break;
    case scJointType: ret = getJointType(context); break;
    case scJointUnits: ret = getJointUnits(context); break;
    case scProgramUnits:
    case scProgramLinearUnits: ret = getProgramLinearUnits(context); break;
    case scProgramAngularUnits: ret = getProgramAngularUnits(context); break;
    case scUserLinearUnits: ret = getUserLinearUnits(context); break;
    case scUserAngularUnits: ret = getUserAngularUnits(context); break;
    case scDisplayLinearUnits: ret = getDisplayLinearUnits(context); break;
    case scDisplayAngularUnits: ret = getDisplayAngularUnits(context); break;
    case scLinearUnitConversion: ret = getLinearUnitConversion(context); break;
    case scAngularUnitConversion: ret = getAngularUnitConversion(context); break;
    case scProbeClear: break;
    case scProbeTripped: ret = getProbeTripped(context); break;
    case scProbeValue: ret = getProbeValue(context); break;
    case scProbe: break;
    case scTeleopEnable: ret = getTeleopEnable(context); break;
    case scKinematicsType: ret = getKinematicsType(context); break;
    case scOverrideLimits: ret = getOverrideLimits(context); break;
    case scSpindleOverride: ret = getSpindleOverride(context); break;
    case scOptionalStop: ret = getOptionalStop(context); break;
    case scUnknown: ret = rtStandardError;
    }
  switch (ret) {
    case rtNoError: // Standard ok response, just write value in buffer
      dprintf(context->cliSock, "%s\r\n", context->outBuf);
      break;
    case rtHandledNoError: // Custom ok response already handled, take no action
      break;
    case rtStandardError: // Standard error response
      dprintf(context->cliSock, setCmdNakStr, pch);
      break;
    case rtCustomError: // Custom error response entered in buffer
      dprintf(context->cliSock, "error: %s\r\n", context->outBuf);
      break;  
    case rtCustomHandledError: ;// Custom error response handled, take no action
    }
  return 0;
}

int commandQuit(connectionRecType *context)
{
  printf("Closing connection with %s\n", context->hostName);
  return -1;
}

int commandShutdown(connectionRecType *context)
{
  if (context->cliSock == enabledConn) {
    printf("Shutting down\n");
    thisQuit();
    return -1;
    }
  else
    return 0;
}

static int helpGeneral(connectionRecType *context)
{
  dprintf(
    context->cliSock,
    "Available commands:\r\n"
    "  Hello <password> <client name> <protocol version>\r\n"
    "  Get <LinuxCNC command>\r\n"
    "  Set <LinuxCNC command>\r\n"
    "  Shutdown\r\n"
    "  Help <command>\r\n"
  );
  return 0;
}

static int helpHello(connectionRecType *context)
{
  dprintf(
    context->cliSock,
    "Usage:\r\n"
    "  Hello <Password> <Client Name> <Protocol Version>\r\nWhere:\r\n"
    "  Password is the connection password to allow communications with the CNC server.\r\n"
    "  Client Name is the name of client trying to connect, typically the network name of the client.\r\n"
    "  Protocol Version is the version of the protocol with which the client wishes to use.\r\n\r\n"
    "  With valid password, server responds with:\r\n"
    "  Hello Ack <Server Name> <Protocol Version>\r\nWhere:\r\n"
    "  Ack is acknowledging the connection has been made.\r\n"
    "  Server Name is the name of the LinuxCNC Server to which the client has connected.\r\n"
    "  Protocol Version is the client requested version or latest version support by server if"
    "  the client requests a version later than that supported by the server.\r\n\r\n"
    "  With invalid password, the server responds with:\r\n"
    "  Hello Nak\r\n"
  );
  return 0;
}

static int helpGet(connectionRecType *context)
{
  dprintf(
    context->cliSock,
    "Usage:\r\nGet <LinuxCNC command>\r\n"
    "  Get commands require that a hello has been successfully negotiated.\r\n"
    "  LinuxCNC command may be one of:\r\n"
    "    Abs_act_pos\r\n"
    "    Abs_cmd_pos\r\n"
    "    Angular_unit_conversion\r\n"
    "    Brake {<Spindle>}\r\n"
    "    Comm_mode\r\n"
    "    Comm_prot\r\n"
    "    Debug\r\n"
    "    Display_angular_units\r\n"
    "    Display_linear_units\r\n"
    "    Echo\r\n"
    "    Enable\r\n"
    "    Error\r\n"
    "    EStop\r\n"
    "    Feed_override\r\n"
    "    Flood\r\n"
    "    Inifile\r\n"
    "    Joint_fault\r\n"
    "    Joint_homed\r\n"
    "    Joint_limit\r\n"
    "    Joint_pos\r\n"
    "    Joint_type\r\n"
    "    Joint_units\r\n"
    "    Kinematics_type\r\n"
    "    Linear_unit_conversion\r\n"
    "    Machine\r\n"
    "    Mist\r\n"
    "    Mode\r\n"
    "    Operator_display\r\n"
    "    Operator_text\r\n"
    "    Optional_stop\r\n"
    "    Override_limits\r\n"
    "    Plat\r\n"
    "    Pos_offset\r\n"
    "    Probe_tripped\r\n"
    "    Probe_value\r\n"
    "    Program\r\n"
    "    Program_angular_units\r\n"
    "    Program_codes\r\n"
    "    Program_line\r\n"
    "    Program_linear_units\r\n"
    "    Program_status\r\n"
    "    Program_units\r\n"
    "    Rel_act_pos\r\n"
    "    Rel_cmd_pos\r\n"
    "    Wait_mode\r\n"
    "    Spindle {<Spindle>}\r\n"
    "    Spindle_override {<Spindle>}\r\n"
    "    Teleop_enable\r\n"
    "    Time\r\n"
    "    Timeout\r\n"
    "    Tool\r\n"
    "    Tool_offset\r\n"
    "    Update\r\n"
    "    User_angular_units\r\n"
    "    User_linear_units\r\n"
    "    Verbose\r\n"
  );
  //    "CONFIG\r\n"
  return 0;
}

static int helpSet(connectionRecType *context)
{
  dprintf(
    context->cliSock,
    "Usage:\r\n  Set <LinuxCNC command>\r\n"
    "  Set commands require that a hello has been successfully negotiated,\r\n"
    "  in most instances requires that control be enabled by the connection.\r\n"
    "  The set commands not requiring control enabled are:\r\n"
    "    Comm_mode <mode>\r\n"
    "    Comm_prot <protocol>\r\n"
    "    Echo <On | Off>\r\n"
    "    Enable <Pwd | Off>\r\n"
    "    Verbose <On | Off>\r\n\r\n"
    "  The set commands requiring control enabled are:\r\n"
    "    Abort\r\n"
    "    Angular_unit_conversion <Deg | Rad | Grad | Auto | Custom>\r\n"
    "    Brake <On | Off> {<Spindle>}\r\n"
    "    Debug <Debug level>\r\n"
    "    EStop <On | Off>\r\n"
    "    Feed_override <Percent>\r\n"
    "    Flood <On | Off>\r\n"
    "    Home <Axis No>\r\n"
    "    Jog <Axis No, Speed>\r\n"
    "    Jog_incr <Axis No, Speed, Distance>\r\n"
    "    Jog_stop <Joint No|Axis letter>\r\n"
    "    Linear_unit_conversion <Inch | CM | MM | Auto | Custom>\r\n"
    "    Load_tool_table <Table name>\r\n"
    "    Machine <On | Off>\r\n"
    "    MDI <MDI String>\r\n"
    "    Mist <On | Off>\r\n"
    "    Mode <Manual | Auto | MDI>\r\n"
    "    Open <File path / name>\r\n"
    "    Optional_stop <none | 0 | 1>\r\n"
    "    Override_limits <On | Off>\r\n"
    "    Pause\r\n"
    "    Probe\r\n"
    "    Probe_clear\r\n"
    "    Resume\r\n"
    "    Run <Line No>\r\n"
    "    Wait_mode <None | Received | Done>\r\n"
    "    Spindle <Increase | Decrease | Forward | Reverse | Constant | Off> {<Spindle>}\r\n"
    "    Spindle_override <percent> {<Spindle>}\r\n"
    "    Step\r\n"
    "    Task_plan_init\r\n"
    "    Teleop_enable <On | Off>\r\n"
    "    Timeout <Time>\r\n"
    "    Tool_offset <Offset>\r\n"
    "    Update <None | Auto>\r\n"
    "    Wait <Time>\r\n"
  );
  return 0;
}

static int helpQuit(connectionRecType *context)
{
  dprintf(
    context->cliSock,
    "Usage:\r\n"
    "  The quit command has the server initiate a disconnect from the client,\r\n"
    "  the command has no parameters and no requirements to have negotiated\r\n"
    "  a hello, or be in control."
  );
  return 0;
}

static int helpShutdown(connectionRecType *context)
{
  dprintf(
    context->cliSock,
    "Usage:\r\n"
    "  The shutdown command terminates the connection with all clients,\r\n"
    "  and initiates a shutdown of LinuxCNC. The command has no parameters, and\r\n"
    "  can only be issued by the connection having control.\r\n"
  );
  return 0;
}

static int helpHelp(connectionRecType *context)
{
  dprintf(
    context->cliSock,
    "If you need help on help, it is time to look into another line of work.\r\n"
  );
  return 0;
}

int commandHelp(connectionRecType *context)
{
  char *s = strtok(NULL, delims);
  if (!s) return (helpGeneral(context));
  strupr(s);
  if (strcmp(s, "HELLO") == 0) return (helpHello(context));
  if (strcmp(s, "GET") == 0) return (helpGet(context));
  if (strcmp(s, "SET") == 0) return (helpSet(context));
  if (strcmp(s, "QUIT") == 0) return (helpQuit(context));
  if (strcmp(s, "SHUTDOWN") == 0) return (helpShutdown(context));
  if (strcmp(s, "HELP") == 0) return (helpHelp(context));
  dprintf(context->cliSock, "%s is not a valid command.", s);
  return 0;
}

cmdType lookupCommand(char *s)
{
  cmdType i = cmdHello;
  int temp;

  while (i < cmdUnknown) {
    if (strcmp(commands[i], s) == 0) return i;
//    (int)i += 1;
    temp = i;
    temp++;
    i = (cmdType) temp;
    }
  return i;
}

// handle the linuxcncrsh command in context->inBuf
int parseCommand(connectionRecType *context)
{
  int ret = 0;
  char *cmdStr;

  if (!(cmdStr = strtok(context->inBuf, delims)))
    return -1;

  strupr(cmdStr);
  switch (lookupCommand(cmdStr)) {

    case cmdHello:
      if ((ret = commandHello(context)) < 0)
        dprintf(context->cliSock, "HELLO NAK\r\n");
      else
        dprintf(context->cliSock, "HELLO ACK %s 1.1\r\n", serverName);
      break;

    case cmdGet:
      ret = commandGet(context);
      break;

    case cmdSet:
      if (!context->linked) {
        dprintf(context->cliSock, "SET NAK\r\n");
        ret = -1;
        break;
      }
      ret = commandSet(context);
      break;

    case cmdQuit:
      ret = commandQuit(context);
      break;

    case cmdShutdown:
      if((ret = commandShutdown(context)) < 0) {
        dprintf(context->cliSock, "SHUTDOWN NAK\r\n");
      }
      break;

    case cmdHelp:
      ret = commandHelp(context);
      break;

    default:
      ret = -2;
  }

  return ret;
}

void *readClient(void *arg)
{
  char buf[1600];
  int context_index;
  int i;
  int len;
  connectionRecType *context = (connectionRecType *)arg;

  context_index = 0;

  while (1) {
    // We always start this loop with an empty buf, though there may be one
    // partial line in context->inBuf[0..context_index].
    len = read(context->cliSock, buf, sizeof(buf));
    if (len < 0) {
      fprintf(stderr, "linuxcncrsh: error reading from client: %s\n", strerror(errno));
      goto finished;
    }
    if (len == 0) {
      printf("linuxcncrsh: eof from client\n");
      goto finished;
    }

    if (context->echo && context->linked)
      if(write(context->cliSock, buf, len) != (ssize_t)len) {
        fprintf(stderr, "linuxcncrsh: write() failed: %s", strerror(errno));
      }

    for (i = 0; i < len; i ++) {
        if ((buf[i] != '\n') && (buf[i] != '\r')) {
            context->inBuf[context_index] = buf[i];
            context_index ++;
            continue;
        }

        // if we get here, i is the index of a line terminator in buf

        if (context_index > 0) {
            // we have some bytes in the context buffer, parse them now
            context->inBuf[context_index] = '\0';

            // The return value from parseCommand was meant to indicate
            // success or error, but it is unusable.  Some paths return
            // the return value of write(2) and some paths return small
            // positive integers (cmdResponseType) to indicate failure.
            // We're best off just ignoring it.
            (void)parseCommand(context);

            context_index = 0;
        }
    }
  }

finished:
  printf("linuxcncrsh: disconnecting client %s (%s)\n", context->hostName, context->version);
  close(context->cliSock);
  free(context);
  pthread_exit((void *)0);
  sessions--;  // FIXME: not reached
}

int sockMain()
{
    int res;

    while (1) {
        int client_sockfd;
        client_len = sizeof(client_address);
        client_sockfd = accept(server_sockfd, (struct sockaddr *)&client_address, &client_len);
        if (client_sockfd < 0) exit(0);
        // count connected clients
        sessions++;
        // enforce limited amount of clients that can connect simultaneously
        if ((maxSessions == -1) || (sessions <= maxSessions)) {
            pthread_t *thrd;
            connectionRecType *context;

            thrd = (pthread_t *)calloc(1, sizeof(pthread_t));
            if (!thrd) {
                fprintf(stderr, "linuxcncrsh: out of memory\n");
                exit(1);
            }

            context = (connectionRecType *)malloc(sizeof(connectionRecType));
            if (!context) {
                fprintf(stderr, "linuxcncrsh: out of memory\n");
                exit(1);
            }

            context->cliSock = client_sockfd;
            context->linked = false;
            context->echo = true;
            context->verbose = false;
            rtapi_strxcpy(context->version, "1.0");
            rtapi_strxcpy(context->hostName, "Default");
            context->enabled = false;
            context->commMode = 0;
            context->commProt = 0;
            context->inBuf[0] = 0;

            res = pthread_create(thrd, NULL, readClient, (void *)context);
        } else {
            fprintf(stderr, "linuxcncrsh: maximum amount of sessions exceeded: %d\n", maxSessions);
            res = -1;
        }
        // discard connection upon error
        if (res != 0) {
            close(client_sockfd);
            sessions--;
        }
    }
    return 0;
}

static void usage(char* pname) {
    printf("Usage: \n");
    printf("         %s [Options] [-- LinuxCNC_Options]\n"
           "Options:\n"
           "         --help       this help\n"
           "         --port       <port number>  (default=%d)\n"
           "         --name       <server name>  (default=%s)\n"
           "         --connectpw  <password>     (default=%s)\n"
           "         --enablepw   <password>     (default=%s)\n"
           "         --sessions   <max sessions> (default=%d) (-1 ==> no limit) \n"
           "         --path       <path>         (default=%s)\n"
           "LinuxCNC_Options:\n"
           "          -ini        <INI file>      (default=%s)\n"
          ,pname,port,serverName,pwd,enablePWD,maxSessions,defaultPath,emc_inifile
          );
}

int main(int argc, char *argv[])
{
    int opt;

    // initialize default values
    emcWaitType = EMC_WAIT_RECEIVED;
    emcCommandSerialNumber = 0;
    emcTimeout = 0.0;
    emcUpdateType = EMC_UPDATE_AUTO;
    linearUnitConversion = LINEAR_UNITS_AUTO;
    angularUnitConversion = ANGULAR_UNITS_AUTO;
    emcCommandBuffer = NULL;
    emcStatusBuffer = NULL;
    emcErrorBuffer = NULL;
    emcStatus = 0;
    error_string[LINELEN-1] = 0;
    operator_text_string[LINELEN-1] = 0;
    operator_display_string[LINELEN-1] = 0;
    programStartLine = 0;


    // process local command line args
    while ((opt = getopt_long(argc, argv, "he:n:p:s:w:d:", longopts, NULL)) != -1) {
        switch (opt) {
            case 'h': usage(argv[0]); exit(1);
            case 'e': snprintf(enablePWD, sizeof(enablePWD), "%s", optarg); break;
            case 'n': snprintf(serverName, sizeof(serverName), "%s", optarg); break;
            case 'p': sscanf(optarg, "%d", &port); break;
            case 's': sscanf(optarg, "%d", &maxSessions); break;
            case 'w': snprintf(pwd, sizeof(pwd), "%s", optarg); break;
            case 'd': snprintf(defaultPath, sizeof(defaultPath), "%s", optarg); break;
        }
    }

    // process LinuxCNC command line args
    // Note: '--' may be used to separate cmd line args
    //       optind is index of next arg to process
    //       make argv[optind] zeroth arg
    argc = argc - optind + 1;
    argv = argv + optind - 1;
    if (emcGetArgs(argc, argv) != 0) {
        rcs_print_error("error in argument list\n");
        exit(1);
    }
    // get configuration information
    iniLoad(emc_inifile);
    // initialize telnet socket
    if (initSocket() != 0) {
        rcs_print_error("error initializing sockets\n");
        exit(1);
    }
    // init NML
    if (tryNml() != 0) {
        rcs_print_error("can't connect to LinuxCNC\n");
        thisQuit();
        exit(1);
    }
    // get current serial number, and save it for restoring when we quit
    // so as not to interfere with real operator interface
    updateStatus();
    emcCommandSerialNumber = emcStatus->echo_serial_number;

    // attach our quit function to SIGINT
    {
        struct sigaction act;
        act.sa_handler = sigQuit;
        sigemptyset(&act.sa_mask);
        act.sa_flags = 0;
        sigaction(SIGINT, &act, NULL);
    }

    // make all threads ignore SIGPIPE
    {
        struct sigaction act;
        act.sa_handler = SIG_IGN;
        sigemptyset(&act.sa_mask);
        act.sa_flags = 0;
        sigaction(SIGPIPE, &act, NULL);
    }

    if (useSockets) sockMain();

    return 0;
}
