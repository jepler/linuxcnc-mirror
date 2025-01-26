/********************************************************************
* Description: schedrmt.cc
*   Extended telnet based scheduler interface
*
*   Derived from a work by Fred Proctor & Will Shackleford
*   Further derived from work by jmkasunich
*
* Author: Eric H. Johnson
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2009 All rights reserved.
*
* Last change:
********************************************************************/

#define _REENTRANT

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <ctype.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <netinet/in.h>
#include <pthread.h>
#include <errno.h>

#include <getopt.h>

#include "rcs.hh"
#include "posemath.h"		// PM_POSE, TO_RAD
#include "emc.hh"		// EMC NML
#include "canon.hh"		// CANON_UNITS, CANON_UNITS_INCHES,MM,CM
#include "emcglb.h"		// EMC_NMLFILE, TRAJ_MAX_VELOCITY, etc.
#include "emccfg.h"		// DEFAULT_TRAJ_MAX_VELOCITY
#include "inifile.hh"		// INIFILE
#include "rcs_print.hh"
#include "timer.hh"             // etime()
#include "shcom.hh"             // NML Messaging functions
#include "emcsched.hh"
#include <rtapi_string.h>

/*
  Using schedrmt:

  schedrmt {-- --port <port number> --name <server name> --connectpw <password>
             --enablepw <password> --sessions <max sessions> --path <path>
             -ini <INI file>}

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

  There are six commands supported, Where the commands set and get contain EMC
  specific sub-commands based on the commands supported by emcsh, but where the "emc_"
  is omitted. Commands and most parameters are not case sensitive. The exceptions are 
  passwords, file paths and text strings.
  
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
  
  The get command includes one of the emc sub-commands, described below and
  zero or more additional parameters. 
  
  ==> Set <==
  
  The set command inclides one of the emc sub-commands, described below and
  one or more additional parameters.
  
  ==> Quit <==
  
  The quit command disconnects the associated socket connection.
  
  ==> Shutdown <==
  
  The shutdown command tells EMC to shutdown before quitting the connection. This
  command may only be issued if the Hello has been successfully negotiated and the
  connection has control of the CNC (see enable sub-command below). This command
  has no parameters.
  
  ==> Help <==
  
  The help command will return help information in text format over the telnet
  connection. If no parameters are specified, it will itemize the available commands.
  If a command is specified, it will provide usage information for the specified
  command. Help will respond regardless of whether a "Hello" has been
  successfully negotiated.
  
  
  EMC sub-commands:
  
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
  Returns the path and file name of the current configuration INI file.

  plat
  Returns the platform for which this was compiled, e.g., linux_2_0_36

  ini <var> <section>
  Returns the string value of <var> in section <section>, in EMC_INIFILE

  debug {<new value>}
  With get, returns the integer value of EMC_DEBUG, in the EMC. Note that
  it may not be true that the local EMC_DEBUG variable here (in emcsh and
  the GUIs that use it) is the same as the EMC_DEBUG value in the EMC. This
  can happen if the EMC is started from one INI file, and the GUI is started
  with another that has a different value for DEBUG.
  With set, sends a command to the EMC to set the new debug level,
  and sets the EMC_DEBUG global here to the same value. This will make
  the two values the same, since they really ought to be the same.

  QMode <mode>
  stop | run | pause | resume (Set only) | error (Get only)
  With no arg, returns the program queue status as "stop", "run", "pause" or "error". Otherwise,
  sends a command to set the current mode to "stop", "run" or "pause".

  QStatus <Queue Size> <First Tag Id> <Last Tag Id> <Queue CRC>
  Get only, returns then number of programs in queue (Queue Size), the Tag id of the first 
  program in the queue, the Tag id of the last program in the queue, and the CRC of all
  of the Tag Ids in the queue. The actual calculation of the CRC is not important, the
  purpose is to be able to compare the current CRC with the previous CRC. If they
  differ, then there has been a change to the size or order of the programs in queue.

  AutoTagId <Start Id>
  With get, returns the next autoincremented unique tag id to associate with a queue record.
  With set, resets auto tag generation to begin at the specified value.

  PgmAdd <priority> <tag id> <x> <y> <z> <zone> <file name> <feed override> <spindle override> <tool>
  With set, adds a program to the queue with priority of the program, a unique tag identifying the 
  program, the x, y and z offsets or zone for the origin of the program, the path + file name, the
  feed and spindle overrides to apply, and the default tool to use. If tag id is zero, the tag id
  will be generated automatically. If zone is zero, then the x, y, and z offsets will be used, 
  otherwise zones 1 to 9 correspond to G54 to G59.3 respectively.

  PgmById <tag id>
  [priority] [tag id] [x] [y] [z] [zone] [file name] [feed override] [spindle override] [tool]
  With get, returns the queue entry matching the specified tag id, including the priority,
  tag id, x, y, and z coordinates, the zone, file name, feed and spindle overrides and the default
  tool.

  PgmByIndex <index>
  [priority] [tag id] [x] [y] [z] [zone] [file name] [feed override] [spindle override] [tool]
  With get, returns the queue entry matching the specified index into the queue, including the priority,
  tag id, x, y, and z coordinates, the zone, file name, feed and spindle overrides and the default
  tool.

  PgmAll
  With get, performs effectively a PgmByIndex for every entry in the queue. Each result will be
  returned in the form: "PGMBYINDEX ..." with cr lf at the end of each record.

  PriorityById <tag id> <priority>
  With get, returns the priority of the queue entry matching the specified tag. With set, changes the 
  priority of the queue entry to the specified priority.

  PriorityByIndex <tag id> <priority>
  With get, returns the priority of the queue entry matching the specified index into the queue. With 
  set, changes the priority of the queue entry to the specified priority.

  DeleteById <tag id>
  With set, deletes the queue entry matching the specified tag id.

  DeleteByIndex <index>
  With set, deletes the queue entry matching the specified index into the queue.

  PollRate <rate>
  With set, sets the rate at which the scheduler polls for information. The default is 1.0 or one
  second. With get, returns the current poll rate.
*/

// EMC_STAT *emcStatus;

typedef enum {
  cmdHello, cmdSet, cmdGet, cmdQuit, cmdShutdown, cmdHelp, cmdUnknown} commandTokenType;
  
typedef enum {
  scEcho, scVerbose, scEnable, scConfig, scCommMode, scCommProt, scIniFile, scPlat, scIni, scDebug, 
  scQMode, scQStatus, scAutoTagId, scPgmAdd, scPgmById, scPgmByIndex, scPgmAll, scPriorityById, 
  scPriorityByIndex, scDeleteById, scDeleteByIndex, scPollRate, scUnknown} setCommandType;
  
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
  char progName[256];} connectionRecType;

int port = 5008;
int server_sockfd, client_sockfd;
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
float pollDelay = 1.0;

const char *setCommands[] = {
   "ECHO", "VERBOSE", "ENABLE", "CONFIG", "COMM_MODE", "COMM_PROT", "INIFILE", "PLAT", "INI", "DEBUG",
   "QMODE", "QSTATUS", "AUTOTAGID", "PGMADD", "PGMBYID", "PGMBYINDEX", "PGMALL", "PRIORITYBYID", 
   "PRIORITYBYINDEX", "DELETEBYID", "DELETEBYINDEX", "POLLRATE",
   ""};

const char *commands[] = {"HELLO", "SET", "GET", "QUIT", "SHUTDOWN", "HELP", ""};

struct option longopts[] = {
  {"port", 1, NULL, 'p'},
  {"name", 1, NULL, 'n'},
  {"sessions", 1, NULL, 's'},
  {"connectpw", 1, NULL, 'w'},
  {"enablepw", 1, NULL, 'e'},
  {"path", 1, NULL, 'd'},
  {0,0,0,0}
  };


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

static int initSockets()
{
  server_sockfd = socket(AF_INET, SOCK_STREAM, 0);
  server_address.sin_family = AF_INET;
  server_address.sin_addr.s_addr = htonl(INADDR_ANY);
  server_address.sin_port = htons(port);
  server_len = sizeof(server_address);
  bind(server_sockfd, (struct sockaddr *)&server_address, server_len);
  listen(server_sockfd, 5);
  signal(SIGCHLD, SIG_IGN);
  return 0;
}

static void sigQuit(int /*sig*/)
{
    thisQuit();
}

static int sockWrite(connectionRecType *context)
{
   rtapi_strxcat(context->outBuf, "\r\n");
   return write(context->cliSock, context->outBuf, strlen(context->outBuf));
}

static setCommandType lookupSetCommand(char *s)
{
  setCommandType i = scEcho;
  int temp;
  
  while (i < scUnknown) {
    if (strcmp(setCommands[i], s) == 0) return i;
//    (int)i += 1;
      temp = i;
      temp++;
      i = (setCommandType) temp;
    }
  return i;
}

static int commandHello(connectionRecType *context)
{
  char *pch;
  
  pch = strtok(NULL, delims);
  if (pch == NULL) return -1;
  if (strcmp(pch, pwd) != 0) return -1;
  pch = strtok(NULL, delims);
  if (pch == NULL) return -1;
  rtapi_strxcpy(context->hostName, pch);  
  pch = strtok(NULL, delims);
  if (pch == NULL) return -1;
  context->linked = true;    
  rtapi_strxcpy(context->version, pch);
  printf("Connected to %s\n", context->hostName);
  return 0;
}

static int checkOnOff(char *s)
{
  static const char *onStr = "ON";
  static const char *offStr = "OFF";
  
  if (s == NULL) return -1;
  strupr(s);
  if (strcmp(s, onStr) == 0) return 0;
  if (strcmp(s, offStr) == 0) return 1;
  return -1;
}

static int checkBinaryASCII(char *s)
{
  static const char *binaryStr = "BINARY";
  static const char *ASCIIStr = "ASCII";
  
  if (s == NULL) return -1;
  strupr(s);
  if (strcmp(s, ASCIIStr) == 0) return 0;
  if (strcmp(s, binaryStr) == 0) return 1;
  return -1;
}

static queueStatusType checkMode(char *s)
{
  static const char *runStr = "RUN";
  static const char *stopStr = "STOP";
  static const char *pauseStr = "PAUSE";
  static const char *resumeStr = "RESUME";

  if (s == NULL) return qsError;
  strupr(s);
  if (strcmp(s, stopStr) == 0) return qsStop;
  if (strcmp(s, runStr) == 0) return qsRun;
  if (strcmp(s, pauseStr) == 0) return qsPause;
  if (strcmp(s, resumeStr) == 0) return qsResume;
  return qsError;
}

static cmdResponseType setEcho(char *s, connectionRecType *context)
{
   
   switch (checkOnOff(s)) {
     case -1: return rtStandardError;
     case 0: context->echo = true; break;
     case 1: context->echo = false;
     }
   return rtNoError;
}

static cmdResponseType setVerbose(char *s, connectionRecType *context)
{
   
   switch (checkOnOff(s)) {
     case -1: return rtStandardError;
     case 0: context->verbose = true; break;
     case 1: context->verbose = false;
     }
   return rtNoError;
}

static cmdResponseType setEnable(char *s, connectionRecType *context)
{
  
   if (strcmp(s, enablePWD) == 0) {
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

static cmdResponseType setDebug(char *s, connectionRecType * /*context*/)
{
  int level;
  
  if (strlen(s) == 0) return rtStandardError;
  if (sscanf(s, "%i", &level) == -1) return rtStandardError;
  else sendDebug(level);
  return rtNoError;
}

static cmdResponseType setConfig(char * /*s*/, connectionRecType * /*context*/)
{
  return rtNoError;
}

static cmdResponseType setCommMode(char *s, connectionRecType *context)
{
  int ret;
  
  ret = checkBinaryASCII(s);
  if (ret == -1) return rtStandardError;
  context->commMode = ret;
  return rtNoError;
}

static cmdResponseType setCommProt(char * /*s*/, connectionRecType *context)
{
  char *pVersion;
  
  pVersion = strtok(NULL, delims);
  if (pVersion == NULL) return rtStandardError;
  rtapi_strxcpy(context->version, pVersion);
  return rtNoError;
}

static cmdResponseType setQMode(char *s, connectionRecType * /*context*/)
{
  queueStatusType st;

  st = checkMode(s);
  switch (st) {
    case qsStop: queueStop(); break;
    case qsRun: queueStart(); break;
    case qsPause: queuePause(); break;
    case qsResume: queueStart(); break;
    case qsError: ;
    case qsUnknown: ;
    }
  if (st == qsError) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setAutoTagId(char *s, connectionRecType * /*context*/)
{
  int tagId;

  if (strlen(s) == 0) tagId = 0;
  if (sscanf(s, "%d", &tagId) <= 0) return rtStandardError;
  resetTagIds(tagId);
  return rtNoError;
}

static cmdResponseType setPgmAdd(connectionRecType * /*context*/)
{
  char *pch;
  int pri;
  int tag;
  float x, y, z;
  int zone;
  string program;
  float feed, spindle;
  int tool;
  int ret;

  pch = strtok(NULL, delims);
  if (pch == NULL) return rtStandardError;
  if (sscanf(pch, "%d", &pri) <=0) return rtStandardError;
  pch = strtok(NULL, delims);
  if (pch == NULL) return rtStandardError;
  if (sscanf(pch, "%d", &tag) <=0) return rtStandardError;
  if (tag == 0) tag = getNextTagId();
  pch = strtok(NULL, delims);
  if (pch == NULL) return rtStandardError;
  if (sscanf(pch, "%f", &x) <=0) return rtStandardError;
  pch = strtok(NULL, delims);
  if (pch == NULL) return rtStandardError;
  if (sscanf(pch, "%f", &y) <=0) return rtStandardError;
  pch = strtok(NULL, delims);
  if (pch == NULL) return rtStandardError;
  if (sscanf(pch, "%f", &z) <=0) return rtStandardError;
  pch = strtok(NULL, delims);
  if (pch == NULL) return rtStandardError;
  if (sscanf(pch, "%d", &zone) <=0) return rtStandardError;
  pch = strtok(NULL, delims);
  if (pch == NULL) return rtStandardError;
  program.append(pch, strlen(pch));
  pch = strtok(NULL, delims);
  if (pch == NULL) return rtStandardError;
  if (sscanf(pch, "%f", &feed) <=0) return rtStandardError;
  pch = strtok(NULL, delims);
  if (pch == NULL) return rtStandardError;
  if (sscanf(pch, "%f", &spindle) <=0) return rtStandardError;
  pch = strtok(NULL, delims);
  if (pch == NULL) return rtStandardError;
  if (sscanf(pch, "%d", &tool) <=0) return rtStandardError;
  ret = addProgram(pri, tag, x, y, z, zone, program, feed, spindle, tool);
  if (ret == 0) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setPriorityById(connectionRecType * /*context*/)
{
  int id;
  int pri;
  char *pch;

  pch = strtok(NULL, delims);
  if (pch == NULL) return rtStandardError;
  if (sscanf(pch, "%d", &id) <= 0) return rtStandardError;
  pch = strtok(pch, delims);
  if (pch == NULL) return rtStandardError;
  if (sscanf(pch, "%d", &pri) <= 0) return rtStandardError;
  if (changePriorityById(id, pri) == -1) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setPriorityByIndex(connectionRecType * /*context*/)
{
  int index;
  int pri;
  char *pch;

  pch = strtok(NULL, delims);
  if (pch == NULL) return rtStandardError;
  if (sscanf(pch, "%d", &index) <= 0) return rtStandardError;
  pch = strtok(pch, delims);
  if (pch == NULL) return rtStandardError;
  if (sscanf(pch, "%d", &pri) <= 0) return rtStandardError;
  if (changePriorityById(index, pri) == -1) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setDeleteById(char *s, connectionRecType * /*context*/)
{
  int id;

  if (sscanf(s, "%d", &id) <= 0) return rtStandardError;
  if (deleteProgramById(id) == -1) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setDeleteByIndex(char *s, connectionRecType * /*context*/)
{
  int index;

  if (strlen(s) == 0) return rtStandardError;
  if (sscanf(s, "%d", &index) <= 0) return rtStandardError;
  if (deleteProgramByIndex(index) == -1) return rtStandardError;
  return rtNoError;
}

static cmdResponseType setPollRate(char *s, connectionRecType * /*context*/)
{
  float rate;

  if (strlen(s) == 0) return rtStandardError;
  if (sscanf(s, "%f", &rate) <= 0) return rtStandardError;
  pollDelay = rate;
  return rtNoError;
}

int commandSet(connectionRecType *context)
{
  static const char *setNakStr = "SET NAK\n\r";
  static const char *setCmdNakStr = "SET %s NAK\n\r";
  static const char *ackStr = "SET %s ACK\n\r";
  setCommandType cmd;
  char *pch;
  cmdResponseType ret = rtNoError;
  
  pch = strtok(NULL, delims);
  if (pch == NULL) {
    return write(context->cliSock, setNakStr, strlen(setNakStr));
    }
  strupr(pch);
  cmd = lookupSetCommand(pch);
  if ((cmd >= scIniFile) && (context->cliSock != enabledConn)) {
    snprintf(context->outBuf, sizeof(context->outBuf), setCmdNakStr, pch);
    return write(context->cliSock, context->outBuf, strlen(context->outBuf));
    }
  switch (cmd) {
    case scEcho: ret = setEcho(strtok(NULL, delims), context); break;
    case scVerbose: ret = setVerbose(strtok(NULL, delims), context); break;
    case scEnable: ret = setEnable(strtok(NULL, delims), context); break;
    case scConfig: ret = setConfig(strtok(NULL, delims), context); break;
    case scCommMode: ret = setCommMode(strtok(NULL, delims), context); break;
    case scCommProt: ret = setCommProt(strtok(NULL, delims), context); break;
    case scIniFile: break;
    case scPlat: break;
    case scIni: break;
    case scDebug: ret = setDebug(strtok(NULL, delims), context); break;
    case scQMode: ret = setQMode(strtok(NULL, delims), context); break;
    case scQStatus: break;
    case scAutoTagId: ret = setAutoTagId(strtok(NULL, delims), context); break;
    case scPgmAdd: ret = setPgmAdd(context); break;
    case scPgmById: break;
    case scPgmByIndex: break;
    case scPgmAll: break;
    case scPriorityById: ret = setPriorityById(context); break;
    case scPriorityByIndex: ret = setPriorityByIndex(context); break;
    case scDeleteById: ret = setDeleteById(strtok(NULL, delims), context); break;
    case scDeleteByIndex: ret = setDeleteByIndex(strtok(NULL, delims), context); break;
    case scPollRate: ret = setPollRate(strtok(NULL, delims), context); break;
    case scUnknown: ret = rtStandardError;
    }
  switch (ret) {
    case rtNoError:  
      if (context->verbose) {
        snprintf(context->outBuf, sizeof(context->outBuf), ackStr, pch);
        return write(context->cliSock, context->outBuf, strlen(context->outBuf));
        }
      break;
    case rtHandledNoError: // Custom ok response already handled, take no action
      break; 
    case rtStandardError:
      snprintf(context->outBuf, sizeof(context->outBuf), setCmdNakStr, pch);
      return write(context->cliSock, context->outBuf, strlen(context->outBuf));
      break;
    case rtCustomError: // Custom error response entered in buffer
      return write(context->cliSock, context->outBuf, strlen(context->outBuf));
      break;
    case rtCustomHandledError: ;// Custom error response handled, take no action
    }
  return 0;
}

static cmdResponseType getEcho(char * /*s*/, connectionRecType *context)
{
  const char *pEchoStr = "ECHO %s";
  
  if (context->echo) snprintf(context->outBuf, sizeof(context->outBuf), pEchoStr, "ON");
  else snprintf(context->outBuf, sizeof(context->outBuf), pEchoStr, "OFF");
  return rtNoError;
}

static cmdResponseType getVerbose(char * /*s*/, connectionRecType *context)
{
  const char *pVerboseStr = "VERBOSE %s";
  
  if (context->verbose) snprintf(context->outBuf, sizeof(context->outBuf), pVerboseStr, "ON");
  else snprintf(context->outBuf, sizeof(context->outBuf), pVerboseStr, "OFF");
  return rtNoError;
}

static cmdResponseType getEnable(char * /*s*/, connectionRecType *context)
{
  const char *pEnableStr = "ENABLE %s";
  
  if (context->cliSock == enabledConn) 
//  if (context->enabled == true)
    snprintf(context->outBuf, sizeof(context->outBuf), pEnableStr, "ON");
  else snprintf(context->outBuf, sizeof(context->outBuf), pEnableStr, "OFF");
  return rtNoError;
}

static cmdResponseType getConfig(char * /*s*/, connectionRecType *context)
{
  const char *pConfigStr = "CONFIG";

  rtapi_strxcpy(context->outBuf, pConfigStr);
  return rtNoError;
}

static cmdResponseType getCommMode(char * /*s*/, connectionRecType *context)
{
  const char *pCommModeStr = "COMM_MODE %s";
  
  switch (context->commMode) {
    case 0: snprintf(context->outBuf, sizeof(context->outBuf), pCommModeStr, "ASCII"); break;
    case 1: snprintf(context->outBuf, sizeof(context->outBuf), pCommModeStr, "BINARY"); break;
    }
  return rtNoError;
}

static cmdResponseType getCommProt(char * /*s*/, connectionRecType *context)
{
  const char *pCommProtStr = "COMM_PROT %s";
  
  snprintf(context->outBuf, sizeof(context->outBuf), pCommProtStr, context->version);
  return rtNoError;
}

static cmdResponseType getDebug(char * /*s*/, connectionRecType *context)
{
  const char *pUpdateStr = "DEBUG %d";
  
  snprintf(context->outBuf, sizeof(context->outBuf), pUpdateStr, emcStatus->debug);
  return rtNoError;
}

static cmdResponseType getIniFile(char * /*s*/, connectionRecType *context)
{
  const char *pIniFile = "INIFILE %s";
  
  snprintf(context->outBuf, sizeof(context->outBuf), pIniFile, emc_inifile);
  return rtNoError;
}

static cmdResponseType getPlat(char * /*s*/, connectionRecType *context)
{
  const char *pPlatStr = "PLAT %s";
  
  snprintf(context->outBuf, sizeof(context->outBuf), pPlatStr, "Linux");
  return rtNoError;  
}

static cmdResponseType getQMode(char * /*s*/, connectionRecType *context)
{
  const char *pQMode = "QMODE %s";

  switch (getStatus()) {
    case qsStop: snprintf(context->outBuf, sizeof(context->outBuf), pQMode, "STOP"); break;
    case qsRun: snprintf(context->outBuf, sizeof(context->outBuf), pQMode, "RUN"); break;
    case qsPause: snprintf(context->outBuf, sizeof(context->outBuf), pQMode, "PAUSE"); break;
    case qsError: snprintf(context->outBuf, sizeof(context->outBuf), pQMode, "ERROR"); break;
    case qsUnknown: ;
    }
  return rtNoError;
}

static cmdResponseType getQStatus(connectionRecType *context)
{
  const char *pQStatus = "QSTATUS %d %d %d %d";

  snprintf(context->outBuf, sizeof(context->outBuf), pQStatus, getQueueSize(), getFirstTagId(), getLastTagId(), getQueueCRC());
  return rtNoError;
}

static cmdResponseType getTagId(connectionRecType *context)
{
  const char *pTagId = "AUTOTAGID %d";

  snprintf(context->outBuf, sizeof(context->outBuf), pTagId, getNextTagId());
  return rtNoError;
}

static cmdResponseType getPgmById(char *s, connectionRecType *context)
{
  qRecType qRec;
  int id;
  
  if (strlen(s) == 0) return rtStandardError;
  if (sscanf(s, "%d", &id) <= 0) return rtStandardError;
  if (getProgramById(id, &qRec) != 0) return rtStandardError;
  snprintf(context->outBuf, sizeof(context->outBuf), "PGMBYID %d %d %f %f %f %d %s %f %f %d", 
    qRec.priority, qRec.tagId, qRec.xpos, qRec.ypos, qRec.zpos, qRec.zone, qRec.fileName,
    qRec.feedOverride, qRec.spindleOverride, qRec.tool);
  return rtNoError;
}

static cmdResponseType getPgmByIndex(char *s, connectionRecType *context)
{
  qRecType qRec;
  int index;

  if (strlen(s) == 0) return rtStandardError;  
  if (sscanf(s, "%d", &index) <= 0) return rtStandardError;
  if (getProgramByIndex(index, &qRec) != 0) return rtStandardError;
  snprintf(context->outBuf, sizeof(context->outBuf), "PGMBYINDEX %d %d %f %f %f %d %s %f %f %d", 
    qRec.priority, qRec.tagId, qRec.xpos, qRec.ypos, qRec.zpos, qRec.zone, qRec.fileName,
    qRec.feedOverride, qRec.spindleOverride, qRec.tool);
  return rtNoError;
}

static cmdResponseType getPgmAll(connectionRecType *context)
{
  qRecType qRec;
  int i;
  int sz;

  sz = getQueueSize();
  for (i = 0; i < sz; i++) {
    if (getProgramByIndex(i, &qRec) != 0) continue;
    snprintf(context->outBuf, sizeof(context->outBuf), "PGMBYINDEX %d %d %f %f %f %d %s %f %f %d", 
      qRec.priority, qRec.tagId, qRec.xpos, qRec.ypos, qRec.zpos, qRec.zone, qRec.fileName,
      qRec.feedOverride, qRec.spindleOverride, qRec.tool);
    sockWrite(context);
    }
  return rtHandledNoError;
}

static cmdResponseType getPriById(char *s, connectionRecType *context)
{
  int id;
  int pri;

  if (sscanf(s, "%d", &id) <= 0) return rtStandardError;
  if (getPriorityById(id, pri) == -1) return rtStandardError;
  snprintf(context->outBuf, sizeof(context->outBuf), "PRIORITYBYID %d %d", id, pri);
  return rtNoError;
}

static cmdResponseType getPriByIndex(char *s, connectionRecType *context)
{
  int index;
  int pri;

  if (sscanf(s, "%d", &index) <= 0) return rtStandardError;
  if (getPriorityByIndex(index, pri) == -1) return rtStandardError;
  snprintf(context->outBuf, sizeof(context->outBuf), "PRIORITYBYINDEX %d %d", index, pri);
  return rtNoError;
}

static cmdResponseType getPollRate(connectionRecType *context)
{
  snprintf(context->outBuf, sizeof(context->outBuf), "POLLRATE %f", pollDelay);
  return rtNoError;
}

int commandGet(connectionRecType *context)
{
  static const char *setNakStr = "GET NAK\r\n";
  static const char *setCmdNakStr = "GET %s NAK\r\n";
  setCommandType cmd;
  char *pch;
  cmdResponseType ret = rtNoError;
  
  pch = strtok(NULL, delims);
  if (pch == NULL) {
    return write(context->cliSock, setNakStr, strlen(setNakStr));
    }
  if (emcUpdateType == EMC_UPDATE_AUTO) updateStatus();
  strupr(pch);
  cmd = lookupSetCommand(pch);
  if (cmd > scIni)
    if (emcUpdateType == EMC_UPDATE_AUTO) updateStatus();
  switch (cmd) {
    case scEcho: ret = getEcho(pch, context); break;
    case scVerbose: ret = getVerbose(pch, context); break;
    case scEnable: ret = getEnable(pch, context); break;
    case scConfig: ret = getConfig(pch, context); break;
    case scCommMode: ret = getCommMode(pch, context); break;
    case scCommProt: ret = getCommProt(pch, context); break;
    case scIniFile: ret = getIniFile(pch, context); break;
    case scPlat: ret = getPlat(pch, context); break;
    case scIni: break;
    case scDebug: ret = getDebug(pch, context); break;
    case scQMode: ret = getQMode(pch, context); break;
    case scQStatus: ret = getQStatus(context); break;
    case scAutoTagId: ret = getTagId(context);  break;
    case scPgmAdd: break;
    case scPgmById: ret = getPgmById(strtok(NULL, delims), context); break;
    case scPgmByIndex: ret = getPgmByIndex(strtok(NULL, delims), context); break;
    case scPgmAll: ret = getPgmAll(context); break;
    case scPriorityById: ret = getPriById(strtok(NULL, delims), context); break;
    case scPriorityByIndex: ret = getPriByIndex(strtok(NULL, delims), context); break;
    case scDeleteById: break;
    case scDeleteByIndex: break;
    case scPollRate: ret = getPollRate(context); break;
    case scUnknown: ret = rtStandardError;
    }
  switch (ret) {
    case rtNoError: // Standard ok response, just write value in buffer
      sockWrite(context);
      break;
    case rtHandledNoError: // Custom ok response already handled, take no action
      break; 
    case rtStandardError: // Standard error response
      snprintf(context->outBuf, sizeof(context->outBuf), setCmdNakStr, pch); 
      sockWrite(context);
      break;
    case rtCustomError: // Custom error response entered in buffer
      sockWrite(context);
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
  snprintf(context->outBuf, sizeof(context->outBuf), "Available commands:\n\r");
  rtapi_strxcat(context->outBuf, "  Hello <password> <client name> <protocol version>\n\r");
  rtapi_strxcat(context->outBuf, "  Get <emc command>\n\r");
  rtapi_strxcat(context->outBuf, "  Set <emc command>\n\r");
  rtapi_strxcat(context->outBuf, "  Shutdown\n\r");
  rtapi_strxcat(context->outBuf, "  Help <command>\n\r");
  sockWrite(context);
  return 0;
}

static int helpHello(connectionRecType *context)
{
  snprintf(context->outBuf, sizeof(context->outBuf), "Usage:\n\r");
  rtapi_strxcat(context->outBuf, "  Hello <Password> <Client Name> <Protocol Version>\n\rWhere:\n\r");
  rtapi_strxcat(context->outBuf, "  Password is the connection password to allow communications with the CNC server.\n\r");
  rtapi_strxcat(context->outBuf, "  Client Name is the name of client trying to connect, typically the network name of the client.\n\r");
  rtapi_strxcat(context->outBuf, "  Protocol Version is the version of the protocol with which the client wishes to use.\n\r\n\r");
  rtapi_strxcat(context->outBuf, "  With valid password, server responds with:\n\r");
  rtapi_strxcat(context->outBuf, "  Hello Ack <Server Name> <Protocol Version>\n\rWhere:\n\r");
  rtapi_strxcat(context->outBuf, "  Ack is acknowledging the connection has been made.\n\r");
  rtapi_strxcat(context->outBuf, "  Server Name is the name of the EMC Server to which the client has connected.\n\r");
  rtapi_strxcat(context->outBuf, "  Protocol Version is the client requested version or latest version support by server if");
  rtapi_strxcat(context->outBuf, "  the client requests a version later than that supported by the server.\n\r\n\r");
  rtapi_strxcat(context->outBuf, "  With invalid password, the server responds with:\n\r");
  rtapi_strxcat(context->outBuf, "  Hello Nak\n\r");
  sockWrite(context);
  return 0;
}

static int helpGet(connectionRecType *context)
{
  snprintf(context->outBuf, sizeof(context->outBuf), "Usage:\n\rGet <emc command>\n\r");
  rtapi_strxcat(context->outBuf, "  Get commands require that a hello has been successfully negotiated.\n\r");
  rtapi_strxcat(context->outBuf, "  Emc command may be one of:\n\r");
  rtapi_strxcat(context->outBuf, "    AutoTagId\n\r");
  rtapi_strxcat(context->outBuf, "    Comm_mode\n\r");
  rtapi_strxcat(context->outBuf, "    Comm_prot\n\r");
  rtapi_strxcat(context->outBuf, "    Debug\n\r");
  rtapi_strxcat(context->outBuf, "    Echo\n\r");
  rtapi_strxcat(context->outBuf, "    Enable\n\r");
  rtapi_strxcat(context->outBuf, "    Inifile\n\r");
  rtapi_strxcat(context->outBuf, "    PgmById <Tag Id>\n\r");
  rtapi_strxcat(context->outBuf, "    PgmByIndex <Index>\n\r");
  rtapi_strxcat(context->outBuf, "    PriorityById <Tag Id>\n\r");
  rtapi_strxcat(context->outBuf, "    PriorityByIndex <Tag Index>\n\r");
  rtapi_strxcat(context->outBuf, "    Plat\n\r");
  rtapi_strxcat(context->outBuf, "    QMode\n\r");
  rtapi_strxcat(context->outBuf, "    QStatus\n\r");
  rtapi_strxcat(context->outBuf, "    Verbose\n\r");
//  rtapi_strxcat(context->outBuf, "CONFIG\n\r");
  sockWrite(context);
  return 0;
}

static int helpSet(connectionRecType *context)
{
  snprintf(context->outBuf, sizeof(context->outBuf), "Usage:\n\r  Set <emc command>\n\r");
  rtapi_strxcat(context->outBuf, "  Set commands require that a hello has been successfully negotiated,\n\r");
  rtapi_strxcat(context->outBuf, "  in most instances requires that control be enabled by the connection.\n\r");
  rtapi_strxcat(context->outBuf, "  The set commands not requiring control enabled are:\n\r");
  rtapi_strxcat(context->outBuf, "    Comm_mode <mode>\n\r");
  rtapi_strxcat(context->outBuf, "    Comm_prot <protocol>\n\r");
  rtapi_strxcat(context->outBuf, "    Echo <On | Off>\n\r");
  rtapi_strxcat(context->outBuf, "    Enable <Pwd | Off>\n\r");
  rtapi_strxcat(context->outBuf, "    Verbose <On | Off>\n\r\n\r");
  rtapi_strxcat(context->outBuf, "  The set commands requiring control enabled are:\n\r");
  rtapi_strxcat(context->outBuf, "    AutoTagId <Start Id>\n\r");
  rtapi_strxcat(context->outBuf, "    PgmAdd <Priority> <Tag Id> <X> <Y> <Z> <Zone> <File Name> <Feed Override> <Spindle Override> <Tool No>\n\r");
  rtapi_strxcat(context->outBuf, "    PriorityById <Tag Id> <Priority>\n\r");
  rtapi_strxcat(context->outBuf, "    PriorityByIndex <Index> <Priority>\n\r");
  rtapi_strxcat(context->outBuf, "    DeleteById <Tag Id> \n\r");
  rtapi_strxcat(context->outBuf, "    DeleteByIndex <Index> \n\r");
  rtapi_strxcat(context->outBuf, "    QMode <stop | run | pause | resume>\n\r");
 
  sockWrite(context);
  return 0;
}

static int helpQuit(connectionRecType *context)
{
  snprintf(context->outBuf, sizeof(context->outBuf), "Usage:\n\r");
  rtapi_strxcat(context->outBuf, "  The quit command has the server initiate a disconnect from the client,\n\r");
  rtapi_strxcat(context->outBuf, "  the command has no parameters and no requirements to have negotiated\n\r");
  rtapi_strxcat(context->outBuf, "  a hello, or be in control.");
  sockWrite(context);
  return 0;
}

static int helpShutdown(connectionRecType *context)
{
  snprintf(context->outBuf, sizeof(context->outBuf), "Usage:\n\r");
  rtapi_strxcat(context->outBuf, "  The shutdown command terminates the connection with all clients,\n\r");
  rtapi_strxcat(context->outBuf, "  and initiates a shutdown of EMC. The command has no parameters, and\n\r");
  rtapi_strxcat(context->outBuf, "  can only be issued by the connection having control.\n\r");
  sockWrite(context);
  return 0;
}

static int helpHelp(connectionRecType *context)
{
  snprintf(context->outBuf, sizeof(context->outBuf), "If you need help on help, it is time to look into another line of work.\n\r");
  sockWrite(context);
  return 0;
}

int commandHelp(connectionRecType *context)
{
  char *pch;
  
  pch = strtok(NULL, delims);
  if (pch == NULL) return (helpGeneral(context));
  strupr(pch);
  if (strcmp(pch, "HELLO") == 0) return (helpHello(context));
  if (strcmp(pch, "GET") == 0) return (helpGet(context));
  if (strcmp(pch, "SET") == 0) return (helpSet(context));
  if (strcmp(pch, "QUIT") == 0) return (helpQuit(context));
  if (strcmp(pch, "SHUTDOWN") == 0) return (helpShutdown(context));
  if (strcmp(pch, "HELP") == 0) return (helpHelp(context));
  snprintf(context->outBuf, sizeof(context->outBuf), "%s is not a valid command.", pch);
  sockWrite(context);
  return 0;
}

commandTokenType lookupToken(char *s)
{
  commandTokenType i = cmdHello;
  int temp;
  
  while (i < cmdUnknown) {
    if (strcmp(commands[i], s) == 0) return i;
//    (int)i += 1;
    temp = i;
    temp++;
    i = (commandTokenType) temp;
    }
  return i;
}
  
int parseCommand(connectionRecType *context)
{
  int ret = 0;
  char *pch;
  char s[64];
  static const char *helloNakStr = "HELLO NAK\r\n";
  static const char *shutdownNakStr = "SHUTDOWN NAK\r\n";
  static const char *helloAckStr = "HELLO ACK %s 1.1\r\n";
  static const char *setNakStr = "SET NAK\r\n";
    
  pch = strtok(context->inBuf, delims);
  snprintf(s, sizeof(s), helloAckStr, serverName);
  if (pch != NULL) {
    strupr(pch);
    switch (lookupToken(pch)) {
      case cmdHello: 
        if (commandHello(context) == -1)
          ret = write(context->cliSock, helloNakStr, strlen(helloNakStr));
        else ret = write(context->cliSock, s, strlen(s));
        break;
      case cmdGet: 
        ret = commandGet(context);
        break;
      case cmdSet:
        if (!context->linked)
	  ret = write(context->cliSock, setNakStr, strlen(setNakStr));
        else ret = commandSet(context);
        break;
      case cmdQuit: 
        ret = commandQuit(context);
        break;
      case cmdShutdown:
        ret = commandShutdown(context);
        if(ret ==0){
          ret = write(context->cliSock, shutdownNakStr, strlen(shutdownNakStr));
        }
	break;
      case cmdHelp:
        ret = commandHelp(context);
	break;
      case cmdUnknown: ret = -2;
      }
    }
  return ret;
}

void *checkQueue(void * /*arg*/)
{
  while (1) {
    updateQueue();
    sleep((unsigned)pollDelay);
    }
  return 0;
}  

void *readClient(void * /*arg*/)
{
  char str[1600];
  char buf[1600];
  unsigned int i, j;
  int len;
  connectionRecType *context;
  
  
//  res = 1;
  context = (connectionRecType *) malloc(sizeof(connectionRecType));
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
  buf[0] = 0;
  
  while (1) {
    len = read(context->cliSock, &str, 1600);
    if (len <= 0) goto finished;
    str[len] = 0;
    rtapi_strxcat(buf, str);
    if (!memchr(str, 0x0d, strlen(str))) continue;
    if (context->echo && context->linked)
      if(write(context->cliSock, buf, strlen(buf)) != (ssize_t)strlen(buf)) {
        fprintf(stderr, "emcrsh: write() failed: %s", strerror(errno));
      }
    i = 0;
    j = 0;
    while (i <= strlen(buf)) {
      if ((buf[i] != '\n') && (buf[i] != '\r')) {
        context->inBuf[j] = buf[i];
	j++;
      }
      else if (j > 0)
      {
        context->inBuf[j] = 0;
        if (parseCommand(context) == -1) goto finished;
        j = 0;
      }
      i++;
    }
  buf[0] = 0;
  }

finished:
  close(context->cliSock);
  free(context);
  pthread_exit((void *)0);
  sessions--;  // FIXME: not reached
}

int sockMain()
{
    pthread_t thrd;
    int res;
    
    while (1) {
      
      client_len = sizeof(client_address);
      client_sockfd = accept(server_sockfd,
        (struct sockaddr *)&client_address, &client_len);
      if (client_sockfd < 0) exit(0);
      sessions++;
      if ((maxSessions == -1) || (sessions <= maxSessions))
        res = pthread_create(&thrd, NULL, readClient, (void *)NULL);
      else res = -1;
      if (res != 0) {
        close(client_sockfd);
        sessions--;
        }
     }
    return 0;
}

static void initMain()
{
    emcWaitType = EMC_WAIT_RECEIVED;
    emcCommandSerialNumber = 0;
    emcTimeout = 0.0;
    emcUpdateType = EMC_UPDATE_AUTO;
    linearUnitConversion = LINEAR_UNITS_AUTO;
    angularUnitConversion = ANGULAR_UNITS_AUTO;
    emcCommandBuffer = 0;
    emcStatusBuffer = 0;
    emcStatus = 0;

    emcErrorBuffer = 0;
    error_string[LINELEN-1] = 0;
    operator_text_string[LINELEN-1] = 0;
    operator_display_string[LINELEN-1] = 0;
    programStartLine = 0;
}


int main(int argc, char *argv[])
{
    int opt;
    pthread_t updateThread;
    int res;

    initMain();
    // process local command line args
    while((opt = getopt_long(argc, argv, "e:n:p:s:w:", longopts, NULL)) != -1) {
      switch(opt) {
        case 'e': snprintf(enablePWD, sizeof(enablePWD), "%s", optarg); break;
        case 'n': snprintf(serverName, sizeof(serverName), "%s", optarg); break;
        case 'p': sscanf(optarg, "%d", &port); break;
        case 's': sscanf(optarg, "%d", &maxSessions); break;
        case 'w': snprintf(pwd, sizeof(pwd), "%s", optarg); break;
        case 'd': snprintf(defaultPath, sizeof(defaultPath), "%s", optarg); break;
        }
      }

    // process emc command line args
    if (emcGetArgs(argc, argv) != 0) {
	rcs_print_error("error in argument list\n");
	exit(1);
    }
    // get configuration information
    iniLoad(emc_inifile);
    initSockets();
    // init NML
    if (tryNml() != 0) {
	rcs_print_error("can't connect to emc\n");
	thisQuit();
	exit(1);
    }
    // get current serial number, and save it for restoring when we quit
    // so as not to interfere with real operator interface
    updateStatus();
    emcCommandSerialNumber = emcStatus->echo_serial_number;

    // attach our quit function to SIGINT
    signal(SIGINT, sigQuit);

    schedInit();
    res = pthread_create(&updateThread, NULL, checkQueue, (void *)NULL);
    if(res != 0) { perror("pthread_create"); return 1; }
    if (useSockets) sockMain();

    return 0;
}
