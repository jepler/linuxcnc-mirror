/********************************************************************
* Description: halrmt.cc
*   Simple telnet interface to LinuxCNC HAL commands (halcmd)
*
*   Derived from work by jmkasunich
*
*   Other contributors:
*     Alex Joni
*
* Author: Eric H. Johnson
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2006-2008 All rights reserved.
*
* Last change:
********************************************************************/

/*******************************************************************
  Using halrmt:

  halrmt {-- --port <port number> --name <server name> --connectpw <password>
             --enablepw <password> --sessions <max sessions> -ini <INI file>}

  With -- --port Waits for socket connections (Telnet) on specified socket, without port
            uses default port 5006. (note: linuxcncrsh uses 5007 as default)
  With -- --name <server name> Sets the server name to specified name for Hello.
  With -- --connectpw <password> Sets the connection password to 'password'. Default EMC
  With -- --enablepw <password> Sets the enable password to 'password'. Default EMCTOO
  With -- --sessions <max sessions> Sets the maximum number of simultaneous connextions
            to max sessions. Default is no limit (-1).
  With -- -ini <INI file>, uses specified INI file instead of default emc.ini. 

  There are six commands supported, Where the commands set and get contain HAL
  specific sub-commands based on the commands supported by halcmd. Commands and 
  most parameters are not case sensitive. The exceptions are passwords, 
  file paths and text strings.
  
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
  
  The get command includes one of the hal sub-commands, described below and
  zero or more additional parameters. 
  
  ==> Set <==
  
  The set command inclides one of the hal sub-commands, described below and
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
  
  
  HAL sub-commands:
  
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

  Comps [<substring>]

  Get only, returns all components beginning with the specified substring. 
  If no substring is specified then it returns all components.

  Pins [<substring>]

  Get only, returns all information about all pins beginning with the
  specified substring. If no substring is specified then it returns all pins.

  PinVals [<substring>]

  Get only, returns only value information about all pins beginning with the
  specified substring. If no substring is specified then it returns all pins.

  Signals [<substring>]

  Get only, returns all information about all signals beginning with the
  specified substring. If no substring is specified then it returns all signals.

  SigVals [<substring>]

  Get only, returns only value information about all signals beginning with the
  specified substring. If no substring is specified then it returns all pins.

  Params [<substring>]

  Get only, returns all information about all parameters beginning with the
  specified substring. If no substring is specified then it returns all 
  parameters.

  ParamVals [<substring>]

  Get only, returns only value information about all parameters beginning with the
  specified substring. If no substring is specified then it returns all pins
  parameters.

  Functs [<substring>]

  Get only, returns all information about all functions beginning with the
  specified substring. If no substring is specified then it returns all 
  functions.
  
  Threads

  Get only, returns all information about all functions.
 
  Comp <name>

  Get only, returns the component matching the specified name.

  Pin <name>

  Get only, returns all information about the pin matching the specified 
  name.

  PinVal <name>

  Get only, returns the value of the pin matching the specified name.

  Sig <name>

  Get only, returns all information about the pin matching the specified 
  name.

  SigVal <name>

  Get only, returns just the value of the signal matching the specified
  name.
  
  Param <name>

  Get only, returns all information about the parameter matching the 
  specified name.

  ParamVal <name>

  Get only, returns just the value of the parameter matching the specified
  name.

  Funct <name>

  Get only, returns all information about the parameter matching the 
  specified name.

  Thread <name>

  Get only, returns all information about the thread matching the 
  specified name.

  LoadRt <name>

  Set only, loads the real time executable specified by name.

  Unload <name>

  Set only, unloads the executable specified by name.

  LoadUsr <name>

  Set only, loads the user executable specified by name.

  Linkps <pin name> <signal name>

  Set only, links the specified pin to the specified signal.

  Linksp <signal name> <pin name>

  Set only, links the specified signal to the specified pin.

  Linkpp <pin name 1> <pin name 2>

  Set only, links the pin specified by pin 1 with the pin specified by pin 2.

  Net <net list>

  Set only, nets the specified net list.

  Unlinkp <pin name 1> <pin name 2>

  Set only, unlinks the specified pins

  Lock

  Unlock

  NewSig <name> <type>

  Set only, creates the signal specified by name and of type specified by type.

  DelSig <name>

  Set only, deletes the signal specified by name.

  SetP <name> <value>

  Set only, sets the parameter specified by name to the value specified by value.

  SetS <name> <value>

  Set only, sets the signal specified by name to the value specified by value.

  AddF <name> <thread> [<parameters>]

  Set only, adds the function specified by name, to the thread specified by
  thread, with the optional parameters specified by parameters.

  DelF <name>

  Set only, deletes the function specified by name.

  Save 

  Start

  Stop

****************************************************************************/

#include "config.h"

#define _REENTRANT

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/uio.h>
#include <pthread.h>
#include <fnmatch.h>
#include <getopt.h>

#include "rtapi.h"		// RTAPI realtime OS API
#include <rtapi_mutex.h>
#include <rtapi_string.h>	// rtapi_strlcpy
#include "hal.h"		// HAL public API decls
#include "../hal_priv.h"	// private HAL decls
/* non-EMC related uses of halrmt may want to avoid libnml dependency
#ifndef NO_INI
#include "inifile.h"		// iniFind() from libnml
#endif
#include <rtapi_string.h>
*/

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

/* These functions are used internally by this file.  The code is at
   the end of the file.  */

#define MAX_TOK 20
#define MAX_CMD_LEN 1024
#define MAX_EXPECTED_SIGS 999

static int release_HAL_mutex(void);
static int do_help_cmd(char *command);
static int unloadrt_comp(char *mod_name);
static const char *data_type(int type);
static const char *pin_data_dir(int dir);
static const char *param_data_dir(int dir);
static const char *data_arrow1(int dir);
static const char *data_arrow2(int dir);
static char *data_value(int type, void *valptr);
static char *data_value2(int type, void *valptr);
static void save_comps(FILE *dst);
static void save_signals(FILE *dst);
static void save_links(FILE *dst, int arrows);
static void save_nets(FILE *dst, int arrows);
static void save_params(FILE *dst);
static void save_threads(FILE *dst);
static void print_help_general(int showR);

/***********************************************************************
*                         GLOBAL VARIABLES                             *
************************************************************************/

int comp_id = -1;	/* -1 means hal_init() not called yet */
int hal_flag = 0;	/* used to indicate that halrmt might have the
			   hal mutex, so the sig handler can't just
			   exit, instead it must set 'done' */
int done = 0;		/* used to break out of processing loop */
int linenumber=0;	/* used to print linenumber on errors */
int scriptmode = 0;	/* used to make output "script friendly" (suppress headers) */
int prompt_mode = 0;	/* when getting input from stdin, print a prompt */
char comp_name[HAL_NAME_LEN+1];	/* name for this instance of halrmt */

char pwd[16] = "EMC\0";              // Connect password
char enablePWD[16] = "EMCTOO\0";     // Enable password
char serverName[24] = "EMCNETSVR\0"; // Server name written in hello response
int sessions = 0;                    // Number of open sessions
int maxSessions = -1;                // Maximum number of sessions to allow

typedef struct {  
  int cliSock;
  char hostName[80];
  char version[8];
  int linked;
  int echo;
  int verbose;
  int enabled;
  int commMode;
  int commProt;
  char inBuf[256];
  char outBuf[4096];
  char progName[256];} connectionRecType;


int port = 5006;
char errorStr[256];

int server_sockfd, client_sockfd;
socklen_t server_len, client_len;
struct sockaddr_in server_address;
struct sockaddr_in client_address;
int useSockets = 1;
int tokenIdx;
const char *delims = " \n\r\0";
int connCount = -1;
int enabledConn = -1;

typedef enum {
  cmdHello, cmdSet, cmdGet, cmdQuit, cmdShutdown, cmdHelp, cmdUnknown} commandTokenType;
  
typedef enum {
  hcEcho, hcVerbose, hcEnable, hcConfig, hcCommMode, hcCommProt,
  hcComps, hcPins, hcPinVals, hcSigs, hcSigVals, hcParams, hcParamVals, hcFuncts, hcThreads,
  hcComp, hcPin, hcPinVal, hcSig, hcSigVal, hcParam, hcParamVal, hcFunct, hcThread,
  hcLoadRt, hcUnload, hcLoadUsr, hcLinkps, hcLinksp, hcLinkpp, hcNet, hcUnlinkp,
  hcLock, hcUnlock, hcNewSig, hcDelSig, hcSetP, hcSetS, hcAddF, hcDelF,
  hcSave, hcStart, hcStop, hcUnknown
  } halCommandType;
  
typedef enum {
  rtNoError, rtHandledNoError, rtStandardError, rtCustomError, rtCustomHandledError
  } cmdResponseType;  

struct option longopts[] = {
  {"port", 1, NULL, 'p'},
  {"name", 1, NULL, 'n'},
  {"sessions", 1, NULL, 's'},
  {"connectpw", 1, NULL, 'w'},
  {"enablepw", 1, NULL, 'e'},
  {0,0,0,0}
};

const char *commands[] = {"HELLO", "SET", "GET", "QUIT", "SHUTDOWN", "HELP", ""};
const char *halCommands[] = {
  "ECHO", "VERBOSE", "ENABLE", "CONFIG", "COMM_MODE", "COMM_PROT",
  "COMPS", "PINS", "PINVALS", "SIGNALS", "SIGVALS", "PARAMS", "PARAMVALS", "FUNCTS", "THREADS",
  "COMP", "PIN", "PINVAL", "SIGNAL", "SIGVAL", "PARAM", "PARAMVAL", "FUNCT", "THREAD",
  "LOADRT", "UNLOAD", "LOADUSR", "LINKPS", "LINKSP", "LINKPP", "NET", "UNLINKP",
  "LOCK", "UNLOCK", "NEWSIG", "DELSIG", "SETP", "SETS", "ADDF", "DELF",
  "SAVE", "START", "STOP", ""};

#ifndef NO_INI
    FILE *inifile = NULL;
#endif


/* signal handler */
static void quit(int sig)
{
    (void)sig;
    if ( hal_flag ) {
	/* this process might have the hal mutex, so just set the
	   'done' flag and return, exit after mutex work finishes */
	done = 1;
    } else {
	/* don't have to worry about the mutex, but if we just
	   return, we might return into the fgets() and wait 
	   all day instead of exiting.  So we exit from here. */
	if ( comp_id > 0 ) {
	    hal_exit(comp_id);
	}
	_exit(1);
    }
}

static void strupr(char *s)
{  
  int i;
  
  for (i = 0; i < (int)strlen(s); i++)
    if (s[i] > 96 && s[i] <= 'z')
      s[i] -= 32;
}

static int initSockets()
{
  int optval = 1;

  server_sockfd = socket(AF_INET, SOCK_STREAM, 0);
  setsockopt(server_sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
  server_address.sin_family = AF_INET;
  server_address.sin_addr.s_addr = htonl(INADDR_ANY);
  server_address.sin_port = htons(port);
  server_len = sizeof(server_address);
  bind(server_sockfd, (struct sockaddr *)&server_address, server_len);
  listen(server_sockfd, 5);
  signal(SIGCHLD, SIG_IGN);
  return 0;
}

static int sockWrite(connectionRecType *context)
{
   rtapi_strxcat(context->outBuf, "\r\n");
   return write(context->cliSock, context->outBuf, strlen(context->outBuf));
}

static void sockWriteError(const char *nakStr, connectionRecType *context)
{
  if (context->verbose == 1)
    snprintf(context->outBuf, sizeof(context->outBuf), "%s %s", nakStr, errorStr);
  else
    snprintf(context->outBuf, sizeof(context->outBuf), "%s", nakStr);
  if(sockWrite(context) < 0) perror("sockWrite");
}

pid_t hal_systemv_nowait(char *const argv[], connectionRecType *context) {
    pid_t pid;
    int n;
    const char *nakStr = "SET LOADRT NAK";

    /* now we need to fork, and then exec .... */
    /* disconnect from the HAL shmem area before forking */
    hal_exit(comp_id);
    comp_id = 0;
    /* now the fork() */
    pid = fork();
    if ( pid < 0 ) {
	/* fork failed */
//	halcmd_error("fork() failed\n");
	/* reconnect to the HAL shmem area */
	comp_id = hal_init(comp_name);
	if (comp_id < 0) {
	    fprintf(stderr, "halcmd: hal_init() failed after fork: %d\n",
                    comp_id );
	    exit(-1);
	}
        hal_ready(comp_id);
	return -1;
    }
    if ( pid == 0 ) {
	/* child process */
	/* print debugging info if "very verbose" (-V) */
        for(n=0; argv[n] != NULL; n++) {
	    rtapi_print_msg(RTAPI_MSG_DBG, "%s ", argv[n] );
	}
        if (n == 0) {
            snprintf(errorStr, sizeof(errorStr), "hal_systemv_nowait: empty argv array passed in\n");
            sockWriteError(nakStr, context);
            exit(1);
        }
	rtapi_print_msg(RTAPI_MSG_DBG, "\n" );
        /* call execv() to invoke command */
	execvp(argv[0], argv);
	/* should never get here */
//	halcmd_error("execv(%s) failed\n", argv[0] );
        snprintf(errorStr, sizeof(errorStr), "execv(%s) failed", argv[0]);
        sockWriteError(nakStr, context);
	exit(1);
    }
    /* parent process */
    /* reconnect to the HAL shmem area */
    comp_id = hal_init(comp_name);

    return pid;
}

int hal_systemv(char *const argv[], connectionRecType *context) {
    pid_t pid;
    int status;
    int retval;
    const char *nakStr = "SET LOADRT NAK";

    /* do the fork */
    pid = hal_systemv_nowait(argv, context);
    /* this is the parent process, wait for child to end */
    retval = waitpid ( pid, &status, 0 );
    retval = 0;
    if (comp_id < 0) {
	fprintf(stderr, "halcmd: hal_init() failed after systemv: %d\n", comp_id );
	exit(-1);
    }
    hal_ready(comp_id);
    /* check result of waitpid() */
    if ( retval < 0 ) {
//	halcmd_error("waitpid(%d) failed: %s\n", pid, strerror(errno) );
        snprintf(errorStr, sizeof(errorStr), "waitpid(%d) failed: %s", pid, strerror(errno));
        sockWriteError(nakStr, context);
	return -1;
    }
    if ( WIFEXITED(status) == 0 ) {
//	halcmd_error("child did not exit normally\n");
        snprintf(errorStr, sizeof(errorStr), "child did not exit normally");
        sockWriteError(nakStr, context);
	return -1;
    }
    retval = WEXITSTATUS(status);
    if ( retval != 0 ) {
//	halcmd_error("exit value: %d\n", retval );
        snprintf(errorStr, sizeof(errorStr), "exit value: %d", retval);
        sockWriteError(nakStr, context);
	return -1;
    }
    return 0;
}


/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/

/* release_HAL_mutex() unconditionally releases the hal_mutex
   very useful after a program segfaults while holding the mutex
*/
static int release_HAL_mutex(void)
{
    int comp_id, mem_id, retval;
    void *mem;
    hal_data_t *hal_data;

    /* do RTAPI init */
    comp_id = rtapi_init("hal_unlocker");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: rtapi init failed\n");
        return -EINVAL;
    }
    /* get HAL shared memory block from RTAPI */
    mem_id = rtapi_shmem_new(HAL_KEY, comp_id, HAL_SIZE);
    if (mem_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "ERROR: could not open shared memory\n");
        rtapi_exit(comp_id);
        return -EINVAL;
    }
    /* get address of shared memory area */
    retval = rtapi_shmem_getptr(mem_id, &mem);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "ERROR: could not access shared memory\n");
        rtapi_exit(comp_id);
        return -EINVAL;
    }
    /* set up internal pointers to shared mem and data structure */
    hal_data = (hal_data_t *) mem;
    /* release mutex  */
    rtapi_mutex_give(&(hal_data->mutex));
    /* release RTAPI resources */
    rtapi_shmem_delete(mem_id, comp_id);
    rtapi_exit(comp_id);
    /* done */
    return 0;

}

static int doLock(char *command, connectionRecType *context)
{
	int retval = 0;
	const char *nakStr = "SET LOCK NAK";

	/* if command is blank, want to lock everything */
	if (*command == '\0')
		retval = hal_set_lock(HAL_LOCK_ALL);
	else if (strcmp(command, "none") == 0)
		retval = hal_set_lock(HAL_LOCK_NONE);
	else if (strcmp(command, "tune") == 0)
		retval = hal_set_lock(HAL_LOCK_TUNE);
	else if (strcmp(command, "all") == 0)
		retval = hal_set_lock(HAL_LOCK_ALL);

    if (retval != 0) {
      snprintf(errorStr, sizeof(errorStr), "HAL:%d: Locking failed", linenumber);
      sockWriteError(nakStr, context);
      }
    return retval;
}

static int doUnlock(char *command, connectionRecType *context)
{
    int retval=0;
    const char *nakStr = "SET UNLOCK NAK";

	/* if command is blank, want to unlock everything */
	if (*command == '\0')
		retval = hal_set_lock(HAL_LOCK_NONE);
	else if (strcmp(command, "none") == 0)
		retval = hal_set_lock(HAL_LOCK_NONE);
	else if (strcmp(command, "tune") == 0)
		retval = hal_set_lock(hal_get_lock() & ~HAL_LOCK_TUNE);
	else if (strcmp(command, "all") == 0)
		retval = hal_set_lock(HAL_LOCK_NONE);

    if (retval != 0) {
      snprintf(errorStr, sizeof(errorStr), "HAL:%d: Unlocking failed", linenumber);
      sockWriteError(nakStr, context);
      }
    return retval;
}

static int doLinkpp(char *first_pin_name, char *second_pin_name, connectionRecType *context)
{
    int retval;
    hal_pin_t *first_pin, *second_pin;
    const char *nakStr = "SET LINKPP NAK";

    rtapi_mutex_get(&(hal_data->mutex));
    /* check if the pins are there */
    first_pin = halpr_find_pin_by_name(first_pin_name);
    second_pin = halpr_find_pin_by_name(second_pin_name);
    if (first_pin == 0) {
	/* first pin not found*/
      rtapi_mutex_give(&(hal_data->mutex));
      snprintf(errorStr, sizeof(errorStr), "HAL:%d: ERROR: pin '%s' not found\n", linenumber, first_pin_name);
      sockWriteError(nakStr, context);
      return -EINVAL; 
      } 
    else 
      if (second_pin == 0) {
	rtapi_mutex_give(&(hal_data->mutex));
        snprintf(errorStr, sizeof(errorStr), "HAL:%d: ERROR: pin '%s' not found", linenumber, second_pin_name);
        sockWriteError(nakStr, context);
	return -EINVAL; 
        }
    
    /* give the mutex, as the other functions use their own mutex */
    rtapi_mutex_give(&(hal_data->mutex));
    
    /* check that both pins have the same type, 
       don't want to create a sig, which after that won't be useful */
    if (first_pin->type != second_pin->type) {
      snprintf(errorStr, sizeof(errorStr), "HAL:%d: ERROR: pins '%s' and '%s' not of the same type", 
	  linenumber, first_pin_name, second_pin_name);
      sockWriteError(nakStr, context);
      return -EINVAL; 
    }
	
    /* now create the signal */
    retval = hal_signal_new(first_pin_name, first_pin->type);

    if (retval == 0) {
      /* if it worked, link the pins to it */
      retval = hal_link(first_pin_name, first_pin_name);

      if ( retval == 0 )
      /* if that worked, link the second pin to the new signal */
        retval = hal_link(second_pin_name, first_pin_name);
      }
    if (retval < 0) {
      snprintf(errorStr, sizeof(errorStr), "HAL:%d: linkpp failed", linenumber);
      sockWriteError(nakStr, context);
      }
    return retval;
}

int doLinkPS(char *pin, char *sig, connectionRecType *context)
{
    int retval;
    const char *nakStr = "SET NET NAK";

    retval = hal_link(pin, sig);
    if (retval == 0) {
	/* print success message */
//        halcmd_info("Pin '%s' linked to signal '%s'\n", pin, sig);
          snprintf(errorStr, sizeof(errorStr), "Pin '%s' linked to signal '%s'", pin, sig);
          sockWriteError(nakStr, context);
    } else {
//        halcmd_error("link failed\n");
          snprintf(errorStr, sizeof(errorStr), "link failed");
          sockWriteError(nakStr, context);
    }
    return retval;
}


static int preflightNet(char *signal, hal_sig_t *sig, char *pins[], connectionRecType *context) 
{
    int i, type=-1, writers=0, bidirs=0, pincnt=0;
    const char *nakStr = "SET NET NAK";

    /* if signal already exists, use its info */
    if (sig) {
	type = sig->type;
	writers = sig->writers;
	bidirs = sig->bidirs;
    }

    for(i=0; pins[i] && *pins[i]; i++) {
        hal_pin_t *pin = 0;
        pin = halpr_find_pin_by_name(pins[i]);
        if(!pin) {
//            halcmd_error("pin '%s' does not exist\n", pins[i]);
            snprintf(errorStr, sizeof(errorStr), "pin '%s' does not exist", pins[i]);
            sockWriteError(nakStr, context);
            return -ENOENT;
        }
        if(SHMPTR(pin->signal) == sig) {
	     /* Already on this signal */
	    pincnt++;
	    continue;
	} else if(pin->signal != 0) {
//            halcmd_error("pin '%s' was already linked\n", pin->name);
            snprintf(errorStr, sizeof(errorStr), "pin '%s' was already linked", pin->name);
            sockWriteError(nakStr, context);
            return -EINVAL;
	}
	if (type == -1) {
	    /* no pre-existing type, use this pin's type */
	    type = pin->type;
	}
        if(type != pin->type) {
//            halcmd_error("Type mismatch on pin '%s'\n", pin->name);
            snprintf(errorStr, sizeof(errorStr), "Type mismatch on pin '%s'", pin->name);
            sockWriteError(nakStr, context);
            return -EINVAL;
        }
        if(pin->dir == HAL_OUT) {
            if(writers || bidirs) {
//                halcmd_error("Signal '%s' can not add OUT pin '%s'\n", signal, pin->name);
            snprintf(errorStr, sizeof(errorStr), "Signal '%s' can not add OUT pin '%s'", signal, pin->name);
            sockWriteError(nakStr, context);
                return -EINVAL;
            }
            writers++;
        }
	if(pin->dir == HAL_IO) {
            if(writers) {
//                halcmd_error("Signal '%s' can not add I/O pin '%s'\n", signal, pin->name);
                snprintf(errorStr, sizeof(errorStr), "Signal '%s' can not add I/O pin '%s'", signal, pin->name);
                sockWriteError(nakStr, context);
                return -EINVAL;
            }
            bidirs++;
        }
        pincnt++;
    }
    if(pincnt)
        return 0;
//    halcmd_error("'net' requires at least one pin, none given\n");
    snprintf(errorStr, sizeof(errorStr), "'net' requires at least one pin, none given");
    sockWriteError(nakStr, context);
    return -EINVAL;
}


int doNet(char *signal, char *pins[], connectionRecType *context) 
{
    hal_sig_t *sig;
    int i, retval;
    const char *nakStr = "SET NET NAK";

    rtapi_mutex_get(&(hal_data->mutex));
    /* see if signal already exists */
    sig = halpr_find_sig_by_name(signal);

    /* verify that everything matches up (pin types, etc) */
    retval = preflightNet(signal, sig, pins, context);
    if(retval < 0) {
        rtapi_mutex_give(&(hal_data->mutex));
        return retval;
    }

    {
	hal_pin_t *pin = halpr_find_pin_by_name(signal);
	if(pin) {
//	    halcmd_error("Signal name '%s' must not be the same as a pin.\n", signal);
            snprintf(errorStr, sizeof(errorStr), "Signal name '%s' must not be the same as a pin.", signal);
            sockWriteError(nakStr, context);
	    rtapi_mutex_give(&(hal_data->mutex));
	    return -ENOENT;
	}
    }
    if(!sig) {
        /* Create the signal with the type of the first pin */
        hal_pin_t *pin = halpr_find_pin_by_name(pins[0]);
        rtapi_mutex_give(&(hal_data->mutex));
        if(!pin) {
            return -ENOENT;
        }
        retval = hal_signal_new(signal, pin->type);
    } else {
	/* signal already exists */
        rtapi_mutex_give(&(hal_data->mutex));
    }
    /* add pins to signal */
    for(i=0; retval == 0 && pins[i] && *pins[i]; i++) {
        retval = doLinkPS(pins[i], signal, context);
    }

    return retval;
}


static int doLink(char *pin, char *sig, connectionRecType *context)
{
    int retval;
    const char *nakStr = "SET LINKPS NAK";

    /* if sig is blank, want to unlink pin */
    if (*sig == '\0') {
	/* unlink the pin */
	retval = hal_unlink(pin);
    } else {
	/* make the link */
	retval = hal_link(pin, sig);
    }
    if (retval != 0) {
      snprintf(errorStr, sizeof(errorStr), "HAL:%d: link failed", linenumber);
      sockWriteError(nakStr, context);
      }
    return retval;
}

static int doNewsig(char *name, char *type, connectionRecType *context)
{
    int retval;
    const char *nakStr = "SET NEWSIG NAK";

    if (strcasecmp(type, "bit") == 0)
      retval = hal_signal_new(name, HAL_BIT);
    else 
      if (strcasecmp(type, "float") == 0) 
        retval = hal_signal_new(name, HAL_FLOAT);
      else 
        if (strcasecmp(type, "u16") == 0)
          retval = hal_signal_new(name, HAL_U32);
        else 
          if (strcasecmp(type, "s32") == 0)
            retval = hal_signal_new(name, HAL_S32);
          else {
            snprintf(errorStr, sizeof(errorStr), "HAL:%d: Unknown signal type '%s'", 
              linenumber, type);
            sockWriteError(nakStr, context);
            retval = -EINVAL;
            }
    if (retval < 0) {
      snprintf(errorStr, sizeof(errorStr), "HAL:%d: newsig failed", linenumber);
      sockWriteError(nakStr, context);
      }
    return retval;
}

static int set_common(hal_type_t type, void *d_ptr, char *value, connectionRecType *context) {
    (void)context;
    // This function assumes that the mutex is held
    int retval = 0;
    double fval;
    long lval;
    unsigned long ulval;
    char *cp = value;

    switch (type) {
    case HAL_BIT:
	if ((strcmp("1", value) == 0) || (strcasecmp("TRUE", value) == 0)) {
	    *(hal_bit_t *) (d_ptr) = 1;
	} else if ((strcmp("0", value) == 0)
	    || (strcasecmp("FALSE", value)) == 0) {
	    *(hal_bit_t *) (d_ptr) = 0;
	} else {
	    retval = -EINVAL;
	}
	break;
    case HAL_FLOAT:
	fval = strtod ( value, &cp );
	if ((*cp != '\0') && (!isspace(*cp))) {
	    /* invalid character(s) in string */
	    retval = -EINVAL;
	} else {
	    *((hal_float_t *) (d_ptr)) = fval;
	}
	break;
    case HAL_S32:
	lval = strtol(value, &cp, 0);
	if ((*cp != '\0') && (!isspace(*cp))) {
	    /* invalid chars in string */
	    retval = -EINVAL;
	} else {
	    *((hal_s32_t *) (d_ptr)) = lval;
	}
	break;
    case HAL_U32:
	ulval = strtoul(value, &cp, 0);
	if ((*cp != '\0') && (!isspace(*cp))) {
	    /* invalid chars in string */
	    retval = -EINVAL;
	} else {
	    *((hal_u32_t *) (d_ptr)) = ulval;
	}
	break;
    default:
	/* Shouldn't get here, but just in case... */
	retval = -EINVAL;
    }
    return retval;
}

static int doSetp(char *name, char *value, connectionRecType *context)
{
    const char *nakStr = "SET SETP NAK";
    int retval;
    hal_param_t *param;
    hal_pin_t *pin;
    hal_type_t type;
    void *d_ptr;

    /* get mutex before accessing shared data */
    rtapi_mutex_get(&(hal_data->mutex));
    /* search param list for name */
    param = halpr_find_param_by_name(name);
    if (param == 0) {
        pin = halpr_find_pin_by_name(name);
        if(pin == 0) {
            rtapi_mutex_give(&(hal_data->mutex));
            snprintf(errorStr, sizeof(errorStr),
                "HAL:%d: ERROR: parameter or pin '%s' not found\n", linenumber, name);
            sockWriteError(nakStr, context);
            return -EINVAL;
        } else {
            /* found it */
            type = pin->type;
            if(pin->dir == HAL_OUT) {
                rtapi_mutex_give(&(hal_data->mutex));
                snprintf(errorStr, sizeof(errorStr),
                    "HAL:%d: ERROR: pin '%s' is not writable\n", linenumber, name);
                sockWriteError(nakStr, context);
                return -EINVAL;
            }
            if(pin->signal != 0) {
                rtapi_mutex_give(&(hal_data->mutex));
                snprintf(errorStr, sizeof(errorStr),
                    "HAL:%d: ERROR: pin '%s' is connected to a signal\n", linenumber, name);
                sockWriteError(nakStr, context);
                return -EINVAL;
            }
            // d_ptr = (void*)SHMPTR(pin->dummysig);
            d_ptr = (void*)&pin->dummysig;
        }
    } else {
        /* found it */
        type = param->type;
        /* is it read only? */
        if (param->dir == HAL_RO) {
            rtapi_mutex_give(&(hal_data->mutex));
            rtapi_print_msg(RTAPI_MSG_ERR,
                "HAL:%d: ERROR: param '%s' is not writable\n", linenumber, name);
            return -EINVAL;
        }
        d_ptr = SHMPTR(param->data_ptr);
    }

    retval = set_common(type, d_ptr, value, context);

    rtapi_mutex_give(&(hal_data->mutex));
    if (retval != 0) {
	snprintf(errorStr, sizeof(errorStr), "HAL:%d: setp failed\n", linenumber);
        sockWriteError(nakStr, context);
    }
    return retval;
}

static int doSets(char *name, char *value, connectionRecType *context)
{
    const char *nakStr = "SET SETS NAK";
    int retval;
    hal_sig_t *sig;
    hal_type_t type;
    void *d_ptr;

    /* get mutex before accessing shared data */
    rtapi_mutex_get(&(hal_data->mutex));
    /* search signal list for name */
    sig = halpr_find_sig_by_name(name);
    if (sig == 0) {
	rtapi_mutex_give(&(hal_data->mutex));
	snprintf(errorStr, sizeof(errorStr),
	    "HAL:%d: ERROR: signal '%s' not found\n", linenumber, name);
        sockWriteError(nakStr, context);
	return -EINVAL;
    }
    /* found it - does it have a writer? */
    if (sig->writers > 0) {
	rtapi_mutex_give(&(hal_data->mutex));
	snprintf(errorStr, sizeof(errorStr),
	    "HAL:%d: ERROR: signal '%s' already has writer(s)\n", linenumber, name);
        sockWriteError(nakStr, context);
	return -EINVAL;
    }
    /* no writer, so we can safely set it */
    type = sig->type;
    d_ptr = SHMPTR(sig->data_ptr);
    retval = set_common(type, d_ptr, value, context);
    rtapi_mutex_give(&(hal_data->mutex));
    if (retval != 0) {
      snprintf(errorStr, sizeof(errorStr), "HAL:%d: sets failed\n", linenumber);
      sockWriteError(nakStr, context);
    }
    return retval;
}

static int doAddf(char *name, char *thread, char *parm, connectionRecType *context)
{
    int retval;
    const char *nakStr = "SET ADDF NAK";
    
    if (parm[0] == '\0')
	    /* no - add function at end of thread */
      retval = hal_add_funct_to_thread(name, thread, -1);
    else
      retval = hal_add_funct_to_thread(name, thread, atoi(parm));
    if (retval != 0) {
      /* print fail message */
      snprintf(errorStr, sizeof(errorStr), "HAL:%d: Unable to add function '%s' to thread '%s'", linenumber, name, thread);
      sockWriteError(nakStr, context);
      }
    return retval;
}

static int doDelf(char *name, char *thread, connectionRecType *context)
{
    int retval;
    const char *nakStr = "SET DELF NAK";
    
    retval = hal_del_funct_from_thread(name, thread);
    if (retval != 0) {
      /* print success message */
      snprintf(errorStr, sizeof(errorStr), "Failed to remove function '%s' from thread '%s'", name, thread);
      sockWriteError(nakStr, context);
    }
    return retval;
}

static int doStart(connectionRecType *context)
{
    int retval;
    const char *nakStr = "SET START NAK";

    retval = hal_start_threads();
    if (retval != 0) {
	    /* print success message */
      snprintf(errorStr, sizeof(errorStr), "Failed to start realtime threads");
      sockWriteError(nakStr, context);
      }
    return retval;
}

static int doStop(connectionRecType *context)
{
    int retval;
    const char *nakStr = "SET STOP NAK";

    retval = hal_stop_threads();
    if (retval != 0) {
	    /* print success message */
      snprintf(errorStr, sizeof(errorStr), "Unable to stop realtime threads");
      sockWriteError(nakStr, context);
      }
    return retval;
}

static int doLoadRt(char *mod_name, char *args[], connectionRecType *context)
{
    char arg_string[MAX_CMD_LEN+1];
    int n=0, retval=0;
    hal_comp_t *comp;
    char *cp1;
    const char *nakStr = "SET LOADRT NAK";

#if defined(RTAPI_USPACE)
#if 0
    argv[m++] = "-Wn";
    argv[m++] = mod_name;
    argv[m++] = EMC2_BIN_DIR "/rtapi_app";
    argv[m++] = "load";
    argv[m++] = mod_name;
    /* loop thru remaining arguments */
    while ( args[n] && args[n][0] != '\0' ) {
        argv[m++] = args[n++];
    }
    argv[m++] = NULL;
//    retval = do_loadusr_cmd(argv);
#endif
#else
    int m=0;
    char *argv[MAX_TOK+3];
    static char *rtmod_dir = EMC2_RTLIB_DIR;
    struct stat stat_buf;
    char mod_path[MAX_CMD_LEN+1];

    if (hal_get_lock()&HAL_LOCK_LOAD) {
      snprintf(errorStr, sizeof(errorStr),  "HAL is locked, loading of modules is not permitted");
      sockWriteError(nakStr, context);
	return -EPERM;
    }
    if ( (strlen(rtmod_dir)+strlen(mod_name)+5) > MAX_CMD_LEN ) {
      snprintf(errorStr, sizeof(errorStr), "Module path too long");
      sockWriteError(nakStr, context);
	return -1;
    }

    /* make full module name '<path>/<name>.o' */
    rtapi_strxcpy (mod_path, rtmod_dir);
    rtapi_strxcat (mod_path, "/");
    rtapi_strxcat (mod_path, mod_name);
    rtapi_strxcat (mod_path, MODULE_EXT);
    /* is there a file with that name? */
    if ( stat(mod_path, &stat_buf) != 0 ) {
        /* can't find it */
      snprintf(errorStr, sizeof(errorStr), "Can't find module '%s' in %s", mod_name, rtmod_dir);
      sockWriteError(nakStr, context);
        return -1;
    }
    
    // TODO - FIXME - remove test after 2.2.x when blocks isn't functional anymore
    if (strncmp(mod_name, "blocks", 6) == 0) {
	//using RTAPI_MSG_ERR as that is the default warning level for halcmd
        snprintf(errorStr, sizeof(errorStr), "blocks is deprecated, use the subcomponents generated by 'comp' instead");
        sockWriteError(nakStr, context);
    }

    argv[0] = EMC2_BIN_DIR "/linuxcnc_module_helper";
    argv[1] = "insert";
    argv[2] = mod_path;
    /* loop thru remaining arguments */
    n = 0;
    m = 3;
    while ( args[n] && args[n][0] != '\0' ) {
        argv[m++] = args[n++];
    }
    /* add a NULL to terminate the argv array */
    argv[m] = NULL;
/*    snprintf(context->outBuf, sizeof(context->outBuf), "Setup argv %s %s %s", argv[0], argv[1], argv[2]);
    sockWrite(context);
    return 0; */


    retval = hal_systemv(argv, context);
#endif

    if ( retval != 0 ) {
        snprintf(errorStr, sizeof(errorStr), "insmod failed, returned %d", retval);
        sockWriteError(nakStr, context);
	return -1;
    }
    /* make the args that were passed to the module into a single string */
    n = 0;
    arg_string[0] = '\0';
    while ( args[n] && args[n][0] != '\0' ) {
	strncat(arg_string, args[n++], MAX_CMD_LEN);
	strncat(arg_string, " ", MAX_CMD_LEN);
    }
    /* allocate HAL shmem for the string */
    cp1 = hal_malloc(strlen(arg_string)+1);
    if ( cp1 == NULL ) {
      snprintf(errorStr, sizeof(errorStr), "failed to allocate memory for module args");
      sockWriteError(nakStr, context);
	return -1;
    }
    /* copy string to shmem */
    strcpy (cp1, arg_string);
    /* get mutex before accessing shared data */
    rtapi_mutex_get(&(hal_data->mutex));
    /* search component list for the newly loaded component */
    comp = halpr_find_comp_by_name(mod_name);
    if (comp == 0) {
	rtapi_mutex_give(&(hal_data->mutex));
        snprintf(errorStr, sizeof(errorStr), "module '%s' not loaded", mod_name);
        sockWriteError(nakStr, context);
	return -EINVAL;
    }
    /* link args to comp struct */
    comp->insmod_args = SHMOFF(cp1);
    rtapi_mutex_give(&(hal_data->mutex));
    /* print success message */
//    halcmd_info("Realtime module '%s' loaded\n", mod_name);
    return 0;
}

static int doDelsig(char *mod_name, connectionRecType *context)
{
    int next, retval, retval1, n;
    hal_sig_t *sig;
    char sigs[MAX_EXPECTED_SIGS][HAL_NAME_LEN+1];
    const char *nakStr = "SET DELSIG NAK";

    /* check for "all" */
    if ( strcmp(mod_name, "all" ) != 0 ) {
      retval = hal_signal_delete(mod_name);
      return retval;
      }
    else {
      /* build a list of signal(s) to delete */
      n = 0;
      rtapi_mutex_get(&(hal_data->mutex));

      next = hal_data->sig_list_ptr;
      while (next != 0) {
        sig = SHMPTR(next);
        /* we want to unload this signal, remember it's name */
        if (n < ( MAX_EXPECTED_SIGS - 1)) {
          snprintf(sigs[n], sizeof(sigs[n]), "%s", sig->name);
	  n++;
	  }
        next = sig->next_ptr;
	}
      rtapi_mutex_give(&(hal_data->mutex));
      sigs[n][0] = '\0';

      if (sigs[0][0] == '\0') {
        /* desired signals not found */
        snprintf(errorStr, sizeof(errorStr), "HAL:%d: ERROR: no signals found to be deleted", linenumber);
        sockWriteError(nakStr, context);
        return -1;
        }
      /* we now have a list of components, unload them */
      n = 0;
      retval1 = 0;
      while (sigs[n][0] != '\0') {
        retval = hal_signal_delete(sigs[n]);
        /* check for fatal error */
	if (retval < -1)
          return retval;
        /* check for other error */
        if (retval != 0)
          retval1 = retval;
        n++;
	}
      }
    return retval1;
}

static int doUnload(char *mod_name, connectionRecType *context)
{
    int next, retval, retval1, n, all;
    hal_comp_t *comp;
    char comps[64][HAL_NAME_LEN+1];
    const char *nakStr = "SET UNLOAD NAK";

    /* check for "all" */
    if ( strcmp(mod_name, "all" ) == 0 ) {
	all = 1;
    } else {
	all = 0;
    }
    /* build a list of component(s) to unload */
    n = 0;
    rtapi_mutex_get(&(hal_data->mutex));
    next = hal_data->comp_list_ptr;
    while (next != 0) {
	comp = SHMPTR(next);
	if ( comp->type == COMPONENT_TYPE_REALTIME ) {
	    /* found a realtime component */
	    if ( all || ( strcmp(mod_name, comp->name) == 0 )) {
		/* we want to unload this component, remember its name */
		if ( n < 63 ) {
		    snprintf(comps[n], sizeof(comps[n]), "%s", comp->name);
		    comps[n][HAL_NAME_LEN] = '\0';
		    n++;
		}
	    }
	}
	next = comp->next_ptr;
    }
    rtapi_mutex_give(&(hal_data->mutex));
    /* mark end of list */
    comps[n][0] = '\0';
    if ( !all && ( comps[0][0] == '\0' )) {
	/* desired component not found */
//	halcmd_error("component '%s' is not loaded\n", mod_name);
        snprintf(errorStr, sizeof(errorStr), "component '%s' is not loaded", mod_name);
        sockWriteError(nakStr, context);
	return -1;
    }
    /* we now have a list of components, unload them */
    n = 0;
    retval1 = 0;
    while ( comps[n][0] != '\0' ) {
	retval = unloadrt_comp(comps[n++]);
	/* check for fatal error */
	if ( retval < -1 ) {
	    return retval;
	}
	/* check for other error */
	if ( retval != 0 ) {
	    retval1 = retval;
	}
    }
/*    if (retval1 < 0) {
        snprintf(errorStr, sizeof(errorStr), "unloadrt failed");
        sockWriteError(nakStr, context);
      } */
    return retval1;
}

static int unloadrt_comp(char *mod_name)
{
    int retval, status;
    char *argv[4];
    pid_t pid;

    /* now we need to fork, and then exec rmmod.... */
    /* disconnect from the HAL shmem area before forking */
    hal_exit(comp_id);
    comp_id = 0;
    /* now the fork() */
    pid = fork();
    if ( pid < 0 ) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL:%d: ERROR: unloadrt fork() failed\n", linenumber);
	/* reconnect to the HAL shmem area */
	comp_id = hal_init(comp_name);
	if (comp_id < 0) {
	    fprintf(stderr, "halrmt: hal_init() failed after fork: %d\n",
                    comp_id);
	    exit(-1);
	}
        hal_ready(comp_id);
	return -1;
    }
    if ( pid == 0 ) {
	/* this is the child process - prepare to exec() rmmod */
	argv[0] = EMC2_BIN_DIR "/linuxcnc_module_helper";
	argv[1] = "remove";
	argv[2] = mod_name;
	/* add a NULL to terminate the argv array */
	argv[3] = NULL;
	/* print debugging info if "very verbose" (-V) */
	rtapi_print_msg(RTAPI_MSG_DBG, "%s %s %s\n", argv[0], argv[1], argv[2] );
	/* call execv() to invoke rmmod */
	execv(argv[0], argv);
	/* should never get here */
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL:%d: ERROR: execv(%s) failed\n", linenumber, argv[0] );
	exit(1);
    }
    /* this is the parent process, wait for child to end */
    retval = waitpid ( pid, &status, 0 );
    /* reconnect to the HAL shmem area */
    comp_id = hal_init(comp_name);
    if (comp_id < 0) {
	fprintf(stderr, "halrmt: hal_init() failed after unloadrt: %d\n",
                comp_id );
	exit(-1);
    }
    hal_ready(comp_id);
    /* check result of waitpid() */
    if ( retval < 0 ) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL:%d: ERROR: waitpid(%d) failed\n", linenumber, pid);
	return -1;
    }
    if ( WIFEXITED(status) == 0 ) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL:%d: ERROR: child did not exit normally\n", linenumber);
	return -1;
    }
    retval = WEXITSTATUS(status);
    if ( retval != 0 ) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL:%d: ERROR: rmmod failed, returned %d\n", linenumber, retval);
	return -1;
    }
    /* print success message */
    rtapi_print_msg(RTAPI_MSG_INFO, "Realtime module '%s' unloaded\n",
	mod_name);
    return 0;
}

static int doLoadUsr(char *args[])
{
    int wait_flag, wait_comp_flag, name_flag, ignore_flag;
    char *prog_name, *new_comp_name;
    char prog_path[MAX_CMD_LEN+1];
    char *cp1, *cp2, *envpath;
    struct stat stat_buf;
    char *argv[MAX_TOK+1];
    int n, m, retval, status;
    pid_t pid;

    if (hal_get_lock()&HAL_LOCK_LOAD) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL:%d: ERROR: HAL is locked, loading of programs is not permitted\n", linenumber);
	return -EPERM;
    }
    /* check for options (-w, -i, and/or -r) */
    wait_flag = 0;
    wait_comp_flag = 0;
    name_flag = 0;
    ignore_flag = 0;
    prog_name = NULL;
    while ( **args == '-' ) {
	/* this argument contains option(s) */
	cp1 = *args;
	cp1++;
	while ( *cp1 != '\0' ) {
	    if ( *cp1 == 'w' ) {
		wait_flag = 1;
            } else if ( *cp1 == 'W' ) {
                wait_comp_flag = 1;
	    } else if ( *cp1 == 'i' ) {
		ignore_flag = 1;
	    } else if ( *cp1 == 'n' ) {
		name_flag = 1;
	    } else {
		rtapi_print_msg(RTAPI_MSG_ERR,
		    "HAL:%d: ERROR: unknown loadusr option '-%c'\n", linenumber, *cp1);
		return -EINVAL;
	    }
	    cp1++;
	}
	/* move to next arg */
	args++;
    }
    /* get program and component name */
    if(name_flag) {
        new_comp_name = *args++;
        prog_name = *args++;
    } else {
        new_comp_name = prog_name = *args++;
    }
    /* need to find path to a program matching "prog_name" */
    prog_path[0] = '\0';
    if ( prog_path[0] == '\0' ) {
	/* try the name by itself */
	rtapi_strlcpy(prog_path, prog_name, MAX_CMD_LEN);
	rtapi_print_msg(RTAPI_MSG_DBG, "Trying '%s'\n", prog_path);
	if ( stat(prog_path, &stat_buf) != 0 ) {
	    /* no luck, clear prog_path to indicate failure */
	    prog_path[0] = '\0';
	}
    }
    if ( prog_path[0] == '\0' ) {
	/* no luck yet, try the emc2/bin directory where
	   the halrmt executable is located */
	n = readlink("/proc/self/exe", prog_path, MAX_CMD_LEN-10);
	if ( n > 0 ) {
	    prog_path[n] = '\0';
	    /* have path to executabie, find last '/' */
	    cp2 = "";
	    cp1 = prog_path;
	    while ( *cp1 != '\0' ) {
		if ( *cp1 == '/' ) {
		    cp2 = cp1;
		}
		cp1++;
	    }
	    if ( *cp2 == '/' ) {
		/* chop "halrmt" from end of path */
		*(++cp2) = '\0';
		/* append the program name */
		strncat(prog_path, prog_name, MAX_CMD_LEN-strlen(prog_path));
		/* and try it */
		rtapi_print_msg(RTAPI_MSG_DBG, "Trying '%s'\n", prog_path);
		if ( stat(prog_path, &stat_buf) != 0 ) {
		    /* no luck, clear prog_path to indicate failure */
		    prog_path[0] = '\0';
		}
	    }
	}
    }
   if ( prog_path[0] == '\0' ) {
	/* no luck yet, try the user's PATH */
	envpath = getenv("PATH");
	if ( envpath != NULL ) {
	    while ( *envpath != '\0' ) {
		/* copy a single directory from the PATH env variable */
		n = 0;
		while ( (*envpath != ':') && (*envpath != '\0') && (n < MAX_CMD_LEN)) {
		    prog_path[n++] = *envpath++;
		}
		/* append '/' and program name */
		if ( n < MAX_CMD_LEN ) {
		    prog_path[n++] = '/';
		}
		cp1 = prog_name;
		while ((*cp1 != '\0') && ( n < MAX_CMD_LEN)) {
		    prog_path[n++] = *cp1++;
		}
		prog_path[n] = '\0';
		rtapi_print_msg(RTAPI_MSG_DBG, "Trying '%s'\n", prog_path);
		if ( stat(prog_path, &stat_buf) != 0 ) {
		    /* no luck, clear prog_path to indicate failure */
		    prog_path[0] = '\0';
		    /* and get ready to try the next directory */
		    if ( *envpath == ':' ) {
		        envpath++;
		    }
		} else {
		    /* success, break out of loop */
		    break;
		}
	    } 
	}
    }
    if ( prog_path[0] == '\0' ) {
	/* still can't find a program to run */
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL:%d: ERROR: Can't find program '%s'\n", linenumber, prog_name);
	return -1;
    }

    /* now we need to fork, and then exec the program.... */
    /* disconnect from the HAL shmem area before forking */
    hal_exit(comp_id);
    comp_id = 0;
    /* now the fork() */
    pid = fork();
    if ( pid < 0 ) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL:%d: ERROR: loadusr fork() failed\n", linenumber);
	/* reconnect to the HAL shmem area */
	comp_id = hal_init(comp_name);
	if (comp_id < 0) {
	    fprintf(stderr, "halrmt: hal_init() failed after fork: %d\n",
                    comp_id);
	    exit(-1);
	}
        hal_ready(comp_id);
	return -1;
    }
    if ( pid == 0 ) {
	/* this is the child process - prepare to exec() the program */
	argv[0] = prog_name;
	/* loop thru remaining arguments */
	n = 0;
	m = 1;
	while ( args[n][0] != '\0' ) {
	    argv[m++] = args[n++];
	}
	/* add a NULL to terminate the argv array */
	argv[m] = NULL;
	/* print debugging info if "very verbose" (-V) */
	rtapi_print_msg(RTAPI_MSG_DBG, "%s ", argv[0] );
	n = 1;
	while ( argv[n] != NULL ) {
	    rtapi_print_msg(RTAPI_MSG_DBG, "%s ", argv[n++] );
	}
	rtapi_print_msg(RTAPI_MSG_DBG, "\n" );
	/* call execv() to invoke the program */
	execv(prog_path, argv);
	/* should never get here */
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL:%d: ERROR: execv(%s) failed: %s\n", linenumber, prog_name,
            strerror(errno));
	exit(1);
    }
    /* this is the parent process, reconnect to the HAL shmem area */
    comp_id = hal_init(comp_name);
    if (comp_id < 0) {
	fprintf(stderr, "halrmt: hal_init() failed after loadusr: %d\n",
                comp_id);
	exit(-1);
    }
    hal_ready(comp_id);
    if ( wait_comp_flag ) {
        int ready = 0, count=0;
        int next;
        while(!ready) {
            struct timespec ts = {0, 10 * 1000 * 1000}; // 10ms
            nanosleep(&ts, NULL);
            retval = waitpid( pid, &status, WNOHANG );
            if(retval != 0) goto wait_common;

            rtapi_mutex_get(&(hal_data->mutex));
            next = hal_data->comp_list_ptr;
            while(next) {
                hal_comp_t *comp = SHMPTR(next);
                next = comp->next_ptr;
                if(strcmp(comp->name, new_comp_name) == 0 && comp->ready) {
                    ready = 1;
                    break;
                }
            }
            rtapi_mutex_give(&(hal_data->mutex));

            count++;
            if(count == 100) {
                fprintf(stderr, "Waiting for component '%s' to become ready.",
                        new_comp_name);
                fflush(stderr);
            } else if(count > 100 && count % 10 == 0) {
                fprintf(stderr, ".");
                fflush(stderr);
            }
        }
        if (count >= 100) {
	    fprintf(stderr, "\n");
	}
	rtapi_print_msg(RTAPI_MSG_INFO, "Component '%s' ready\n", new_comp_name);
    }
    if ( wait_flag ) {
	/* wait for child process to complete */
	retval = waitpid ( pid, &status, 0 );
	/* check result of waitpid() */
wait_common:
	if ( retval < 0 ) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL:%d: ERROR: waitpid(%d) failed\n", linenumber, pid);
	    return -1;
	}
	if ( WIFEXITED(status) == 0 ) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"HAL:%d: ERROR: program '%s' did not exit normally\n", linenumber, prog_name );
	    return -1;
	}
	if ( ignore_flag == 0 ) {
	    retval = WEXITSTATUS(status);
	    if ( retval != 0 ) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		    "HAL:%d: ERROR: program '%s' failed, returned %d\n", linenumber, prog_name, retval );
		return -1;
	    }
	}
	/* print success message */
	rtapi_print_msg(RTAPI_MSG_INFO, "Program '%s' finished\n", prog_name);
    } else {
	/* print success message */
	rtapi_print_msg(RTAPI_MSG_INFO, "Program '%s' started\n", prog_name);
    }
    return 0;
}

static void getCompInfo(char *pattern, connectionRecType *context)
{
    int next, len;
    hal_comp_t *comp;

    rtapi_mutex_get(&(hal_data->mutex));
    len = strlen(pattern);
    next = hal_data->comp_list_ptr;
    while (next != 0) {
      comp = SHMPTR(next);
      if (strncmp(pattern, comp->name, len) == 0) {
        snprintf(context->outBuf, sizeof(context->outBuf), "COMP %s %02d %s", comp->name, comp->comp_id, (comp->type != COMPONENT_TYPE_USER) ? "RT  " : "User");
	sockWrite(context);
	}
      next = comp->next_ptr;
      }
    rtapi_mutex_give(&(hal_data->mutex));
}


static void getPinInfo(char *pattern, int valuesOnly, connectionRecType *context)
{
    int next, len;
    hal_pin_t *pin;
    hal_comp_t *comp;
    hal_sig_t *sig;
    void *dptr;

    rtapi_mutex_get(&(hal_data->mutex));
    len = strlen(pattern);
    next = hal_data->pin_list_ptr;
    while (next != 0) {
      pin = SHMPTR(next);
      if (strncmp(pattern, pin->name, len) == 0) {
        comp = SHMPTR(pin->owner_ptr);
        if (pin->signal != 0) {
	  sig = SHMPTR(pin->signal);
	  dptr = SHMPTR(sig->data_ptr);
	  } 
        else {
	  sig = 0;
	  dptr = &(pin->dummysig);
	  }
	if (valuesOnly == 0)  
          snprintf(context->outBuf, sizeof(context->outBuf), "PIN %s %s %02d %s %s",
	    pin->name,
	    data_value2((int) pin->type, dptr),
	    comp->comp_id,
	    data_type((int) pin->type),
	    pin_data_dir((int) pin->dir));
	else
	  snprintf(context->outBuf, sizeof(context->outBuf), "PINVAL %s %s",
	    pin->name,
	    data_value2((int) pin->type, dptr));
	sockWrite(context);
	} 
      next = pin->next_ptr;
    }
    rtapi_mutex_give(&(hal_data->mutex));
}


static void getSigInfo(char *pattern, int valuesOnly, connectionRecType *context)
{
    int next, len;
    hal_sig_t *sig;
    void *dptr;

    rtapi_mutex_get(&(hal_data->mutex));
    len = strlen(pattern);
    next = hal_data->sig_list_ptr;
    while (next != 0) {
      sig = SHMPTR(next);
      if (strncmp(pattern, sig->name, len) == 0) {
        dptr = SHMPTR(sig->data_ptr);
        if (valuesOnly == 0)
          snprintf(context->outBuf, sizeof(context->outBuf), "SIGNAL %s  %s  %s", 
            sig->name,
	    data_value((int) sig->type, dptr),
	    data_type((int) sig->type));
	else
	  snprintf(context->outBuf, sizeof(context->outBuf), "SIGNALVAL %s %s", sig->name, data_value((int) sig->type, dptr));
	sockWrite(context);
        }
      next = sig->next_ptr;
      }
    rtapi_mutex_give(&(hal_data->mutex));
}

static void getParamInfo(char *pattern, int valuesOnly, connectionRecType *context)
{
    int next, len;
    hal_param_t *param;
    hal_comp_t *comp;

    rtapi_mutex_get(&(hal_data->mutex));
    len = strlen(pattern);
    next = hal_data->param_list_ptr;
    while (next != 0) {
      param = SHMPTR(next);
      if ( strncmp(pattern, param->name, len) == 0 ) {
        comp = SHMPTR(param->owner_ptr);
	if (valuesOnly == 0)
          snprintf(context->outBuf, sizeof(context->outBuf), "PARAM %s %s %02d %s %s",
	    param->name,
            data_value((int) param->type, SHMPTR(param->data_ptr)),
            comp->comp_id, 
	    data_type((int) param->type),
            param_data_dir((int) param->dir));
	else
	  snprintf(context->outBuf, sizeof(context->outBuf), "PARAMVAL %s %s",
	    param->name,
	    data_value((int) param->type, SHMPTR(param->data_ptr)));
	sockWrite(context);
	}
      next = param->next_ptr;
      }
    rtapi_mutex_give(&(hal_data->mutex));
}

static void getFunctInfo(char *pattern, connectionRecType *context)
{
    int next, len;
    hal_funct_t *fptr;
    hal_comp_t *comp;

    rtapi_mutex_get(&(hal_data->mutex));
    len = strlen(pattern);
    next = hal_data->funct_list_ptr;
    while (next != 0) {
      fptr = SHMPTR(next);
      if (strncmp(pattern, fptr->name, len) == 0) {
        comp = SHMPTR(fptr->owner_ptr);
	snprintf(context->outBuf, sizeof(context->outBuf), "FUNCT %s %02d %08lX %08lX %s %3d", 
	  fptr->name, 
	  comp->comp_id, 
	  (unsigned long)fptr->funct,
	  (unsigned long)fptr->arg,
	  (fptr->uses_fp ? "YES" : "NO"),
	  fptr->users);
	sockWrite(context);
	}
      next = fptr->next_ptr;
    }
    rtapi_mutex_give(&(hal_data->mutex));
}

static void getThreadInfo(char *pattern, connectionRecType *context)
{
    int next_thread, len, n;
    hal_thread_t *tptr;
    hal_list_t *list_root, *list_entry;
    hal_funct_entry_t *fentry;
    hal_funct_t *funct;

    rtapi_mutex_get(&(hal_data->mutex));
    len = strlen(pattern);
    next_thread = hal_data->thread_list_ptr;
    while (next_thread != 0) {
      tptr = SHMPTR(next_thread);
      if (strncmp(pattern, tptr->name, len) == 0) {
        char name[HAL_NAME_LEN+1];
        hal_pin_t* pin;
        hal_sig_t* sig;
        void* dptr;
        unsigned int runtime_pin_value;

        size_t ret = snprintf(name, sizeof(name), "%s.time",tptr->name);
        if (ret >= sizeof(name)) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                 "unexpected: pin name too long for %s thread",tptr->name);
            runtime_pin_value = 0;
        } else {
        pin = halpr_find_pin_by_name(name);
        if (pin) {
            if (pin->signal != 0) {
                sig = SHMPTR(pin->signal);
                dptr = SHMPTR(sig->data_ptr);
            } else {
                sig = 0;
                dptr = &(pin->dummysig);
            }
            runtime_pin_value = (int)*(int*)dptr;
        } else {
            runtime_pin_value = 0;
            rtapi_print_msg(RTAPI_MSG_ERR,
                 "unexpected: cannot find time pin for %s thread",tptr->name);
        }
	}

        snprintf(context->outBuf, sizeof(context->outBuf), "THREAD %s %11d %s %d %d",
	  tptr->name, 
	  (unsigned int)tptr->period, 
	  (tptr->uses_fp ? "YES" : "NO "),  
	  runtime_pin_value,
	  (unsigned int)tptr->maxtime);
	sockWrite(context);
        list_root = &(tptr->funct_list);
        list_entry = list_next(list_root);
        n = 1;
        while (list_entry != list_root) {
          /* print the function info */
          fentry = (hal_funct_entry_t *) list_entry;
          funct = SHMPTR(fentry->funct_ptr);
          /* scriptmode only uses one line per thread, which contains: 
             thread period, FP flag, name, then all functs separated by spaces  */
          snprintf(context->outBuf, sizeof(context->outBuf), "SUBTHREAD %s %2d", funct->name, n);
	  sockWrite(context);
          n++;
          list_entry = list_next(list_entry);
	  }
	}
      next_thread = tptr->next_ptr;
      }
    rtapi_mutex_give(&(hal_data->mutex));
}


/* Switch function for pin/sig/param type for the print_*_list functions */
static const char *data_type(int type)
{
    const char *type_str;

    switch (type) {
    case HAL_BIT:
	type_str = "bit  ";
	break;
    case HAL_FLOAT:
	type_str = "float";
	break;
    case HAL_S32:
	type_str = "s32  ";
	break;
    case HAL_U32:
	type_str = "u32  ";
	break;
    default:
	/* Shouldn't get here, but just in case... */
	type_str = "undef";
    }
    return type_str;
}

/* Switch function for pin direction for the print_*_list functions  */
static const char *pin_data_dir(int dir)
{
    const char *pin_dir;

    switch (dir) {
    case HAL_IN:
	pin_dir = "IN ";
	break;
    case HAL_OUT:
	pin_dir = "OUT";
	break;
    case HAL_IO:
	pin_dir = "I/O";
	break;
    default:
	/* Shouldn't get here, but just in case... */
	pin_dir = "???";
    }
    return pin_dir;
}

/* Switch function for param direction for the print_*_list functions  */
static const char *param_data_dir(int dir)
{
    const char *param_dir;

    switch (dir) {
    case HAL_RO:
	param_dir = "RO";
	break;
    case HAL_RW:
	param_dir = "RW";
	break;
    default:
	/* Shouldn't get here, but just in case... */
	param_dir = "??";
    }
    return param_dir;
}

/* Switch function for arrow direction for the print_*_list functions  */
static const char *data_arrow1(int dir)
{
    const char *arrow;

    switch (dir) {
    case HAL_IN:
	arrow = "<==";
	break;
    case HAL_OUT:
	arrow = "==>";
	break;
    case HAL_IO:
	arrow = "<=>";
	break;
    default:
	/* Shouldn't get here, but just in case... */
	arrow = "???";
    }
    return arrow;
}

/* Switch function for arrow direction for the print_*_list functions  */
static const char *data_arrow2(int dir)
{
    const char *arrow;

    switch (dir) {
    case HAL_IN:
	arrow = "==>";
	break;
    case HAL_OUT:
	arrow = "<==";
	break;
    case HAL_IO:
	arrow = "<=>";
	break;
    default:
	/* Shouldn't get here, but just in case... */
	arrow = "???";
    }
    return arrow;
}

/* Switch function to return var value for the print_*_list functions  */
/* the value is printed in a fixed width field */
static char *data_value(int type, void *valptr)
{
    char *value_str;
    static char buf[15];

    switch (type) {
    case HAL_BIT:
	if (*((char *) valptr) == 0)
	    value_str = "       FALSE";
	else
	    value_str = "        TRUE";
	break;
    case HAL_FLOAT:
	snprintf(buf, 14, "%12.7g", (double)*((hal_float_t *) valptr));
	value_str = buf;
	break;
    case HAL_S32:
	snprintf(buf, 14, "  %10ld", (long)*((hal_u32_t *) valptr));
	value_str = buf;
	break;
    case HAL_U32:
	snprintf(buf, 14, "    %08lX", (unsigned long)*((hal_u32_t *) valptr));
	value_str = buf;
	break;
    default:
	/* Shouldn't get here, but just in case... */
	value_str = "   undef    ";
    }
    return value_str;
}

/* Switch function to return var value in string form  */
/* the value is printed as a packed string (no whitespace */
static char *data_value2(int type, void *valptr)
{
    char *value_str;
    static char buf[15];

    switch (type) {
    case HAL_BIT:
	if (*((char *) valptr) == 0)
	    value_str = "FALSE";
	else
	    value_str = "TRUE";
	break;
    case HAL_FLOAT:
	snprintf(buf, 14, "%.7g", (double)*((hal_float_t *) valptr));
	value_str = buf;
	break;
    case HAL_S32:
	snprintf(buf, 14, "%ld", (long)*((hal_s32_t *) valptr));
	value_str = buf;
	break;
    case HAL_U32:
	snprintf(buf, 14, "%ld", (unsigned long)*((hal_u32_t *) valptr));
	value_str = buf;
	break;
    default:
	/* Shouldn't get here, but just in case... */
	value_str = "unknown_type";
    }
    return value_str;
}

static int doSave(char *type, char *filename, connectionRecType *context)
{
    FILE *dst;
    const char *nakStr = "SET SAVE NAK";

    if (rtapi_get_msg_level() == RTAPI_MSG_NONE) {
	/* must be -Q, don't print anything */
	return 0;
      }
    if (*filename == '\0' )
      dst = stdout;
    else {
      dst = fopen(filename, "w" );
      if (dst == NULL) {
	snprintf(errorStr, sizeof(errorStr), "HAL:%d: Can't open 'save' destination '%s'", linenumber, filename);
	sockWriteError(nakStr, context);
	return -1;
	}
      }
    if (*type == '\0')
      type = "all";
    if (strcmp(type, "all" ) == 0) {
	/* save everything */
	save_comps(dst);
	save_signals(dst);
	save_links(dst, 0);
	save_params(dst);
	save_threads(dst);
      } 
    else 
      if (strcmp(type, "comp") == 0)
	save_comps(dst);
      else 
        if (strcmp(type, "sig") == 0)
	  save_signals(dst);
        else 
	  if (strcmp(type, "link") == 0)
	    save_links(dst, 0);
          else 
	    if (strcmp(type, "linka") == 0)
	      save_links(dst, 1);
            else 
	      if (strcmp(type, "net") == 0)
	        save_nets(dst, 0);
              else 
	        if (strcmp(type, "neta") == 0)
	          save_nets(dst, 1);
                else 
		  if (strcmp(type, "param") == 0)
	            save_params(dst);
                  else 
		    if (strcmp(type, "thread") == 0)
	              save_threads(dst);
                    else {
	              snprintf(errorStr, sizeof(errorStr), "HAL:%d: Unknown 'save' type '%s'", linenumber, type);
		      sockWriteError(nakStr, context);
                      fclose(dst);
	              return -1;
                      }
    if (*filename != '\0' )
      fclose(dst);
    return 0;
}

static void save_comps(FILE *dst)
{
    int next;
    hal_comp_t *comp;

    fprintf(dst, "# components\n");
    rtapi_mutex_get(&(hal_data->mutex));
    next = hal_data->comp_list_ptr;
    while (next != 0) {
	comp = SHMPTR(next);
	if ( comp->type == COMPONENT_TYPE_REALTIME ) {
	    /* only print realtime components */
	    if ( comp->insmod_args == 0 ) {
		fprintf(dst, "#loadrt %s  (not loaded by loadrt, no args saved)\n", comp->name);
	    } else {
		fprintf(dst, "loadrt %s %s\n", comp->name,
		    (char *)SHMPTR(comp->insmod_args));
	    }
	}
	next = comp->next_ptr;
    }
    rtapi_mutex_give(&(hal_data->mutex));
}

static void save_signals(FILE *dst)
{
    int next;
    hal_sig_t *sig;

    fprintf(dst, "# signals\n");
    rtapi_mutex_get(&(hal_data->mutex));
    next = hal_data->sig_list_ptr;
    while (next != 0) {
	sig = SHMPTR(next);
	fprintf(dst, "newsig %s %s\n", sig->name, data_type((int) sig->type));
	next = sig->next_ptr;
    }
    rtapi_mutex_give(&(hal_data->mutex));
}

static void save_links(FILE *dst, int arrow)
{
    int next;
    hal_pin_t *pin;
    hal_sig_t *sig;
    const char *arrow_str;

    fprintf(dst, "# links\n");
    rtapi_mutex_get(&(hal_data->mutex));
    next = hal_data->pin_list_ptr;
    while (next != 0) {
	pin = SHMPTR(next);
	if (pin->signal != 0) {
	    sig = SHMPTR(pin->signal);
	    if (arrow != 0) {
		arrow_str = data_arrow1((int) pin->dir);
	    } else {
		arrow_str = "\0";
	    }
	    fprintf(dst, "linkps %s %s %s\n", pin->name, arrow_str, sig->name);
	}
	next = pin->next_ptr;
    }
    rtapi_mutex_give(&(hal_data->mutex));
}

static void save_nets(FILE *dst, int arrow)
{
    int next;
    hal_pin_t *pin;
    hal_sig_t *sig;
    const char *arrow_str;

    fprintf(dst, "# nets\n");
    rtapi_mutex_get(&(hal_data->mutex));
    next = hal_data->sig_list_ptr;
    while (next != 0) {
	sig = SHMPTR(next);
	fprintf(dst, "newsig %s %s\n", sig->name, data_type((int) sig->type));
	pin = halpr_find_pin_by_sig(sig, 0);
	while (pin != 0) {
	    if (arrow != 0) {
		arrow_str = data_arrow2((int) pin->dir);
	    } else {
		arrow_str = "\0";
	    }
	    fprintf(dst, "linksp %s %s %s\n", sig->name, arrow_str, pin->name);
	    pin = halpr_find_pin_by_sig(sig, pin);
	}
	next = sig->next_ptr;
    }
    rtapi_mutex_give(&(hal_data->mutex));
}

static void save_params(FILE *dst)
{
    int next;
    hal_param_t *param;

    fprintf(dst, "# parameter values\n");
    rtapi_mutex_get(&(hal_data->mutex));
    next = hal_data->param_list_ptr;
    while (next != 0) {
	param = SHMPTR(next);
	if (param->dir != HAL_RO) {
	    /* param is writable, save it's value */
	    fprintf(dst, "setp %s %s\n", param->name,
		data_value((int) param->type, SHMPTR(param->data_ptr)));
	}
	next = param->next_ptr;
    }
    rtapi_mutex_give(&(hal_data->mutex));
}

static void save_threads(FILE *dst)
{
    int next_thread;
    hal_thread_t *tptr;
    hal_list_t *list_root, *list_entry;
    hal_funct_entry_t *fentry;
    hal_funct_t *funct;

    fprintf(dst, "# realtime thread/function links\n");
    rtapi_mutex_get(&(hal_data->mutex));
    next_thread = hal_data->thread_list_ptr;
    while (next_thread != 0) {
	tptr = SHMPTR(next_thread);
	list_root = &(tptr->funct_list);
	list_entry = list_next(list_root);
	while (list_entry != list_root) {
	    /* print the function info */
	    fentry = (hal_funct_entry_t *) list_entry;
	    funct = SHMPTR(fentry->funct_ptr);
	    fprintf(dst, "addf %s %s\n", funct->name, tptr->name);
	    list_entry = list_next(list_entry);
	}
	next_thread = tptr->next_ptr;
    }
    rtapi_mutex_give(&(hal_data->mutex));
}

static int do_help_cmd(char *command)
{

    if (strcmp(command, "help") == 0) {
	printf("If you need help to use 'help', then I can't help you.\n");
    } else if (strcmp(command, "loadrt") == 0) {
	printf("loadrt modname [modarg(s)]\n");
	printf("  Loads realtime HAL module 'modname', passing 'modargs'\n");
	printf("  to the module.\n");
    } else if (strcmp(command, "unloadrt") == 0) {
	printf("unloadrt modname\n");
	printf("  Unloads realtime HAL module 'modname'.  If 'modname'\n");
	printf("  is 'all', unloads all realtime modules.\n");
    } else if (strcmp(command, "loadusr") == 0) {
	printf("loadusr [options] progname [progarg(s)]\n");
	printf("  Starts user space program 'progname', passing\n");
	printf("  'progargs' to it.  Options are:\n");
	printf("  -w  wait for program to finish\n");
	printf("  -i  ignore program return value (use with -w)\n");
    } else if ((strcmp(command, "linksp") == 0) || (strcmp(command,"linkps") == 0)) {
	printf("linkps pinname [arrow] signame\n");
	printf("linksp signame [arrow] pinname\n");
	printf("  Links pin 'pinname' to signal 'signame'.  Both forms do\n");
	printf("  the same thing.  Use whichever makes sense.  The optional\n");
	printf("  'arrow' can be '==>', '<==', or '<=>' and is ignored.  It\n");
	printf("  can be used in files to show the direction of data flow,\n");
	printf("  but don't use arrows on the command line.\n");
    } else if (strcmp(command, "linkpp") == 0) {
	printf("linkpp firstpin secondpin\n");
	printf("  Creates a signal with the name of the first pin,\n");	printf("  then links both pins to the signal. \n");
    }else if (strcmp(command, "unlinkp") == 0) {
	printf("unlinkp pinname\n");
	printf("  Unlinks pin 'pinname' if it is linked to any signal.\n");
    } else if (strcmp(command, "lock") == 0) {
	printf("lock [all|tune|none]\n");
	printf("  Locks HAL to some degree.\n");
	printf("  none - no locking done.\n");
	printf("  tune - some tuning is possible (setp & such).\n");
	printf("  all  - HAL completely locked.\n");
    } else if (strcmp(command, "unlock") == 0) {
	printf("unlock [all|tune]\n");
	printf("  Unlocks HAL to some degree.\n");
	printf("  tune - some tuning is possible (setp & such).\n");
	printf("  all  - HAL completely unlocked.\n");
    } else if (strcmp(command, "newsig") == 0) {
	printf("newsig signame type\n");
	printf("  Creates a new signal called 'signame'.  Type is 'bit',\n");
	printf("  'float', 'u8', 's8', 'u16', 's16', 'u32', or 's32'.\n");
    } else if (strcmp(command, "delsig") == 0) {
	printf("delsig signame\n");
	printf("  Deletes signal 'signame'.  If 'signame is 'all',\n");
	printf("  deletes all signals\n");
    } else if (strcmp(command, "setp") == 0) {
	printf("setp paramname value\n");
	printf("paramname = value\n");
	printf("  Sets parameter 'paramname' to 'value' (if writable).\n");
	printf("  'setp' and '=' work the same, don't use '=' on the\n");
	printf("  command line.  'value' may be a constant such as 1.234\n");
	printf("  or TRUE, or a reference to an environment variable,\n");
#ifdef NO_INI
	printf("  using the syntax '$name'./n");
#else
	printf("  using the syntax '$name'.  If option -i was given,\n");
	printf("  'value' may also be a reference to an INI file entry\n");
	printf("  using the syntax '[section]name'.\n");
#endif
    } else if (strcmp(command, "sets") == 0) {
	printf("sets signame value\n");
	printf("  Sets signal 'signame' to 'value' (if sig has no writers).\n");
    } else if (strcmp(command, "getp") == 0) {
	printf("getp paramname\n");
	printf("  Gets the value of parameter 'paramname'.\n");
    } else if (strcmp(command, "gets") == 0) {
	printf("gets signame\n");
	printf("  Gets the value of signal 'signame'.\n");
    } else if (strcmp(command, "addf") == 0) {
	printf("addf functname threadname [position]\n");
	printf("  Adds function 'functname' to thread 'threadname'.  If\n");
	printf("  'position' is specified, adds the function to that spot\n");
	printf("  in the thread, otherwise adds it to the end.  Negative\n");
	printf("  'position' means position with respect to the end of the\n");
	printf("  thread.  For example '1' is start of thread, '-1' is the\n");
	printf("  end of the thread, '-3' is third from the end.\n");
    } else if (strcmp(command, "delf") == 0) {
	printf("delf functname threadname\n");
	printf("  Removes function 'functname' from thread 'threadname'.\n");
    } else if (strcmp(command, "show") == 0) {
	printf("show [type] [pattern]\n");
	printf("  Prints info about HAL items of the specified type.\n");
	printf("  'type' is 'comp', 'pin', 'sig', 'param', 'funct',\n");
	printf("  'thread', or 'all'.  If 'type' is omitted, it assumes\n");
	printf("  'all' with no pattern.  If 'pattern' is specified\n");
	printf("  it prints only those items whose names match the\n");
	printf("  pattern (no fancy regular expressions, just a simple\n");
	printf("  match: 'foo' matches 'foo', 'foobar' and 'foot' but\n");
	printf("  not 'fo' or 'frobz' or 'ffoo').\n");
    } else if (strcmp(command, "list") == 0) {
	printf("list type [pattern]\n");
	printf("  Prints the names of HAL items of the specified type.\n");
	printf("  'type' is 'comp', 'pin', 'sig', 'param', 'funct', or\n");
	printf("  'thread'.  If 'pattern' is specified it prints only\n");
	printf("  those names that match the pattern (no fancy regular\n");
	printf("  expressions, just a simple match: 'foo' matches 'foo',\n");
	printf("  'foobar' and 'foot' but not 'fo' or 'frobz' or 'ffoo').\n");
	printf("  Names are printed on a single line, space separated.\n");
    } else if (strcmp(command, "status") == 0) {
	printf("status [type]\n");
	printf("  Prints status info about HAL.\n");
	printf("  'type' is 'lock', 'mem', or 'all'. \n");
	printf("  If 'type' is omitted, it assumes\n");
	printf("  'all'.\n");
    } else if (strcmp(command, "save") == 0) {
	printf("save [type] [filename]\n");
	printf("  Prints HAL state to 'filename' (or stdout), as a series\n");
	printf("  of HAL commands.  State can later be restored by using\n");
	printf("  \"halrmt -f filename\".\n");
	printf("  Type can be 'comp', 'sig', 'link[a]', 'net[a]', 'param',\n");
	printf("  or 'thread'.  ('linka' and 'neta' show arrows for pin\n");
	printf("  direction.)  If 'type' is omitted or 'all', does the\n");
	printf("  equivalent of 'comp', 'sig', 'link', 'param', and 'thread'.\n");
    } else if (strcmp(command, "start") == 0) {
	printf("start\n");
	printf("  Starts all realtime threads.\n");
    } else if (strcmp(command, "stop") == 0) {
	printf("stop\n");
	printf("  Stops all realtime threads.\n");
    } else if (strcmp(command, "quit") == 0) {
	printf("quit\n");
	printf("  Stop processing input and terminate halrmt (when\n");
	printf("  reading from a file or stdin).\n");
    } else if (strcmp(command, "exit") == 0) {
	printf("exit\n");
	printf("  Stop processing input and terminate halrmt (when\n");
	printf("  reading from a file or stdin).\n");
    } else {
	printf("No help for unknown command '%s'\n", command);
    }
    return 0;
}

static void print_help_general(int showR)
{
    printf("\nUsage:   halrmt [options] [cmd [args]]\n\n");
    printf("options:\n\n");
    printf("  -f [filename]  Read commands from 'filename', not command\n");
    printf("                 line.  If no filename, read from stdin.\n");
#ifndef NO_INI
    printf("  -i filename    Open INI file 'filename', allow commands\n");
    printf("                 to get their values from INI file.\n");
#endif
    printf("  -q             Quiet - print errors only (default).\n");
    printf("  -Q             Very quiet - print nothing.\n");
    if (showR != 0) {
    printf("  -R             Release mutex (for crash recovery only).\n");
    }
    printf("  -s             Script friendly - don't print headers on output.\n");
    printf("  -v             Verbose - print result of every command.\n");
    printf("  -V             Very verbose - print lots of junk.\n");
    printf("  -h             Help - print this help screen and exit.\n\n");
    printf("commands:\n\n");
    printf("  loadrt, unloadrt, loadusr, lock, unlock, linkps, linksp, linkpp,\n");
    printf("  unlinkp, newsig, delsig, setp, getp, sets, gets, addf, delf, show,\n");
    printf("  list, save, status, start, stop, quit, exit\n");
    printf("  help           Lists all commands with short descriptions\n");
    printf("  help command   Prints detailed help for 'command'\n\n");
}

static halCommandType lookupHalCommand(char *s)
{
  halCommandType i = hcEcho;
  int temp;
  
  while (i < hcUnknown) {
    if (strcmp(halCommands[i], s) == 0) return i;
//    (int)i += 1;
      temp = i;
      temp++;
      i = (halCommandType) temp;
    }
  return i;
}
 
 static int commandHello(connectionRecType *context)
{
  char *pch;
//  const char *password = "EMC";
  
  pch = strtok(NULL, delims);
  if (pch == NULL) return -1;
  if (strcmp(pch, pwd) != 0) return -1;
  pch = strtok(NULL, delims);
  if (pch == NULL) return -1;
  rtapi_strxcpy(context->hostName, pch);  
  pch = strtok(NULL, delims);
  if (pch == NULL) return -1;
  context->linked = 1;    
  rtapi_strxcpy(context->version, pch);
  printf("Connected to %s\n\r", context->hostName);
  return 0;
}

static cmdResponseType getEcho(char *s, connectionRecType *context)
{
  (void)s;
  const char *pEchoStr = "ECHO %s";
  
  if (context->echo == 1) snprintf(context->outBuf, sizeof(context->outBuf), pEchoStr, "ON");
  else snprintf(context->outBuf, sizeof(context->outBuf), pEchoStr, "OFF");
  return rtNoError;
}

static cmdResponseType getVerbose(char *s, connectionRecType *context)
{
  (void)s;
  const char *pVerboseStr = "VERBOSE %s";
  
  if (context->verbose == 1) snprintf(context->outBuf, sizeof(context->outBuf), pVerboseStr, "ON");
  else snprintf(context->outBuf, sizeof(context->outBuf), pVerboseStr, "OFF");
  return rtNoError;
}

static cmdResponseType getEnable(char *s, connectionRecType *context)
{
  (void)s;
  const char *pEnableStr = "ENABLE %s";
  
  if (context->cliSock == enabledConn) snprintf(context->outBuf, sizeof(context->outBuf), pEnableStr, "ON");
  else snprintf(context->outBuf, sizeof(context->outBuf), pEnableStr, "OFF");
  return rtNoError;
}

static cmdResponseType getConfig(char *s, connectionRecType *context)
{
  (void)s;
  const char *pConfigStr = "CONFIG";

  rtapi_strxcpy(context->outBuf, pConfigStr);
  return rtNoError;
}

static cmdResponseType getCommMode(char *s, connectionRecType *context)
{
  (void)s;
  const char *pCommModeStr = "COMM_MODE %s";
  
  switch (context->commMode) {
    case 0: snprintf(context->outBuf, sizeof(context->outBuf), pCommModeStr, "ASCII"); break;
    case 1: snprintf(context->outBuf, sizeof(context->outBuf), pCommModeStr, "BINARY"); break;
    }
  return rtNoError;
}

static cmdResponseType getCommProt(char *s, connectionRecType *context)
{
  (void)s;
  const char *pCommProtStr = "COMM_PROT %s";
  
  snprintf(context->outBuf, sizeof(context->outBuf), pCommProtStr, context->version);
  return rtNoError;
}

static cmdResponseType getComps(char *s, connectionRecType *context)
{
  if (s == NULL) 
    getCompInfo("", context);
  else
    getCompInfo(s, context);
  return rtHandledNoError;
}

static cmdResponseType getPins(char *s, connectionRecType *context)
{
  if (s == NULL) 
   getPinInfo("", 0, context);
  else 
    getPinInfo(s, 0, context);
  return rtHandledNoError;
}

static cmdResponseType getPinVals(char *s, connectionRecType *context)

{
  if (s == NULL)
    getPinInfo("", 1, context);
  else
    getPinInfo(s, 1, context);
  return rtHandledNoError;
}

static cmdResponseType getSignals(char *s, connectionRecType *context)
{
  if (s == NULL)
    getSigInfo("", 0, context);
  else
    getSigInfo(s, 0, context);
  return rtHandledNoError;
}

static cmdResponseType getSignalVals(char *s, connectionRecType *context)
{
  if (s == NULL)
    getSigInfo("", 1, context);
  else
    getSigInfo(s, 1, context);
  return rtHandledNoError;
}

static cmdResponseType getParams(char *s, connectionRecType *context)
{
  if (s == NULL)
    getParamInfo("", 0, context);
  else
    getParamInfo(s, 0, context);
  return rtHandledNoError;
}

static cmdResponseType getParamVals(char *s, connectionRecType *context)
{
  if (s == NULL) 
    getParamInfo("", 1, context);
  else
    getParamInfo(s, 1, context);
  return rtHandledNoError;
}

static cmdResponseType getFuncts(char *s, connectionRecType *context)
{
  if (s == NULL)
    getFunctInfo("", context);
  else
    getFunctInfo(s, context);
  return rtHandledNoError;
}

static cmdResponseType getThreads(char *s, connectionRecType *context)
{
  if (s == NULL)
    getThreadInfo("", context);
  else
    getThreadInfo(s, context);
  return rtHandledNoError;
}

static cmdResponseType getComp(char *s, connectionRecType *context)
{
  if (s == NULL) return rtStandardError;
  getCompInfo(s, context);
  return rtHandledNoError;
}

static cmdResponseType getPin(char *s, connectionRecType *context)
{
  if (s == NULL) return rtStandardError;
  getPinInfo(s, 0, context);
  return rtHandledNoError;
}

static cmdResponseType getPinVal(char *s, connectionRecType *context)
{
  if (s == NULL) return rtStandardError;
  getPinInfo(s, 1, context);
  return rtHandledNoError;
}

static cmdResponseType getSignal(char *s, connectionRecType *context)
{
  if (s == NULL) return rtStandardError;
  getSigInfo(s, 0, context);
  return rtHandledNoError;
}

static cmdResponseType getSignalVal(char *s, connectionRecType *context)
{
  if (s == NULL) return rtStandardError;
  getSigInfo(s, 1, context);
  return rtHandledNoError;
}

static cmdResponseType getParam(char *s, connectionRecType *context)
{
  if (s == NULL) return rtStandardError;
  getParamInfo(s, 0, context);
  return rtHandledNoError;
}

static cmdResponseType getParamVal(char *s, connectionRecType *context)
{
  if (s == NULL) return rtStandardError;
  getParamInfo(s, 1, context);
  return rtHandledNoError;
}

static cmdResponseType getFunct(char *s, connectionRecType *context)
{
  if (s == NULL) return rtStandardError;
  getFunctInfo(s, context);
  return rtHandledNoError;
}

static cmdResponseType getThread(char *s, connectionRecType *context)
{
  if (s == NULL) return rtStandardError;
  getThreadInfo(s, context);
  return rtHandledNoError;
}

int commandGet(connectionRecType *context)
{
  static char *setNakStr = "GET NAK\r\n";
  static char *setCmdNakStr = "GET %s NAK\r\n";
  halCommandType cmd;
  char *pch;
  cmdResponseType ret = rtNoError;
  
  pch = strtok(NULL, delims);
  if (pch == NULL) {
    return write(context->cliSock, setNakStr, strlen(setNakStr));
    }
  strupr(pch);
  cmd = lookupHalCommand(pch);
  switch (cmd) {
    case hcEcho: ret = getEcho(pch, context); break;
    case hcVerbose: ret = getVerbose(pch, context); break;
    case hcEnable: ret = getEnable(pch, context); break;
    case hcConfig: ret = getConfig(pch, context); break;
    case hcCommMode: ret = getCommMode(pch, context); break;
    case hcCommProt: ret = getCommProt(pch, context); break;
    case hcComps: ret = getComps(strtok(NULL, delims), context); break;
    case hcPins: ret = getPins(strtok(NULL, delims), context); break;
    case hcPinVals: ret = getPinVals(pch, context); break;
    case hcSigs: ret = getSignals(strtok(NULL, delims), context); break;
    case hcSigVals: ret = getSignalVals(strtok(NULL, delims), context); break;
    case hcParams: ret = getParams(strtok(NULL, delims), context); break;
    case hcParamVals: ret = getParamVals(strtok(NULL, delims), context); break;
    case hcFuncts: ret = getFuncts(strtok(NULL, delims), context); break;
    case hcThreads: ret = getThreads(strtok(NULL, delims), context); break;
    case hcComp: ret = getComp(strtok(NULL, delims), context); break;
    case hcPin: ret = getPin(strtok(NULL, delims), context); break;
    case hcPinVal: ret = getPinVal(strtok(NULL, delims), context); break;
    case hcSig: ret = getSignal(strtok(NULL, delims), context); break;
    case hcSigVal: ret = getSignalVal(strtok(NULL, delims), context); break;
    case hcParam: ret = getParam(strtok(NULL, delims), context); break;
    case hcParamVal: ret = getParamVal(strtok(NULL, delims), context); break;
    case hcFunct: ret = getFunct(strtok(NULL, delims), context); break;
    case hcThread: ret = getThread(strtok(NULL, delims), context); break;
    case hcLoadRt: ;
    case hcUnload: ; 
    case hcLoadUsr: ; 
    case hcLinkps: ;
    case hcLinksp: ; 
    case hcLinkpp: ;
    case hcNet: ;
    case hcUnlinkp: ;
    case hcLock: ;
    case hcUnlock: ;
    case hcNewSig: ;
    case hcDelSig: ;
    case hcSetP: ;
    case hcSetS: ;
    case hcAddF: ;
    case hcDelF: ;
    case hcSave: ;
    case hcStart: ;
    case hcStop: ;
    case hcUnknown: ret = rtStandardError;
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

static int checkOnOff(char *s)
{
  static char *onStr = "ON";
  static char *offStr = "OFF";
  
  if (s == NULL) return -1;
  strupr(s);
  if (strcmp(s, onStr) == 0) return 0;
  if (strcmp(s, offStr) == 0) return 1;
  return -1;
}

static int checkBinaryASCII(char *s)
{
  static char *binaryStr = "BINARY";
  static char *ASCIIStr = "ASCII";
  
  if (s == NULL) return -1;
  strupr(s);
  if (strcmp(s, ASCIIStr) == 0) return 0;
  if (strcmp(s, binaryStr) == 0) return 1;
  return -1;
}

static cmdResponseType setEcho(char *s, connectionRecType *context)
{
   
   switch (checkOnOff(s)) {
     case -1: return rtStandardError;
     case 0: context->echo = 1; break;
     case 1: context->echo = 0;
     }
   return rtNoError;
}

static cmdResponseType setVerbose(char *s, connectionRecType *context)
{
   
   switch (checkOnOff(s)) {
     case -1: return rtStandardError;
     case 0: context->verbose = 1; break;
     case 1: context->verbose = 0;
     }
   return rtNoError;
}

static cmdResponseType setEnable(char *s, connectionRecType *context)
{
   if (strcmp(s, enablePWD) == 0) {
     enabledConn = context->cliSock;
     context->enabled = 1;
     return rtNoError;
     }
   else 
     if (checkOnOff(s) == 1) {
       context->enabled = 0;
       enabledConn = -1;
       return rtNoError;
       }
     else return rtStandardError;
}

static cmdResponseType setConfig(char *s, connectionRecType *context)
{
  (void)s;
  (void)context;
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

static cmdResponseType setCommProt(char *s, connectionRecType *context)
{
  (void)s;
  char *pVersion;
  
  pVersion = strtok(NULL, delims);
  if (pVersion == NULL) return rtStandardError;
  rtapi_strxcpy(context->version, pVersion);
  return rtNoError;
}

static cmdResponseType setLoadRt(char *s, connectionRecType *context)
{
  char *pch;
  char *args[MAX_TOK+3];
  int i;
  
  if (*s == '\0') return rtCustomHandledError;
  i = 0;
  pch = strtok(NULL, delims);
  while (pch != NULL) {
    args[i] = pch;
    pch = strtok(NULL, delims);
    i++;
    } 
  args[i] = NULL;
  if (doLoadRt(s, args, context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setUnload(char *s, connectionRecType *context)
{
  if (doUnload(s, context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setLoadUsr(char *s, connectionRecType *context)
{
  (void)context;
  char *argv[MAX_TOK+1] = {0};

  argv[0] = s;
  argv[1] = "\0";
  if (doLoadUsr(argv) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setLinkps(char *p, char *s, connectionRecType *context)
{
  if ((p == NULL) || (s == NULL)) return rtStandardError;
  if (doLink(p, s, context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setLinksp(char *p, char *s, connectionRecType *context)
{
  if ((p == NULL) || (s == NULL)) return rtStandardError;
  if (doLink(s, p, context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setLinkpp(char *p1, char *p2, connectionRecType *context)
{
  if ((p1 == NULL) || (p2 == NULL)) return rtStandardError;
  if (doLinkpp(p1, p2, context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setNet(char *p1, connectionRecType *context)
{
  char *argv[MAX_TOK];
  int i;
  char *pch;

  if (p1 == NULL) return rtStandardError;
  i = 0;

  do {
    pch = strtok(NULL, delims);
    if (pch == NULL) break;
    argv[i] = pch;
    i++;
    if (i >= MAX_TOK) return rtStandardError;  // too many pins to link to the net
  } while(1);

  if (i == 0) return rtStandardError;  // no pins to link to the net
  else 
    if (doNet(p1, argv, context) == 0)
      return rtNoError;
    else return rtCustomHandledError;
}

static cmdResponseType setUnlink(char *s, connectionRecType *context)
{
  if (s == NULL) return rtStandardError;
  if (doLink(s, "\0", context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setLock(char *s, connectionRecType *context)
{
  if (s == NULL) return rtStandardError;
  if (doLock(s, context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setUnlock(char *s, connectionRecType *context)
{
  if (s == NULL) return rtStandardError;
  if (doUnlock(s, context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setNewsig(char *s, char *t, connectionRecType *context)
{
  if ((s == NULL) || (t == NULL)) return rtStandardError;
  if (doNewsig(s, t, context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setDelsig(char *s, connectionRecType *context)
{
  if (s == NULL) return rtStandardError;
  if (doDelsig(s, context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setSetp(char *s, char *p, connectionRecType *context)
{
  if ((s == NULL) || (p == NULL)) return rtStandardError;
  if (doSetp(s, p, context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setSets(char *s, char *p, connectionRecType *context)
{
  if ((s == NULL) || (p == NULL)) return rtStandardError;
  if (doSets(s, p, context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setAddf(char *s, char *t, char *p, connectionRecType *context)
{
  if ((s == NULL) || (t == NULL)) return rtStandardError;
  if (doAddf(s, t, p, context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setDelf(char *s, char *t, connectionRecType *context)
{
  if ((s == NULL) || (t == NULL)) return rtStandardError;
  if (doDelf(s, t, context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setSave(char *type, char *fileName, connectionRecType *context)
{
  if (doSave(type, fileName, context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setStart(connectionRecType *context)
{
  if (doStart(context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

static cmdResponseType setStop(connectionRecType *context)
{
  if (doStop(context) == 0)
    return rtNoError;
  else
    return rtCustomHandledError;
}

#define MAX_TOKENS 5

int commandSet(connectionRecType *context)
{
  static char *setNakStr = "SET NAK\n\r";
  static char *setCmdNakStr = "SET %s NAK\n\r";
  static char *ackStr = "SET %s ACK\n\r";
  halCommandType cmd;
  char *tokens[MAX_TOKENS];
  int i;
  char *pch;
  char *pcmd;
  int retval = 0;
  cmdResponseType ret = rtNoError;
  
  pcmd = strtok(NULL, delims);
  if (pcmd == NULL) {
    return write(context->cliSock, setNakStr, strlen(setNakStr));
    }
  strupr(pcmd);
  cmd = lookupHalCommand(pcmd);
  if ((cmd >= hcCommProt) && (context->cliSock != enabledConn)) {
    snprintf(context->outBuf, sizeof(context->outBuf), setCmdNakStr, pcmd);
    return write(context->cliSock, context->outBuf, strlen(context->outBuf));
    }
  pch = strtok(NULL, delims);
  i = 0;
  while (pch != NULL) {
    tokens[i] = pch;
    i++;
    pch = strtok(NULL, delims);
    }
  switch (cmd) {
    case hcEcho: ret = setEcho(tokens[0], context); break;
    case hcVerbose: ret = setVerbose(tokens[0], context); break;
    case hcEnable: ret = setEnable(tokens[0], context); break;
    case hcConfig: ret = setConfig(tokens[0], context); break;
    case hcCommMode: ret = setCommMode(tokens[0], context); break;
    case hcCommProt: ret = setCommProt(tokens[0], context); break;
    case hcComps: break;
    case hcPins: break;
    case hcPinVals: break;
    case hcSigs: break;
    case hcSigVals: break;
    case hcParams: break;
    case hcParamVals: break;
    case hcFuncts: break;
    case hcThreads: break;
    case hcComp: break;
    case hcPin: break;
    case hcPinVal: break;
    case hcSig: break;
    case hcSigVal: break;
    case hcParam: break;
    case hcParamVal: break;
    case hcFunct: break;
    case hcThread: break;
    case hcLoadRt: ret = setLoadRt(tokens[0], context); break;
    case hcUnload: ret = setUnload(tokens[0], context); break;
    case hcLoadUsr: ret = setLoadUsr(tokens[0], context); break;
    case hcLinkps: setLinkps(tokens[0], tokens[1], context); break;
    case hcLinksp: setLinksp(tokens[0], tokens[1], context); break; 
    case hcLinkpp: setLinkpp(tokens[0], tokens[1], context); break;
    case hcNet:    setNet(tokens[0], context); break;
    case hcUnlinkp: setUnlink(tokens[0], context); break;
    case hcLock: setLock(tokens[0], context); break;
    case hcUnlock: setUnlock(tokens[0], context); break;;
    case hcNewSig: setNewsig(tokens[0], tokens[1], context); break;
    case hcDelSig: setDelsig(tokens[0], context); break;
    case hcSetP: setSetp(tokens[0], tokens[1], context); break;
    case hcSetS: setSets(tokens[0], tokens[1], context); break;
    case hcAddF: setAddf(tokens[0], tokens[1], tokens[2], context); break;
    case hcDelF: setDelf(tokens[0], tokens[1], context); break;
    case hcSave: setSave(tokens[0], tokens[1], context); break;
    case hcStart: setStart(context); break;
    case hcStop: setStop(context); break;
    case hcUnknown: ret = rtStandardError;
    }
  switch (ret) {
    case rtNoError:  
      if (context->verbose) {
        snprintf(context->outBuf, sizeof(context->outBuf), ackStr, pcmd);
        retval = write(context->cliSock, context->outBuf, strlen(context->outBuf));
        }
      break;
    case rtHandledNoError: // Custom ok response already handled, take no action
      break; 
    case rtStandardError:
      snprintf(context->outBuf, sizeof(context->outBuf), setCmdNakStr, pcmd);
      retval = write(context->cliSock, context->outBuf, strlen(context->outBuf));
      break;
    case rtCustomError: // Custom error response entered in buffer
      retval = write(context->cliSock, context->outBuf, strlen(context->outBuf));
      break;
    case rtCustomHandledError: ;// Custom error response handled, take no action
    }
  return retval;
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
    hal_flag = 1;
    hal_exit(comp_id);
    exit(0);
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
  rtapi_strxcat(context->outBuf, "  Quit\n\r");
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
  rtapi_strxcat(context->outBuf, "    Comm_mode\n\r");
  rtapi_strxcat(context->outBuf, "    Comm_prot\n\r");
  rtapi_strxcat(context->outBuf, "    Comp <comp name\n\r");
  rtapi_strxcat(context->outBuf, "    Comps\n\r");
  rtapi_strxcat(context->outBuf, "    Echo\n\r");
  rtapi_strxcat(context->outBuf, "    Enable\n\r");
  rtapi_strxcat(context->outBuf, "    Funct <funct name>\n\r");
  rtapi_strxcat(context->outBuf, "    Functs\n\r");
  rtapi_strxcat(context->outBuf, "    Param <param name>\n\r");
  rtapi_strxcat(context->outBuf, "    Params\n\r");
  rtapi_strxcat(context->outBuf, "    ParamVal <param name>\n\r");
  rtapi_strxcat(context->outBuf, "    ParamVals\n\r");
  rtapi_strxcat(context->outBuf, "    Pin <pin name>\n\r");
  rtapi_strxcat(context->outBuf, "    PinVal <pin name>\n\r");
  rtapi_strxcat(context->outBuf, "    Pins\n\r");
  rtapi_strxcat(context->outBuf, "    PinVals\n\r");
  rtapi_strxcat(context->outBuf, "    Signal <signal name>\n\r");
  rtapi_strxcat(context->outBuf, "    Signals\n\r");
  rtapi_strxcat(context->outBuf, "    SigVal <signal name>\n\r");
  rtapi_strxcat(context->outBuf, "    SigVals\n\r");
  rtapi_strxcat(context->outBuf, "    Thread <thread name>\n\r");
  rtapi_strxcat(context->outBuf, "    Threads\n\r");
  rtapi_strxcat(context->outBuf, "    Verbose\n\r");
//  rtapi_strxcat(outBuf, "CONFIG\n\r");
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
  rtapi_strxcat(context->outBuf, "    Addf <function name> <threadname> [<parameters>]\n\r");
  rtapi_strxcat(context->outBuf, "    Delf <function name>\n\r");
  rtapi_strxcat(context->outBuf, "    DelSig <signal name>\n\r");
  rtapi_strxcat(context->outBuf, "    Linkpp <pin name> <pin name>\n\r");
  rtapi_strxcat(context->outBuf, "    Linkps <pin name> <signal name>\n\r");
  rtapi_strxcat(context->outBuf, "    Linksp <signal name> <pin name>\n\r");
  rtapi_strxcat(context->outBuf, "    Loadrt <name>\n\r");
  rtapi_strxcat(context->outBuf, "    Loadusr <name> [<param 1> .. <param n>]\n\r");
  rtapi_strxcat(context->outBuf, "    Lock <command>\n\r");
  rtapi_strxcat(context->outBuf, "    Net <signal name> [<pin 1 name> .. <pin n name>\n\r");
  rtapi_strxcat(context->outBuf, "    NewSig <signal name> <signal type>\n\r");
  rtapi_strxcat(context->outBuf, "    Save [<hal type> [<file name>]]\n\r");
  rtapi_strxcat(context->outBuf, "    Setp <pin name> <value>\n\r");
  rtapi_strxcat(context->outBuf, "    Sets <signal name> <value>\n\r");
  rtapi_strxcat(context->outBuf, "    Start\n\r");
  rtapi_strxcat(context->outBuf, "    Stop\n\r");
  rtapi_strxcat(context->outBuf, "    Unlink <pin name>\n\r");
  rtapi_strxcat(context->outBuf, "    Unload <name>\n\r");
  rtapi_strxcat(context->outBuf, "    Unlock <command>\n\r");
  
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
  printf("Command = %s", pch);
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
  static char *helloNakStr = "HELLO NAK\r\n";
  static char *helloAckStr = "HELLO ACK %s 1.1\r\n";
  static char *setNakStr = "SET NAK\r\n";

  pch = strtok(context->inBuf, delims);
  snprintf(s, sizeof(s), helloAckStr, serverName);
  if (pch != NULL) {
    strupr(pch);
    switch (lookupToken(pch)) {
      case cmdHello: 
        if (commandHello(context) == -1)
          ret = write(context->cliSock, helloNakStr, strlen(helloNakStr));
        else 
          ret = write(context->cliSock, s, strlen(s));
        break;
      case cmdGet: 
        ret = commandGet(context);
        break;
      case cmdSet:
        if (context->linked == 0)
	  ret = write(context->cliSock, setNakStr, strlen(setNakStr));
        else ret = commandSet(context);
        break;
      case cmdQuit: 
        ret = commandQuit(context);
        break;
      case cmdShutdown:
        ret = commandShutdown(context);
	break;
      case cmdHelp:
        ret = commandHelp(context);
	break;
      case cmdUnknown: ret = -2;
      }
    }
  return ret;
}  

void *readClient(void *arg)
{
  (void)arg;
  char str[1600];
  char buf[1600];
  unsigned int i, j;
  int len;
  int ret;
  connectionRecType *context;
  
  
//  res = 1;
  context = (connectionRecType *) malloc(sizeof(connectionRecType));
  context->cliSock = client_sockfd;
  context->linked = 0;
  context->echo = 1;
  context->verbose = 0;
  rtapi_strxcpy(context->version, "1.0");
  rtapi_strxcpy(context->hostName, "Default");
  connCount++;
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
    if ((context->echo == 1) && (context->linked == 1)) {
      ret = write(context->cliSock, &buf, strlen(buf));
      if (ret < 0) {
        goto finished;
      }
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
}
  
/***********************************************************************
*                            MAIN PROGRAM                              *
************************************************************************/ 
 
 /* main() is responsible for parsing command line options, and then
   parsing either a single command from the command line or a series
   of commands from a file or standard input.  It breaks the command[s]
   into tokens, and passes them to parse_cmd() which does the actual
   work for each command.
*/

int sockMain()
{
    pthread_t thrd;
    int res;
    
    while (1) {
      
      client_len = sizeof(client_address);
      client_sockfd = accept(server_sockfd,
        (struct sockaddr *)&client_address, &client_len);
      if (client_sockfd < 0) exit(0);
      res = pthread_create(&thrd, NULL, readClient, (void *)NULL);
      if (res != 0)
        close(client_sockfd);
     }
    return 0;
}

int main(int argc, char **argv)
{
    int n, fd;
    int errorcount;
    int opt;
    char *cp1, *filename = NULL;
    FILE *srcfile = NULL;

    /* set default level of output - 'quiet' */
    rtapi_set_msg_level(RTAPI_MSG_ERR);
    /* set default for other options */
    // process halrmt command line args
    while((opt = getopt_long(argc, argv, "e:n:p:s:w:", longopts, NULL)) != -1) {
      switch(opt) {
        case 'e': snprintf(enablePWD, sizeof(enablePWD), "%s", optarg); break;
        case 'n': snprintf(serverName, sizeof(serverName), "%s", optarg); break;
        case 'p': sscanf(optarg, "%d", &port); break;
        case 's': sscanf(optarg, "%d", &maxSessions); break;
        case 'w': snprintf(pwd, sizeof(pwd), "%s", optarg); break;
        }
      }

    /* start parsing halcmd options */
    n = optind;
    while ((n < argc) && (argv[n][0] == '-')) {
	cp1 = argv[n++];
	/* loop to parse grouped options */
	while (*(++cp1) != '\0') {
	    switch (*cp1) {
	    case 'R':
		/* force an unlock of the HAL mutex - to be used after a segfault in a hal program */
		if (release_HAL_mutex() < 0) {
			printf("halrmt: Release Mutex failed!\n");
			return 1;
		}
		return 0;
		break;
	    case 'h':
		/* -h = help */
                if (argc > n) {       /* there are more arguments, n has been incremented already */
                    do_help_cmd(argv[n]);
                } else
		    print_help_general(1);
		return 0;
		break;
	    case 'q':
		/* -q = quiet (default) */
		rtapi_set_msg_level(RTAPI_MSG_ERR);
		break;
	    case 'Q':
		/* -Q = very quiet */
		rtapi_set_msg_level(RTAPI_MSG_NONE);
		break;
	    case 's':
		/* script friendly mode */
		scriptmode = 1;
		break;
	    case 'v':
		/* -v = verbose */
		rtapi_set_msg_level(RTAPI_MSG_INFO);
		break;
	    case 'V':
		/* -V = very verbose */
		rtapi_set_msg_level(RTAPI_MSG_ALL);
		break;
	    case 'f':
		/* -f = read from file (or stdin) */
		if (srcfile == NULL) {
		    /* it's the first -f (ignore repeats) */
		    if ((n < argc) && (argv[n][0] != '-')) {
			/* there is a following arg, and it's not an option */
			filename = argv[n++];
			srcfile = fopen(filename, "r");
			if (srcfile == NULL) {
			    fprintf(stderr,
				"Could not open command file '%s'\n",
				filename);
			    exit(-1);
			}
			/* make sure file is closed on exec() */
			fd = fileno(srcfile);
			fcntl(fd, F_SETFD, FD_CLOEXEC);
		    } else {
			/* no filename followed -f option, use stdin */
			srcfile = stdin;
			prompt_mode = 1;
		    }
		}
		break;
#ifndef NO_INI
	    case 'i':
		/* -i = allow reading 'setp' values from an INI file */
		if (inifile == NULL) {
		    /* it's the first -i (ignore repeats) */
		    if ((n < argc) && (argv[n][0] != '-')) {
			/* there is a following arg, and it's not an option */
			filename = argv[n++];
			inifile = fopen(filename, "r");
			if (inifile == NULL) {
			    fprintf(stderr,
				"Could not open INI file '%s'\n",
				filename);
			    exit(-1);
			}
			/* make sure file is closed on exec() */
			fd = fileno(inifile);
			fcntl(fd, F_SETFD, FD_CLOEXEC);
		    } else {
			/* no filename followed -i option, error */
			fprintf(stderr,
			    "No missing INI filename for -i option\n");
			exit(-1);
		    }
		}
		break;
#endif /* NO_INI */
	    default:
		/* unknown option */
		printf("Unknown option '-%c'\n", *cp1);
		break;
	    }
	}
    }
    signal(SIGINT, quit);
    signal(SIGTERM, quit);
    signal(SIGPIPE, SIG_IGN);
    /* at this point all options are parsed, connect to HAL */
    /* create a unique module name, to allow for multiple halrmt's */
    snprintf(comp_name, HAL_NAME_LEN, "halrmt%d", getpid());
    /* tell the signal handler that we might have the mutex */
    hal_flag = 1;
    /* connect to the HAL */
    comp_id = hal_init(comp_name);
    /* done with mutex */
    hal_flag = 0;
    /* check result */
    if (comp_id < 0) {
	fprintf(stderr, "halrmt: hal_init() failed: %d\n", comp_id );
	fprintf(stderr, "NOTE: 'rtapi' kernel module must be loaded\n" );
	return 1;
    }
    hal_ready(comp_id);
    errorcount = 0;
    initSockets();
    /* HAL init is OK, let's process the command(s) */
    /* tell the signal handler we might have the mutex */
    sockMain();
    hal_flag = 1;
    hal_exit(comp_id);
    if ( errorcount > 0 ) {
	return 1;
    } else {
	return 0;
    }
}

