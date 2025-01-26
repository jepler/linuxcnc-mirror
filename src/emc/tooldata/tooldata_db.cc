/*
** Copyright: 2021
** Author:    Dewey Garrett <dgarrett@panix.com>
**
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include "tooldata.hh"
#include <sys/poll.h>

#define DB_VERSION "v2.1"
#define MAX_DB_PROGRAM_ARGS 10
#define UNEXPECTED_MSG fprintf(stderr,"UNEXPECTED %s %d\n",__FILE__,__LINE__);

static bool db_live  = 0;
static bool db_show  = 0; // use environmental var: DB_SHOW
static bool db_debug = 0; // use environmental var: DB_DEBUG

//-----------------------------------------------------------------
/* pipe code adapted from:
** https://jineshkj.wordpress.com/2006/12/22/how-to-capture-stdin-stdout-and-stderr-of-child-program/
*/

#define NUM_PIPES          2
// pipes[0][] parent -------> child
// pipes[1][] parent <------- child
// pipes[][0] is for read
// pipes[][1] is for write
#define PARENT_WRITE_PIPE  0
#define PARENT_READ_PIPE   1

static int pipes[NUM_PIPES][2];

#define READ_FD  0
#define WRITE_FD 1

#define PARENT_READ_FD  ( pipes[PARENT_READ_PIPE] [READ_FD]  )
#define PARENT_WRITE_FD ( pipes[PARENT_WRITE_PIPE][WRITE_FD] )

#define CHILD_READ_FD   ( pipes[PARENT_WRITE_PIPE][READ_FD]  )
#define CHILD_WRITE_FD  ( pipes[PARENT_READ_PIPE] [WRITE_FD] )

#define READ_TIMEOUT_MS 10000

static bool is_random_toolchanger = 0;
static char db_childname[PATH_MAX];

static void handle_sigchild(int /*s*/)
{
    pid_t pid;
    int status;
    while((pid = waitpid(-1, &status, WUNTRACED | WCONTINUED | WNOHANG)) > 0) ;
    if (WIFSIGNALED(status))  {
        db_live = 0;
        fprintf(stderr,"%5d !!!%s terminated by signal %d\n"
               ,getpid(),db_childname,WTERMSIG(status));
    }
    if (WIFEXITED(status)) {
        db_live = 0;
        fprintf(stderr,"%5d ===%s normal exit status=%d\n"
               ,getpid(),db_childname,WEXITSTATUS(status));
    }
#if 0
    if (WIFSTOPPED(status)) {
        fprintf(stderr,"%5d ===%s stopped\n",getpid(),db_childname);
    }
    if (WIFCONTINUED(status)) {
        fprintf(stderr,"%5d ===%s continued\n",getpid(),db_childname);
    }
#endif
}

static int fork_create(int /*myargc*/, char *const myargv[])
{
    // O_DIRECT:packet mode
    if (pipe2(pipes[PARENT_READ_PIPE],O_DIRECT)) {
        perror("E: pipe2");exit(EXIT_FAILURE);
    }
    if (pipe2(pipes[PARENT_WRITE_PIPE],0)) {
        perror("E: pipe2");exit(EXIT_FAILURE);
    }

    signal(SIGCHLD, handle_sigchild);
    pid_t childpid = fork();
    if (childpid == -1) {
         perror("E: fork failed");
         exit(EXIT_FAILURE);
    }

    if(!childpid) { // CHILD------------------------------------
        dup2(CHILD_READ_FD,  STDIN_FILENO);
        dup2(CHILD_WRITE_FD, STDOUT_FILENO);
        close(CHILD_READ_FD);   // not reqd by child
        close(CHILD_WRITE_FD);  // not reqd by child
        close(PARENT_READ_FD);  // not reqd by child
        close(PARENT_WRITE_FD); // not reqd by child
        if (execv(myargv[0], myargv)) {perror("execv fail");}
        return -1;
    } else { // PARENT------------------------------------------
        close(CHILD_READ_FD); //not reqd by parent
        close(CHILD_WRITE_FD);//not reqd by parent
        db_live = 1;
        return 0;
    }
    return -1;
} // fork_create()

static int readready (void)
{
    struct pollfd fds[1];
    fds[0].fd     = PARENT_READ_FD;
    fds[0].events = POLLIN;
    int fdct = poll(fds, 1, READ_TIMEOUT_MS);
    if (fdct == -1) { perror("poll"); return 0; }
    return fds[0].revents & POLLIN; // !=0 == true: ok
} // readready()

static int send_request(char *cmd)
{
    // expect request with \n
    if (db_debug) {fprintf(stderr,"SEND:  %s\n",cmd);}
    if (write(PARENT_WRITE_FD, cmd,strlen(cmd) ) < 0) {
        perror("send_request write");
        return -1;
    }
    return 0;
} // send_request()

static int read_reply(char reply[],int len)
{
    char buffer[CANON_TOOL_ENTRY_LEN];
    int count = 0;
    buffer[count] = 0;
    if (!db_live) {
        fprintf(stderr,"read_reply: db not active\n");
        return -1;
    }
    if ( !readready() ) {
        fprintf(stderr,"!!!! tooldata_db:read_reply fail: timeout_ms=%d\n"
               ,READ_TIMEOUT_MS);
        return -1;
    }
    count = read(PARENT_READ_FD, buffer, sizeof(buffer)-1);
    if (count > 1) {
        buffer[count] = 0;
        // strip \n
        if (buffer[strlen(buffer)-1] =='\n') {
            buffer[strlen(buffer)-1] = 0;
        }
    } else {
        fprintf(stderr,"read_reply IO Error count=%d\n",count);
        return -1;
    }
    snprintf(reply,len,"%s",buffer);
    if (strstr(reply,"NAK")) {
        fprintf(stderr,"!!!! %s\n",reply);
    }
    if (db_debug) {fprintf(stderr,"RECV:  %s\n",reply);}
    return strlen(reply);
} // read_reply()

static int send_and_verify(char* msg)
{
    if (send_request(msg)) { UNEXPECTED_MSG; return -1; }
    char reply[CANON_TOOL_ENTRY_LEN];
    if (read_reply(reply,sizeof(reply)) <0) {
        //fprintf(stderr,"%5d ??????replylen=%lu active=%d\n"
        //        ,getpid(),strlen(reply),db_live);
        return -1;
    }
    //fprintf(stderr,"REPLY: %s\n",reply);
    return 0;
} // send_and_verify()

int tooldata_db_getall() {
    if (!db_live) {return -1;}

    static bool initial_getall = 1;
    if (send_request((char*)"g\n")) {
        UNEXPECTED_MSG; return -1;
    }

    tooldata_reset();  // reset all idx

    if (is_random_toolchanger) {
        int ran_start_idx = 0;
        tooldata_add_init(ran_start_idx);
    } else {
        int nonran_start_idx = 1;
        tooldata_add_init(nonran_start_idx);
    }
    //int ct=1;
    while (1) {
        char reply[CANON_TOOL_ENTRY_LEN];
        if (read_reply(reply,sizeof(reply)) <0) {
            //fprintf(stderr,"%5d ??????replylen=%lu active=%d\n"
            //        ,getpid(),strlen(reply),db_live);
            return -1;
        }
        if (initial_getall || db_show) {fprintf(stderr,"GET:   %s\n",reply);}
        if (strstr(reply,"FINI")) {
            //fprintf(stderr,"=====<ct=%3d>%s\n",ct,reply);
            break;
        }

        int foundidx = tooldata_read_entry(reply);
        //ct++;
        if (foundidx < 0) {
            fprintf(stderr,"!!!tooldata_db_getall %s\n",reply);
            return -1;
        }
    }
    // update the single-entry tbl file (typ: db_spindle.tbl)
    tooldata_save((char*)NULL);
    initial_getall = 0;
    return 0;
} // tooldata_db_getall()

int tooldata_db_init(char progname_plus_args[],int random_toolchanger)
{
    if (getenv( (char*)"DB_DEBUG") ) {db_debug = 1;}
    if (getenv( (char*)"DB_SHOW") )  {db_show  = 1;}
    int   child_argc = 0;
    char* child_argv[MAX_DB_PROGRAM_ARGS] = {0};
    char* token = strtok(progname_plus_args, " ");
    while (token != NULL) {
        child_argv[child_argc] = token;
        child_argc++;
        if (child_argc >= MAX_DB_PROGRAM_ARGS) {
            fprintf(stderr,"!!!!db_init: argc exceeds MAX_DB_PROGRAM_ARGS=%d\n"
                   ,MAX_DB_PROGRAM_ARGS);
            return -1;
        }
        token = strtok(NULL, " ");
    }
    snprintf(db_childname,sizeof(db_childname),"%s",child_argv[0]);
    is_random_toolchanger = random_toolchanger;

    if (access(child_argv[0],X_OK)) {
        fprintf(stderr,"!!!!db_init: <%s> not executable\n",child_argv[0]);
        return -1;
    }
    //fprintf(stderr,"=====db_childname=%s\n",db_childname);
    if (0 == fork_create(child_argc,child_argv) ) {
        //fprintf(stderr,"=====db_init forked %s\n",child_argv[0]);
    } else {
        fprintf(stderr,"!!!!!db_init: fork FAIL\n");
        perror("FORK");
        return -1;
    }

    // block for response
    fprintf(stderr,"====Waiting for %s version reply from %s\n",
            DB_VERSION,child_argv[0]);
    char reply[CANON_TOOL_ENTRY_LEN];

    if (read_reply(reply,sizeof(reply)) < 0 ) {
        fprintf(stderr,"!!!! tooldata_db_init: read_reply initial read failed\n");
        db_live = 0;
        return -1;
    } else {
        if (strncmp(reply,DB_VERSION,sizeof(DB_VERSION)) != 0) {
            fprintf(stderr,
                    "!!!! db_version MISMATCH require:%s received:%s\n",
                    DB_VERSION,reply);
            db_live = 0;
            return -1;
        } else {
            fprintf(stderr,"====Connected to %s\n",child_argv[0]);
        }
    }
    if (db_debug) {
        if (tooldata_db_getall()) {
            fprintf(stderr,"!!!! tooldata_db_init() get all fail\n");
            return -1;
        }
    }
    return 0;
} // tooldata_db_init()


int tooldata_db_notify(tool_notify_t ntype,
                       int toolno,
                       int pocketno,
                       CANON_TOOL_TABLE tdata)
{
    if (!db_live) return 0;   //silently ignore
    char msg[CANON_TOOL_ENTRY_LEN +20];
    char buffer[CANON_TOOL_ENTRY_LEN] = {0};

    // make a tool line suitable for notifying db_program
    // caller may specify pocketno different from tdata.pocketno
    CANON_TOOL_TABLE notifydata;
    notifydata = tooldata_entry_init(); //nulls
    notifydata.toolno   = toolno;
    notifydata.pocketno = pocketno;
    switch (ntype) {
    case SPINDLE_LOAD: // 'l' command
         tooldata_format_toolline(pocketno,
                                  1, // ignore_zero_values
                                  notifydata, buffer);
         snprintf(msg,sizeof(msg),"l %s\n",buffer);
         if (db_debug) {fprintf(stderr,"SPINDLE_LOAD:%s\n",msg);}
         break;
    case SPINDLE_UNLOAD: // 'u' command
         tooldata_format_toolline(pocketno,
                                  1, // ignore_zero_values
                                  notifydata, buffer);
         snprintf(msg,sizeof(msg),"u %s\n",buffer);
         if (db_debug) {fprintf(stderr,"SPINDLE_UNLOAD:%s\n",msg);}
         break;
    case TOOL_OFFSET: // 'p' command
         notifydata          = tdata;  // from caller
         notifydata.toolno   = toolno;
         notifydata.pocketno = pocketno;
         tooldata_format_toolline(pocketno,
                                  0, // do not ignore_zero_values
                                  notifydata, buffer);
         snprintf(msg,sizeof(msg),"p %s\n",buffer);
         if (db_debug) {fprintf(stderr,"PUT:   %s\n",msg);}
         break;
    default: UNEXPECTED_MSG;
    }
    send_and_verify(msg);

    return 0;
} // tooldata_db_notify()

#undef UNEXPECTED_MSG
