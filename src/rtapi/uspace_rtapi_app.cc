/* Copyright (C) 2006-2014 Jeff Epler <jepler@unpythonic.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "config.h"

#include <sys/fsuid.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <dlfcn.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#ifdef HAVE_SYS_IO_H
#include <sys/io.h>
#endif
#include <sys/resource.h>
#include <sys/mman.h>
#include <malloc.h>
#include <sys/prctl.h>

#include "config.h"

#include "rtapi.h"
#include "rtapi/uspace_common.h"
#include "hal.h"
#include "hal/hal_priv.h"
#include "rtapi_uspace.hh"

#include <sys/ipc.h>		/* IPC_* */
#include <sys/shm.h>		/* shmget() */
#include <string.h>

int WithRoot::level;

namespace
{
RtapiApp &App();
}

static int sim_rtapi_run_threads(int fd, int (*callback)(int fd));

using namespace std;

template<class T> T DLSYM(void *handle, const string &name) {
	return (T)(dlsym(handle, name.c_str()));
}

template<class T> T DLSYM(void *handle, const char *name) {
	return (T)(dlsym(handle, name));
}

static std::map<string, void*> modules;

static int instance_count = 0;
static int force_exit = 0;

static int do_newinst_cmd(string type, string name, string arg) {
    void *module = modules["hal_lib"];
    if(!module) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "newinst: hal_lib is required, but not loaded\n");
        return -1;
    }

    hal_comp_t *(*find_comp_by_name)(char*) =
        DLSYM<hal_comp_t*(*)(char *)>(module, "halpr_find_comp_by_name");
    if(!find_comp_by_name) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "newinst: halpr_find_comp_by_name not found\n");
        return -1;
    }

    hal_comp_t *comp = find_comp_by_name((char*)type.c_str());
    if(!comp) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "newinst: component %s not found\n", type.c_str());
        return -1;
    }

    return comp->make((char*)name.c_str(), (char*)arg.c_str());
}

static int do_one_item(char item_type_char, const string &param_name, const string &param_value, void *vitem, int idx=0) {
    char *endp;
    switch(item_type_char) {
        case 'l': {
            long *litem = *(long**) vitem;
            litem[idx] = strtol(param_value.c_str(), &endp, 0);
	    if(*endp) {
                rtapi_print_msg(RTAPI_MSG_ERR,
                        "`%s' invalid for parameter `%s'",
                        param_value.c_str(), param_name.c_str());
                return -1;
            }
            return 0;
        }
        case 'i': {
            int *iitem = *(int**) vitem;
            iitem[idx] = strtol(param_value.c_str(), &endp, 0);
	    if(*endp) {
                rtapi_print_msg(RTAPI_MSG_ERR,
                        "`%s' invalid for parameter `%s'",
                        param_value.c_str(), param_name.c_str());
                return -1;
            }
            return 0;
        }
        case 's': {
            char **sitem = *(char***) vitem;
            sitem[idx] = strdup(param_value.c_str());
            return 0;
        }
        default:
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "%s: Invalid type character `%c'\n",
                    param_name.c_str(), item_type_char);
            return -1;
        }
    return 0;
}

void remove_quotes(string &s) {
    s.erase(remove_copy(s.begin(), s.end(), s.begin(), '"'), s.end());
}

static int do_comp_args(void *module, vector<string> args) {
    for(unsigned i=1; i < args.size(); i++) {
        string &s = args[i];
	remove_quotes(s);
        size_t idx = s.find('=');
        if(idx == string::npos) {
            rtapi_print_msg(RTAPI_MSG_ERR, "Invalid paramter `%s'\n",
                    s.c_str());
            return -1;
        }
        string param_name(s, 0, idx);
        string param_value(s, idx+1);
        void *item=DLSYM<void*>(module, "rtapi_info_address_" + param_name);
        if(!item) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
                    "Unknown parameter `%s'\n", s.c_str());
            return -1;
        }
        char **item_type=DLSYM<char**>(module, "rtapi_info_type_" + param_name);
        if(!item_type || !*item_type) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
                    "Unknown parameter `%s' (type information missing)\n",
                    s.c_str());
            return -1;
        }
        string item_type_string = *item_type;

        if(item_type_string.size() > 1) {
            int a, b;
            char item_type_char;
            int r = sscanf(item_type_string.c_str(), "%d-%d%c",
                    &a, &b, &item_type_char);
            if(r != 3)
                r = sscanf(item_type_string.c_str(), "%d-(%d)%c",
                    &a, &b, &item_type_char);
            if(r != 3) {
                rtapi_print_msg(RTAPI_MSG_ERR,
                    "Unknown parameter `%s' (corrupt array type information '%s' r=%d)\n",
                    s.c_str(), item_type_string.c_str(), r);
                return -1;
            }
            size_t idx = 0;
            int i = 0;
            while(idx != string::npos) {
                if(i == b) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                            "%s: can only take %d arguments\n",
                            s.c_str(), b);
                    return -1;
                }
                size_t idx1 = param_value.find(",", idx);
                string substr(param_value, idx, idx1 - idx);
                int result = do_one_item(item_type_char, s, substr, item, i);
                if(result != 0) return result;
                i++;
                idx = idx1 == string::npos ? idx1 : idx1 + 1;
            }
        } else {
            char item_type_char = item_type_string[0];
            int result = do_one_item(item_type_char, s, param_value, item);
            if(result != 0) return result;
        }
    }
    return 0;
}

static int do_load_cmd(string name, vector<string> args) {
    void *w = modules[name];
    if(w == NULL) {
        char what[LINELEN+1];
        snprintf(what, LINELEN, "%s/%s.so", EMC2_RTLIB_DIR, name.c_str());
        void *module = modules[name] = dlopen(what, RTLD_GLOBAL | RTLD_NOW);
        if(!module) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: dlopen: %s\n", name.c_str(), dlerror());
            modules.erase(name);
            return -1;
        }
	/// XXX handle arguments
        int (*start)(void) = DLSYM<int(*)(void)>(module, "rtapi_app_main");
        if(!start) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: dlsym: %s\n", name.c_str(), dlerror());
            dlclose(module);
            modules.erase(name);
            return -1;
        }
        int result;

        result = do_comp_args(module, args);
        if(result < 0) {
            dlclose(module);
            modules.erase(name);
            return -1;
        }

        if ((result=start()) < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: rtapi_app_main: %s (%d)\n",
                name.c_str(), strerror(-result), result);
            dlclose(module);
            modules.erase(name);
	    return result;
        } else {
            instance_count ++;
	    return 0;
        }
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: already exists\n", name.c_str());
        return -1;
    }
}

static int do_unload_cmd(string name) {
    void *w = modules[name];
    if(w == NULL) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: not loaded\n", name.c_str());
	return -1;
    } else {
        int (*stop)(void) = DLSYM<int(*)(void)>(w, "rtapi_app_exit");
	if(stop) stop();
	modules.erase(modules.find(name));
        dlclose(w);
        instance_count --;
    }
    return 0;
}

struct ReadError : std::exception {};
struct WriteError : std::exception {};

static int read_number(int fd) {
    int r = 0, neg=1;
    char ch;

    while(1) {
        int res = read(fd, &ch, 1);
        if(res != 1) return -1;
        if(ch == '-') neg = -1;
        else if(ch == ' ') return r * neg;
        else r = 10 * r + ch - '0';
    }
}

static string read_string(int fd) {
    int len = read_number(fd);
    char buf[len];
    if(read(fd, buf, len) != len) throw ReadError();
    return string(buf, len);
}

static vector<string> read_strings(int fd) {
    vector<string> result;
    int count = read_number(fd);
    for(int i=0; i<count; i++) {
        result.push_back(read_string(fd));
    }
    return result;
}

static void write_number(string &buf, int num) {
    char numbuf[10];
    sprintf(numbuf, "%d ", num);
    buf = buf + numbuf;
}

static void write_string(string &buf, string s) {
    write_number(buf, s.size());
    buf += s;
}

static void write_strings(int fd, vector<string> strings) {
    string buf;
    write_number(buf, strings.size());
    for(unsigned int i=0; i<strings.size(); i++) {
        write_string(buf, strings[i]);
    }
    if(write(fd, buf.data(), buf.size()) != (ssize_t)buf.size()) throw WriteError();
}

static int handle_command(vector<string> args) {
    if(args.size() == 0) { return 0; }
    if(args.size() == 1 && args[0] == "exit") {
        force_exit = 1;
        return 0;
    } else if(args.size() >= 2 && args[0] == "load") {
        string name = args[1];
        args.erase(args.begin());
        return do_load_cmd(name, args);
    } else if(args.size() == 2 && args[0] == "unload") {
        return do_unload_cmd(args[1]);
    } else if(args.size() == 3 && args[0] == "newinst") {
        return do_newinst_cmd(args[1], args[2], "");
    } else if(args.size() == 4 && args[0] == "newinst") {
        return do_newinst_cmd(args[1], args[2], args[3]);
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "Unrecognized command starting with %s\n",
                args[0].c_str());
        return -1;
    }
}

static int slave(int fd, vector<string> args) {
    try {
        write_strings(fd, args);
    }
    catch (WriteError &e) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "rtapi_app: failed to write to master: %s\n", strerror(errno));
    }

    int result = read_number(fd);
    return result;
}

static int callback(int fd)
{
    struct sockaddr_un client_addr;
    memset(&client_addr, 0, sizeof(client_addr));
    socklen_t len = sizeof(client_addr);
    int fd1 = accept(fd, (sockaddr*)&client_addr, &len);
    if(fd1 < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "rtapi_app: failed to accept connection from slave: %s\n", strerror(errno));
        return -1;
    } else {
        int result;
        try {
            result = handle_command(read_strings(fd1));
        } catch (ReadError &e) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "rtapi_app: failed to read from slave: %s\n", strerror(errno));
            close(fd1);
            return -1;
        }
        string buf;
        write_number(buf, result);
        if(write(fd1, buf.data(), buf.size()) != (ssize_t)buf.size()) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "rtapi_app: failed to write to slave: %s\n", strerror(errno));
        };
        close(fd1);
    }
    return !force_exit && instance_count > 0;
}

static int master(int fd, vector<string> args) {
    do_load_cmd("hal_lib", vector<string>()); instance_count = 0;
    App(); // force rtapi_app to be created
    if(args.size()) {
        int result = handle_command(args);
        if(result != 0) return result;
        if(force_exit || instance_count == 0) return 0;
    }
    sim_rtapi_run_threads(fd, callback);

    return 0;
}

static std::string
_get_fifo_path() {
    std::string s;
    if(getenv("RTAPI_FIFO_PATH"))
       s = getenv("RTAPI_FIFO_PATH");
    else if(getenv("HOME"))
       s = std::string(getenv("HOME")) + "/.rtapi_fifo";
    else {
       rtapi_print_msg(RTAPI_MSG_ERR,
           "rtapi_app: RTAPI_FIFO_PATH and HOME are unset.  rtapi fifo creation is unsafe.");
       return NULL;
    }
    if(s.size() + 1 > sizeof(sockaddr_un::sun_path)) {
       rtapi_print_msg(RTAPI_MSG_ERR,
           "rtapi_app: rtapi fifo path is too long (arch limit %zd): %s",
               sizeof(sockaddr_un::sun_path), s.c_str());
       return NULL;
    }
    return s;
}

static const char *
get_fifo_path() {
    static std::string path = _get_fifo_path();
    return path.c_str();
}

static int
get_fifo_path(char *buf, size_t bufsize) {
    const char *s = get_fifo_path();
    if(!s) return -1;
    strncpy(buf, s, bufsize);
    return 0;
}

int main(int argc, char **argv) {
    if(getuid() == 0) {
        char *fallback_uid_str = getenv("RTAPI_UID");
        int fallback_uid = fallback_uid_str ? atoi(fallback_uid_str) : 0;
        if(fallback_uid == 0)
        {
            fprintf(stderr,
                "Refusing to run as root without fallback UID specified\n"
                "To run under a debugger with I/O, use e.g.,\n"
                "    sudo env RTAPI_UID=`id -u` RTAPI_FIFO_PATH=$HOME/.rtapi_fifo gdb " EMC2_BIN_DIR "/rtapi_app\n");
            exit(1);
        }
        setreuid(fallback_uid, 0);
        fprintf(stderr,
            "Running with fallback_uid.  getuid()=%d geteuid()=%d\n",
            getuid(), geteuid());
    }
    setfsuid(getuid());
    vector<string> args;
    for(int i=1; i<argc; i++) { args.push_back(string(argv[i])); }

become_master:
    int fd = socket(PF_UNIX, SOCK_STREAM, 0);
    if(fd == -1) { perror("socket"); exit(1); }

    int enable = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    struct sockaddr_un addr;
    addr.sun_family = AF_UNIX;
    if(get_fifo_path(addr.sun_path, sizeof(addr.sun_path)) < 0)
       exit(1);
    int result = ::bind(fd, (sockaddr*)&addr, sizeof(addr));

    if(result == 0) {
        int result = listen(fd, 10);
        if(result != 0) { perror("listen"); exit(1); }
        setsid(); // create a new session if we can...
        result = master(fd, args);
        unlink(get_fifo_path());
        return result;
    } else if(errno == EADDRINUSE) {
        struct timeval t0, t1;
        gettimeofday(&t0, NULL);
        gettimeofday(&t1, NULL);
        for(int i=0; i < 3 || (t1.tv_sec < 3 + t0.tv_sec) ; i++) {
            result = connect(fd, (sockaddr*)&addr, sizeof(addr));
            if(result == 0) break;
            if(i==0) srand48(t0.tv_sec ^ t0.tv_usec);
            usleep(lrand48() % 100000);
            gettimeofday(&t1, NULL);
        }
        if(result < 0 && errno == ECONNREFUSED) {
            unlink(get_fifo_path());
            fprintf(stderr, "Waited 3 seconds for master.  giving up.\n");
            close(fd);
            goto become_master;
        }
        if(result < 0) { perror("connect"); exit(1); }
        return slave(fd, args);
    } else {
        perror("bind"); exit(1);
    }
}


/* These structs hold data associated with objects like tasks, etc. */
/* Task handles are pointers to these structs.                      */

struct rtapi_module {
  int magic;
};

#define MODULE_MAGIC  30812
#define SHMEM_MAGIC   25453

#define MAX_MODULES  64
#define MODULE_OFFSET 32768

rtapi_task::rtapi_task()
    : magic{}, id{}, owner{}, appspecific{}, stacksize{}, prio{},
      period{}, nextstart{},
      ratio{}, arg{}, taskcode{}
{}

namespace
{
struct PosixTask : rtapi_task
{
    PosixTask() : rtapi_task{}, thr{}
    {}

    pthread_t thr;                /* thread's context */
};

struct Posix : RtapiApp
{
    Posix(int policy = SCHED_FIFO) : RtapiApp(policy), do_thread_lock(policy != SCHED_FIFO) {
        pthread_once(&key_once, init_key);
        if(do_thread_lock)
            pthread_mutex_init(&thread_lock, 0);
    }
    int task_delete(int id);
    int task_start(int task_id, unsigned long period_nsec);
    int task_pause(int task_id);
    int task_resume(int task_id);
    int task_self();
    void wait();
    struct rtapi_task *do_task_new() {
        return new PosixTask;
    }
    unsigned char do_inb(unsigned int port);
    void do_outb(unsigned char value, unsigned int port);
    int run_threads(int fd, int (*callback)(int fd));
    static void *wrapper(void *arg);
    bool do_thread_lock;
    pthread_mutex_t thread_lock;

    static pthread_once_t key_once;
    static pthread_key_t key;
    static void init_key(void) {
        pthread_key_create(&key, NULL);
    }
};

static void signal_handler(int sig, siginfo_t *si, void *uctx)
{
    switch (sig) {
    case SIGXCPU:
        // should not happen - must be handled in RTAPI if enabled
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "rtapi_app: BUG: SIGXCPU received - exiting\n");
        exit(0);
        break;

    case SIGTERM:
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "rtapi_app: SIGTERM - shutting down\n");
        exit(0);
        break;

    default: // pretty bad
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "rtapi_app: caught signal %d - dumping core\n", sig);
        sleep(1); // let syslog drain
        signal(sig, SIG_DFL);
        raise(sig);
        break;
    }
    exit(1);
}

const size_t PRE_ALLOC_SIZE = 1024*1024*32;
const static struct rlimit unlimited = {RLIM_INFINITY, RLIM_INFINITY};
static void configure_memory()
{
    int res = setrlimit(RLIMIT_MEMLOCK, &unlimited);
    if(res < 0) perror("setrlimit");

    res = mlockall(MCL_CURRENT | MCL_FUTURE);
    if(res < 0) perror("mlockall");

    /* Turn off malloc trimming.*/
    if (!mallopt(M_TRIM_THRESHOLD, -1)) {
        rtapi_print_msg(RTAPI_MSG_WARN,
                  "mallopt(M_TRIM_THRESHOLD, -1) failed\n");
    }
    /* Turn off mmap usage. */
    if (!mallopt(M_MMAP_MAX, 0)) {
        rtapi_print_msg(RTAPI_MSG_WARN,
                  "mallopt(M_MMAP_MAX, -1) failed\n");
    }
    char *buf = static_cast<char *>(malloc(PRE_ALLOC_SIZE));
    if (buf == NULL) {
        rtapi_print_msg(RTAPI_MSG_WARN, "malloc(PRE_ALLOC_SIZE) failed\n");
        return;
    }
    long pagesize = sysconf(_SC_PAGESIZE);
    /* Touch each page in this piece of memory to get it mapped into RAM */
    for (size_t i = 0; i < PRE_ALLOC_SIZE; i += pagesize) {
            /* Each write to this buffer will generate a pagefault.
             * Once the pagefault is handled a page will be locked in
             * memory and never given back to the system. */
            buf[i] = 0;
    }
    free(buf);
}

static int harden_rt()
{
    if(!rtapi_is_realtime()) return -EINVAL;

    WITH_ROOT;
#if defined(__x86_64__) || defined(__i386__)
    if (iopl(3) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "cannot gain I/O privileges - "
                        "forgot 'sudo make setuid'?\n");
        return -EPERM;
    }
#endif

    struct sigaction sig_act = {};
    // enable realtime
    if (setrlimit(RLIMIT_RTPRIO, &unlimited) < 0)
    {
	rtapi_print_msg(RTAPI_MSG_WARN,
		  "setrlimit(RTLIMIT_RTPRIO): %s\n",
		  strerror(errno));
        return -errno;
    }

    // enable core dumps
    if (setrlimit(RLIMIT_CORE, &unlimited) < 0)
	rtapi_print_msg(RTAPI_MSG_WARN,
		  "setrlimit: %s - core dumps may be truncated or non-existant\n",
		  strerror(errno));

    // even when setuid root
    if (prctl(PR_SET_DUMPABLE, 1) < 0)
	rtapi_print_msg(RTAPI_MSG_WARN,
		  "prctl(PR_SET_DUMPABLE) failed: no core dumps will be created - %d - %s\n",
		  errno, strerror(errno));

    configure_memory();

    sigemptyset( &sig_act.sa_mask );
    sig_act.sa_handler = SIG_IGN;
    sig_act.sa_sigaction = NULL;

    // prevent stopping of RT threads by ^Z
    sigaction(SIGTSTP, &sig_act, (struct sigaction *) NULL);

    sig_act.sa_sigaction = signal_handler;
    sig_act.sa_flags   = SA_SIGINFO;

    sigaction(SIGSEGV, &sig_act, (struct sigaction *) NULL);
    sigaction(SIGILL,  &sig_act, (struct sigaction *) NULL);
    sigaction(SIGFPE,  &sig_act, (struct sigaction *) NULL);
    sigaction(SIGTERM, &sig_act, (struct sigaction *) NULL);
    sigaction(SIGINT, &sig_act, (struct sigaction *) NULL);

    int fd = open("/dev/cpu_dma_latency", O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        rtapi_print_msg(RTAPI_MSG_WARN, "failed to open /dev/cpu_dma_latency: %s\n", strerror(errno));
    }
    setfsuid(getuid());
    if(fd >= 0) {
        int r;
        r = write(fd, "\0\0\0\0", 4);
        if (r != 4) {
            rtapi_print_msg(RTAPI_MSG_WARN, "failed to write to /dev/cpu_dma_latency: %s\n", strerror(errno));
        }
        // deliberately leak fd until program exit
    }
    return 0;
}


static RtapiApp *makeApp()
{
    if(harden_rt() < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Note: Using POSIX non-realtime\n");
        return new Posix(SCHED_OTHER);
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR, "Note: Using POSIX realtime\n");
        return new Posix(SCHED_FIFO);
    }
}
RtapiApp &App()
{
    static RtapiApp *app = makeApp();
    return *app;
}

}
/* data for all tasks */
struct rtapi_task *task_array[MAX_TASKS];

/* Priority functions.  Uspace uses POSIX task priorities. */

int RtapiApp::prio_highest()
{
    return sched_get_priority_max(policy);
}

int RtapiApp::prio_lowest()
{
  return sched_get_priority_min(policy);
}

int RtapiApp::prio_next_higher(int prio)
{
  /* return a valid priority for out of range arg */
  if (prio >= rtapi_prio_highest())
    return rtapi_prio_highest();
  if (prio < rtapi_prio_lowest())
    return rtapi_prio_lowest();

  /* return next higher priority for in-range arg */
  return prio + 1;
}

int RtapiApp::prio_next_lower(int prio)
{
  /* return a valid priority for out of range arg */
  if (prio <= rtapi_prio_lowest())
    return rtapi_prio_lowest();
  if (prio > rtapi_prio_highest())
    return rtapi_prio_highest();
  /* return next lower priority for in-range arg */
  return prio - 1;
}

int RtapiApp::allocate_task_id()
{
    for(int n=0; n<MAX_TASKS; n++)
    {
        rtapi_task **taskptr = &(task_array[n]);
        if(__sync_bool_compare_and_swap(taskptr, (rtapi_task*)0, TASK_MAGIC_INIT))
            return n;
    }
    return -ENOSPC;
}

int RtapiApp::task_new(void (*taskcode) (void*), void *arg,
        int prio, int owner, unsigned long int stacksize, int uses_fp) {
  /* check requested priority */
  if ((prio > rtapi_prio_highest()) || (prio < rtapi_prio_lowest()))
  {
    return -EINVAL;
  }

  /* label as a valid task structure */
  int n = allocate_task_id();
  if(n < 0) return n;

  // cannot use get_task, since task->magic is TASK_MAGIC_INIT
  struct rtapi_task *task = do_task_new();
  if(stacksize < (1024*1024)) stacksize = (1024*1024);
  memset(task, 0, sizeof(*task));
  task->id = n;
  task->owner = owner;
  task->arg = arg;
  task->stacksize = stacksize;
  task->taskcode = taskcode;
  task->prio = prio;
  task->magic = TASK_MAGIC;
  task_array[n] = task;

  /* and return handle to the caller */

  return n;
}

rtapi_task *RtapiApp::get_task(int task_id) {
    if(task_id < 0 || task_id >= MAX_TASKS) return NULL;
    /* validate task handle */
    rtapi_task *task = task_array[task_id];
    if(!task || task == TASK_MAGIC_INIT || task->magic != TASK_MAGIC)
        return NULL;

    return task;
}

template<class T=rtapi_task>
T *get_task(int task_id) {
    return static_cast<T*>(RtapiApp::get_task(task_id));
}

int Posix::task_delete(int id)
{
  auto task = ::get_task<PosixTask>(id);
  if(!task) return -EINVAL;

  pthread_cancel(task->thr);
  pthread_join(task->thr, 0);
  task->magic = 0;
  task_array[id] = 0;
  delete task;
  return 0;
}

int Posix::task_start(int task_id, unsigned long int period_nsec)
{
  auto task = ::get_task<PosixTask>(task_id);
  if(!task) return -EINVAL;

  if(period_nsec < (unsigned long)period) period_nsec = (unsigned long)period;
  task->period = period_nsec;
  task->ratio = period_nsec / period;

  struct sched_param param;
  memset(&param, 0, sizeof(param));
  param.sched_priority = task->prio;

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  int nprocs = sysconf( _SC_NPROCESSORS_ONLN );
  CPU_SET(nprocs-1, &cpuset); // assumes processor numbers are contiguous

  pthread_attr_t attr;
  if(pthread_attr_init(&attr) < 0)
      return -errno;
  if(pthread_attr_setstacksize(&attr, task->stacksize) < 0)
      return -errno;
  if(pthread_attr_setschedpolicy(&attr, policy) < 0)
      return -errno;
  if(pthread_attr_setschedparam(&attr, &param) < 0)
      return -errno;
  if(pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) < 0)
      return -errno;
  if(nprocs > 1)
      if(pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset) < 0)
          return -errno;
  if(pthread_create(&task->thr, &attr, &wrapper, reinterpret_cast<void*>(task)) < 0)
      return -errno;

  return 0;
}

const unsigned long ONE_SEC_IN_NS = 1000000000;
static void advance_clock(struct timespec &result, const struct timespec &src, unsigned long nsec)
{
    time_t sec = src.tv_sec;
    while(nsec >= ONE_SEC_IN_NS)
    {
        ++sec;
        nsec -= ONE_SEC_IN_NS;
    }
    nsec += src.tv_nsec;
    if(nsec >= ONE_SEC_IN_NS)
    {
        ++sec;
        nsec -= ONE_SEC_IN_NS;
    }
    result.tv_sec = sec;
    result.tv_nsec = nsec;
}

#define RTAPI_CLOCK (CLOCK_MONOTONIC)

pthread_once_t Posix::key_once = PTHREAD_ONCE_INIT;
pthread_key_t Posix::key;

void *Posix::wrapper(void *arg)
{
  struct rtapi_task *task;

  /* use the argument to point to the task data */
  task = (struct rtapi_task*)arg;
  long int period = App().period;
  if(task->period < period) task->period = period;
  task->ratio = task->period / period;
  task->period = task->ratio * period;
  rtapi_print_msg(RTAPI_MSG_INFO, "task %p period = %lu ratio=%u\n",
	  task, task->period, task->ratio);

  pthread_setspecific(key, arg);

  Posix &papp = reinterpret_cast<Posix&>(App());
  if(papp.do_thread_lock)
      pthread_mutex_lock(&papp.thread_lock);

  struct timespec now;
  clock_gettime(RTAPI_CLOCK, &now);
  advance_clock(task->nextstart, now, task->period);

  /* call the task function with the task argument */
  (task->taskcode) (task->arg);

  rtapi_print("ERROR: reached end of wrapper for task %d\n", task->id);
  return NULL;
}

int Posix::task_pause(int) {
    return -ENOSYS;
}

int Posix::task_resume(int) {
    return -ENOSYS;
}

int Posix::task_self() {
    struct rtapi_task *task = reinterpret_cast<rtapi_task*>(pthread_getspecific(key));
    if(!task) return -EINVAL;
    return task->id;
}

static bool ts_less(const struct timespec &ta, const struct timespec &tb) {
    if(ta.tv_sec < tb.tv_sec) return 1;
    if(ta.tv_sec > tb.tv_sec) return 0;
    return ta.tv_nsec < tb.tv_nsec;
}

void Posix::wait() {
    if(do_thread_lock)
        pthread_mutex_unlock(&thread_lock);
    pthread_testcancel();
    struct rtapi_task *task = reinterpret_cast<rtapi_task*>(pthread_getspecific(key));
    advance_clock(task->nextstart, task->nextstart, task->period);
    struct timespec now;
    clock_gettime(RTAPI_CLOCK, &now);
    if(ts_less(task->nextstart, now))
    {
        static int printed = 0;
        if(policy == SCHED_FIFO && !printed)
        {
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "Unexpected realtime delay on task %d with period %ld\n"
		    "This Message will only display once per session.\n"
		    "Run the Latency Test and resolve before continuing.\n", 
                    task->id, task->period);
            printed = 1;
        }
    }
    else
    {
        int res = rtapi_clock_nanosleep(RTAPI_CLOCK, TIMER_ABSTIME, &task->nextstart, nullptr, &now);
        if(res < 0) perror("clock_nanosleep");
    }
    if(do_thread_lock)
        pthread_mutex_lock(&thread_lock);
}

unsigned char Posix::do_inb(unsigned int port)
{
#ifdef HAVE_SYS_IO_H
    return inb(port);
#else
    return 0;
#endif
}

void Posix::do_outb(unsigned char val, unsigned int port)
{
#ifdef HAVE_SYS_IO_H
    return outb(val, port);
#endif
}

int rtapi_prio_highest(void)
{
    return App().prio_highest();
}

int rtapi_prio_lowest(void)
{
    return App().prio_lowest();
}

int rtapi_prio_next_higher(int prio)
{
    return App().prio_next_higher(prio);
}

int rtapi_prio_next_lower(int prio)
{
    return App().prio_next_lower(prio);
}

long rtapi_clock_set_period(long nsecs)
{
    return App().clock_set_period(nsecs);
}

long RtapiApp::clock_set_period(long nsecs)
{
  if(nsecs == 0) return period;
  if(period != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "attempt to set period twice\n");
      return -EINVAL;
  }
  period = nsecs;
  return period;
}


int rtapi_task_new(void (*taskcode) (void*), void *arg,
        int prio, int owner, unsigned long int stacksize, int uses_fp) {
    return App().task_new(taskcode, arg, prio, owner, stacksize, uses_fp);
}

int rtapi_task_delete(int id) {
    return App().task_delete(id);
}

int rtapi_task_start(int task_id, unsigned long period_nsec)
{
    return App().task_start(task_id, period_nsec);
}

int rtapi_task_pause(int task_id)
{
    return App().task_pause(task_id);
}

int rtapi_task_resume(int task_id)
{
    return App().task_resume(task_id);
}

int rtapi_task_self()
{
    return App().task_self();
}

void rtapi_wait(void)
{
    App().wait();
}

void rtapi_outb(unsigned char byte, unsigned int port)
{
    App().do_outb(byte, port);
}

unsigned char rtapi_inb(unsigned int port)
{
    return App().do_inb(port);
}

long int simple_strtol(const char *nptr, char **endptr, int base) {
  return strtol(nptr, endptr, base);
}

int Posix::run_threads(int fd, int(*callback)(int fd)) {
    while(callback(fd)) { /* nothing */ }
    return 0;
}

int sim_rtapi_run_threads(int fd, int (*callback)(int fd)) {
    return App().run_threads(fd, callback);
}



