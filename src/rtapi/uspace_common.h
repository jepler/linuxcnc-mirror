//    Copyright 2006-2014, various authors
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
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <sys/utsname.h>
#include <string.h>
#include <unistd.h>
#include <rtapi_errno.h>
#include <rtapi.h>

static int msg_level = RTAPI_MSG_ERR;	/* message printing level */

#include <sys/ipc.h>		/* IPC_* */
#include <sys/shm.h>		/* shmget() */
/* These structs hold data associated with objects like tasks, etc. */
/* Task handles are pointers to these structs.                      */

typedef struct {
  int magic;			/* to check for valid handle */
  int key;			/* key to shared memory area */
  int id;			/* OS identifier for shmem */
  int count;                    /* count of maps in this process */
  unsigned long int size;	/* size of shared memory area */
  void *mem;			/* pointer to the memory */
} rtapi_shmem_handle;

#define MAX_SHM 64

#define SHMEM_MAGIC   25453	/* random numbers used as signatures */

static rtapi_shmem_handle shmem_array[MAX_SHM] = {{0},};

int rtapi_shmem_new(int key, int module_id, unsigned long int size)
{
  rtapi_shmem_handle *shmem;
  int i;

  for(i=0 ; i < MAX_SHM; i++) {
    if(shmem_array[i].magic == SHMEM_MAGIC && shmem_array[i].key == key) {
      shmem_array[i].count ++;
      return i;
    }
    if(shmem_array[i].magic != SHMEM_MAGIC) break;
  }
  if(i == MAX_SHM)
  {
    rtapi_print_msg(RTAPI_MSG_ERR, "rtapi_shmem_new failed due to MAX_SHM\n");
    return -ENOMEM;
  }
  shmem = &shmem_array[i];

  /* now get shared memory block from OS */
  shmem->id = shmget((key_t) key, (int) size, IPC_CREAT | 0666);
  if (shmem->id == -1) {
    rtapi_print_msg(RTAPI_MSG_ERR, "rtapi_shmem_new failed due to shmget(key=0x%08x): %s\n", key, strerror(errno));
    return -errno;
  }

  struct shmid_ds stat;
  int res = shmctl(shmem->id, IPC_STAT, &stat);
  if(res < 0) perror("shmctl IPC_STAT");

  if(geteuid() == 0 && getuid() != 0) {
    stat.shm_perm.uid = getuid();
  }

  stat.shm_perm.mode |= SHM_LOCKED;
  res = shmctl(shmem->id, IPC_SET, &stat);
  if(res < 0) perror("shmctl IPC_SET");

  /* and map it into process space */
  shmem->mem = shmat(shmem->id, 0, 0);
  if ((ssize_t) (shmem->mem) == -1) {
    rtapi_print_msg(RTAPI_MSG_ERR, "rtapi_shmem_new failed due to shmat()\n");
    return -errno;
  }

  long pagesize = sysconf(_SC_PAGESIZE);
  /* touch every page */
  for(size_t off = 0; off < size; off += pagesize)
  {
      volatile char i = ((char*)shmem->mem)[off];
      (void)i;
  }

  /* label as a valid shmem structure */
  shmem->magic = SHMEM_MAGIC;
  /* fill in the other fields */
  shmem->size = size;
  shmem->key = key;
  shmem->count = 1;

  /* return handle to the caller */
  return i;
}


int rtapi_shmem_getptr(int handle, void **ptr)
{
  rtapi_shmem_handle *shmem;
  if(handle < 0 || handle >= MAX_SHM)
    return -EINVAL;

  shmem = &shmem_array[handle];

  /* validate shmem handle */
  if (shmem->magic != SHMEM_MAGIC)
    return -EINVAL;

  /* pass memory address back to caller */
  *ptr = shmem->mem;
  return 0;
}


int rtapi_shmem_delete(int handle, int module_id)
{
  struct shmid_ds d;
  int r1, r2;
  rtapi_shmem_handle *shmem;

  if(handle < 0 || handle >= MAX_SHM)
    return -EINVAL;

  shmem = &shmem_array[handle];

  /* validate shmem handle */
  if (shmem->magic != SHMEM_MAGIC)
    return -EINVAL;

  shmem->count --;
  if(shmem->count) return 0;

  /* unmap the shared memory */
  r1 = shmdt(shmem->mem);

  /* destroy the shared memory */
  r2 = shmctl(shmem->id, IPC_STAT, &d);
  if(r2 == 0 && d.shm_nattch == 0) {
      r2 = shmctl(shmem->id, IPC_RMID, &d);
  }

  /* free the shmem structure */
  shmem->magic = 0;

  if ((r1 != 0) || (r2 != 0))
    return -EINVAL;
  return 0;
}




void default_rtapi_msg_handler(msg_level_t level, const char *fmt, va_list ap) {
    if(level == RTAPI_MSG_ALL) {
	vfprintf(stdout, fmt, ap);
        fflush(stdout);
    } else {
	vfprintf(stderr, fmt, ap);
        fflush(stderr);
    }
}
static rtapi_msg_handler_t rtapi_msg_handler = default_rtapi_msg_handler;

rtapi_msg_handler_t rtapi_get_msg_handler(void) {
    return rtapi_msg_handler;
}

void rtapi_set_msg_handler(rtapi_msg_handler_t handler) {
    if(handler == NULL) rtapi_msg_handler = default_rtapi_msg_handler;
    else rtapi_msg_handler = handler;
}


void rtapi_print(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    rtapi_msg_handler(RTAPI_MSG_ALL, fmt, args);
    va_end(args);
}


void rtapi_print_msg(msg_level_t level, const char *fmt, ...)
{
    va_list args;

    if ((level <= msg_level) && (msg_level != RTAPI_MSG_NONE)) {
	va_start(args, fmt);
	rtapi_msg_handler(level, fmt, args);
	va_end(args);
    }
}

int rtapi_snprintf(char *buffer, unsigned long int size, const char *msg, ...) {
    va_list args;
    int result;

    va_start(args, msg);
    /* call the normal library vnsprintf() */
    result = vsnprintf(buffer, size, msg, args);
    va_end(args);
    return result;
}

int rtapi_vsnprintf(char *buffer, unsigned long int size, const char *fmt,
	va_list args) {
    return vsnprintf(buffer, size, fmt, args);
}

int rtapi_set_msg_level(int level) {
    msg_level = level;
    return 0;
}

int rtapi_get_msg_level() {
    return msg_level;
}

long long rtapi_get_time(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

#ifdef MSR_H_USABLE
#include <asm/msr.h>
#elif defined(__i386__)
#define rdtscll(val) \
         __asm__ __volatile__("rdtsc" : "=A" (val))
#elif defined(__amd64__)
#define rdtscll(val) \
    ({ unsigned int eax, edx; \
         __asm__ __volatile__("rdtsc" : "=a" (eax), "=d" (edx));  \
        val = ((unsigned long)eax) | (((unsigned long)edx) << 32); })
#else
#define rdtscll(val) ((val) = rtapi_get_time())
#endif

long long rtapi_get_clocks(void)
{
    long long int retval;

    rdtscll(retval);
    return retval;
}

typedef struct {
    unsigned long mutex;
    int           uuid;
} uuid_data_t;

#define UUID_KEY  0x48484c34 /* key for UUID for simulator */

int rtapi_init(const char *modname)
{
    static uuid_data_t* uuid_data   = 0;
    static         int  uuid_mem_id = 0;
    const static   int  uuid_id     = 0;

    static char* uuid_shmem_base = 0;
    int retval,id;
    void *uuid_mem;

    uuid_mem_id = rtapi_shmem_new(UUID_KEY,uuid_id,sizeof(uuid_data_t));
    if (uuid_mem_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
        "rtapi_init: could not open shared memory for uuid\n");
        rtapi_exit(uuid_id);
        return -EINVAL;
    }
    retval = rtapi_shmem_getptr(uuid_mem_id,&uuid_mem);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
        "rtapi_init: could not access shared memory for uuid\n");
        rtapi_exit(uuid_id);
        return -EINVAL;
    }
    if (uuid_shmem_base == 0) {
        uuid_shmem_base =        (char *) uuid_mem;
        uuid_data       = (uuid_data_t *) uuid_mem;
    }
    rtapi_mutex_get(&uuid_data->mutex);
        uuid_data->uuid++;
        id = uuid_data->uuid;
    rtapi_mutex_give(&uuid_data->mutex);

    return id;
}

int rtapi_exit(int module_id)
{
  /* does nothing, for now */
  return 0;
}

int rtapi_is_kernelspace() { return 0; }
static int _rtapi_is_realtime = -1;
static int detect_realtime() {
    struct utsname u;
    int crit1, crit2 = 0;
    FILE *fd;

    uname(&u);
    crit1 = strcasestr (u.version, "PREEMPT RT") != 0;

    if ((fd = fopen("/sys/kernel/realtime","r")) != NULL) {
        int flag;
        crit2 = ((fscanf(fd, "%d", &flag) == 1) && (flag == 1));
        fclose(fd);
    }
    return crit1 && crit2;
}

int rtapi_is_realtime() {
    if(_rtapi_is_realtime == -1) _rtapi_is_realtime = detect_realtime();
    return _rtapi_is_realtime;
}

void rtapi_delay(long ns) {
    struct timespec ts = {0, ns};
    clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, 0);
}

long int rtapi_delay_max() { return 10000; }
