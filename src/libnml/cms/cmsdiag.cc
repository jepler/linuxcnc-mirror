/********************************************************************
* Description: cmsdiag.cc
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: LGPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change: 
********************************************************************/

#include "cmsdiag.hh"
#include "rcsversion.h"
#include <sys/types.h>
#include <unistd.h>		/* getpid() */
#include "timer.hh"		// etime()
#include <stdlib.h>		// memset()
#include <rtapi_string.h> 	// strncpy() -> rtapi_strlcpy()
#include <time.h>		// time_t, time()
#include <math.h>		// floor()
#include "linklist.hh"          // LinkedList
#include "physmem.hh"           // PHYSMEM_HANDLE

CMS_DIAGNOSTICS_INFO::CMS_DIAGNOSTICS_INFO()
{
    last_writer_dpi = NULL;
    last_reader_dpi = NULL;
    dpis = NULL;
}

CMS_DIAGNOSTICS_INFO::~CMS_DIAGNOSTICS_INFO()
{
    last_writer_dpi = NULL;
    last_reader_dpi = NULL;
    if (NULL != dpis) {
	delete dpis;
	dpis = NULL;
    }
}

CMS_DIAG_PROC_INFO *CMS::get_diag_proc_info()
{
    return dpi;
}

void CMS::set_diag_proc_info(CMS_DIAG_PROC_INFO * _dpi)
{
    dpi = _dpi;
}

double cmsdiag_timebias = 0.0;
int cmsdiag_timebias_set = 0;

void CMS::setup_diag_proc_info()
{
    first_diag_store = 1;
    if (NULL == dpi) {
	dpi = new CMS_DIAG_PROC_INFO();
    }
    rtapi_strlcpy(dpi->name, ProcessName, 16);	// process name
    int sysinfo_len = 0;
    memset(dpi->host_sysinfo, 0, 32);
    gethostname(dpi->host_sysinfo, 31);
    sysinfo_len += strlen(dpi->host_sysinfo);
    dpi->host_sysinfo[sysinfo_len++] = ',';
    dpi->host_sysinfo[sysinfo_len++] = ' ';

    if (lib_minor_version < 100) {
	dpi->rcslib_ver = (lib_major_version + (lib_minor_version * 1e-2));
    } else {
	dpi->rcslib_ver = (lib_major_version + (lib_minor_version * 1e-3));
    }

    dpi->pid = getpid();

    dpi->access_type = CMS_ZERO_ACCESS;	// access type of last operation
    dpi->msg_id = 0;		// id of the message written or at time of
    // read.
    dpi->msg_size = 0;		// size of the message written or at time of
    // read.
    dpi->msg_type = 0;		// id of the message written or at time of
    // read.

    dpi->number_of_accesses = 0;
    dpi->number_of_new_messages = 0;
    dpi->bytes_moved = 0;
    dpi->last_access_time = 0;
    dpi->first_access_time = 0;
    dpi->max_difference = 0;
    dpi->min_difference = 0;
    first_diag_store = 1;
    if (!cmsdiag_timebias_set) {
	cmsdiag_timebias_set = 1;
	time_t ttime = time(NULL);
	cmsdiag_timebias = floor(etime() - ttime);
    }
}

void CMS::calculate_and_store_diag_info(PHYSMEM_HANDLE * _handle,
    void *_user_data)
{
    double cmsdiag_curtime = 0.0;
    if (NULL == dpi || _handle == NULL || !enable_diagnostics) {
	return;
    }
    long orig_offset = _handle->offset;
    CMS_DIAG_HEADER dh;

    _handle->enable_byte_counting = 0;
    _handle->read(&dh, sizeof(CMS_DIAG_HEADER));
    if (connection_number == 0 && first_diag_store && dh.last_writer == 0) {
	dh.last_writer = -1;
    }
    if (connection_number == 0 && first_diag_store && dh.last_reader == 0) {
	dh.last_reader = -1;
    }
    if (internal_access_type == CMS_WRITE_ACCESS ||
	internal_access_type == CMS_WRITE_IF_READ_ACCESS) {
	dh.last_writer = connection_number;
    } else if (internal_access_type == CMS_READ_ACCESS) {
	dh.last_reader = connection_number;
    }
    _handle->write(&dh, sizeof(CMS_DIAG_HEADER));
    _handle->offset += sizeof(CMS_DIAG_HEADER);
    _handle->offset += (connection_number * sizeof(CMS_DIAG_PROC_INFO));
    char c;
    _handle->read(&c, 1);
    first_diag_store |= ((c != ProcessName[0] && c != dpi->name[0])
	|| c == 0);
    if (!first_diag_store) {
	_handle->read(dpi, sizeof(CMS_DIAG_PROC_INFO));
    }
    dpi->access_type = internal_access_type;	// access type of last
    // operation
    dpi->msg_id = header.write_id;	// id of the message written or at
    // time of read.
    dpi->msg_size = header.in_buffer_size;
    if (internal_access_type == CMS_WRITE_ACCESS ||
	internal_access_type == CMS_WRITE_IF_READ_ACCESS) {
	if (NULL != _user_data) {
	    dpi->msg_type = *((long *) _user_data);	// id of the message
	    // written or at time 
	    // of read.
	}
    } else {
	if (NULL != subdiv_data) {
	    dpi->msg_type = *((long *) subdiv_data);	// id of the message
	    // written or at time 
	    // of read.
	}
    }
    if (!disable_diag_store) {
	dpi->number_of_accesses++;
    }
    if (dpi->number_of_accesses < 1) {
	dpi->number_of_accesses = 1;
	dpi->number_of_new_messages = 1;
	_handle->total_bytes_moved = 0;
	pre_op_total_bytes_moved = 0;
	first_diag_store = 1;
    }
    if (internal_access_type == CMS_WRITE_ACCESS ||
	internal_access_type == CMS_WRITE_IF_READ_ACCESS ||
	status == CMS_READ_OK) {
	dpi->number_of_new_messages++;
	if (dpi->number_of_new_messages < 1) {
	    dpi->number_of_accesses = 1;
	    dpi->number_of_new_messages = 1;
	    _handle->total_bytes_moved = 0;
	    pre_op_total_bytes_moved = 0;
	    first_diag_store = 1;
	}
    } else if (disable_diag_store) {
	_handle->offset = orig_offset;
	first_diag_store = 0;
	_handle->enable_byte_counting = 1;
	return;
    }

    dpi->bytes_moved +=
	(_handle->total_bytes_moved - pre_op_total_bytes_moved);
    cmsdiag_curtime = etime() - cmsdiag_timebias;
    if (!first_diag_store) {
	double diff = cmsdiag_curtime - dpi->last_access_time;
	if (diff < 0.0) {
	    dpi->bytes_moved = _handle->total_bytes_moved = 0.0;
	    dpi->first_access_time = cmsdiag_curtime;
	    dpi->last_access_time = cmsdiag_curtime;
	    dpi->min_difference = 1e4;
	    dpi->max_difference = 0.0;
	    dpi->number_of_accesses = 0;
	    dpi->number_of_new_messages = 0;
	    _handle->total_bytes_moved = 0;
	    pre_op_total_bytes_moved = 0;
	    first_diag_store = 1;
	}
	if (!disable_diag_store) {
	    if (diff < dpi->min_difference) {
		dpi->min_difference = diff;
	    }
	}
	if (diff > dpi->max_difference) {
	    dpi->max_difference = diff;
	}
	if (!disable_diag_store) {
	    dpi->last_access_time = cmsdiag_curtime;
	}
    } else {
	dpi->bytes_moved = _handle->total_bytes_moved =
	    (_handle->total_bytes_moved - pre_op_total_bytes_moved);
	dpi->first_access_time = cmsdiag_curtime;
	dpi->last_access_time = cmsdiag_curtime;
	dpi->min_difference = 1e4;
	dpi->max_difference = 0.0;
	dpi->number_of_accesses = 1;
	dpi->number_of_new_messages = 1;
	_handle->total_bytes_moved = 0.0;
	pre_op_total_bytes_moved = 0.0;
    }
    _handle->write(dpi, sizeof(CMS_DIAG_PROC_INFO));
    _handle->offset = orig_offset;
    first_diag_store = 0;
    _handle->enable_byte_counting = 1;
}

void CMS::internal_retrieve_diag_info(PHYSMEM_HANDLE * _handle,
    void * /*_user_data*/)
{
    if (NULL == _handle || !enable_diagnostics) {
	return;
    }
    long orig_offset = _handle->offset;
    _handle->enable_byte_counting = 0;
    if (NULL == di) {
	di = new CMS_DIAGNOSTICS_INFO();
	di->dpis = new LinkedList();
    } else {
	di->dpis->delete_members();
    }
    _handle->read(di, sizeof(CMS_DIAG_HEADER));
    _handle->offset += sizeof(CMS_DIAG_HEADER);

    for (int i = 0; i < total_connections; i++) {
	CMS_DIAG_PROC_INFO cms_dpi;
	_handle->read(&cms_dpi, sizeof(CMS_DIAG_PROC_INFO));
	_handle->offset += sizeof(CMS_DIAG_PROC_INFO);
	if (cms_dpi.name[0] != 0 || cms_dpi.number_of_accesses != 0) {
	    di->dpis->store_at_tail(&cms_dpi, sizeof(CMS_DIAG_PROC_INFO), 1);
	    if (i == di->last_writer) {
		di->last_writer_dpi =
		    (CMS_DIAG_PROC_INFO *) di->dpis->get_tail();
	    }
	    if (i == di->last_reader) {
		di->last_reader_dpi =
		    (CMS_DIAG_PROC_INFO *) di->dpis->get_tail();
	    }
	}
    }
    _handle->offset = orig_offset;
    _handle->enable_byte_counting = 1;

}

CMS_DIAGNOSTICS_INFO *CMS::get_diagnostics_info()
{
    if (!enable_diagnostics) {
	return (NULL);
    }

    internal_access_type = CMS_GET_DIAG_INFO_ACCESS;
    status = CMS_STATUS_NOT_SET;
    blocking_timeout = 0;
    main_access(data);
    return (di);
}
