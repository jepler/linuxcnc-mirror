/********************************************************************
* Description: phantom.cc
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

#include "cms.hh"		/* enum CMS_STATUS */
#include "phantom.hh"		/* class PHANTOMMEM */

PHANTOMMEM::PHANTOMMEM(const char *bufline, const char *procline):CMS(bufline, procline)
{
}

PHANTOMMEM::~PHANTOMMEM()
{
}

CMS_STATUS PHANTOMMEM::main_access(void * /*_local*/, int * /*serial_number*/)
{
    switch (internal_access_type) {
    case CMS_READ_ACCESS:
    case CMS_PEEK_ACCESS:
	return (status = CMS_READ_OLD);
    case CMS_WRITE_ACCESS:
    case CMS_WRITE_IF_READ_ACCESS:
	return (status = CMS_WRITE_OK);
    case CMS_CHECK_IF_READ_ACCESS:
    case CMS_CLEAR_ACCESS:
    case CMS_ZERO_ACCESS:
	header.was_read = 0;
	return (status);
    default:
	break;
    }
    return (status);
}
