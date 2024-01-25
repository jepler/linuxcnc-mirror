/*
 * Copyright (C) 2013 Jeff Epler <jepler@unpythonic.net>
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifndef INTERP_BASE_HH
#define INTERP_BASE_HH
#include <stdlib.h>

/* Size of certain arrays */
#define ACTIVE_G_CODES 16
#define ACTIVE_M_CODES 10
#define ACTIVE_SETTINGS 3

class InterpBase {
public:
    virtual ~InterpBase();
    virtual char *error_text(int errcode, char *buf, size_t buflen) = 0;
    virtual char *line_text(char *buf, size_t buflen) = 0;
    virtual char *file_name(char *buf, size_t buflen) = 0;
    virtual char *stack_name(int index, char *buf, size_t buflen) = 0;
    virtual size_t line_length() = 0;
    virtual int sequence_number() = 0;
    virtual int ini_load(const char *inifile) = 0;
    virtual int init() = 0;
    virtual int execute() = 0;
    virtual int execute(const char *line) = 0;
    virtual int execute(const char *line, int line_number) = 0;
    virtual int synch() = 0;
    virtual int exit() = 0;
    virtual int open(const char *filename) = 0;
    virtual int read() = 0;
    virtual int read(const char *line) = 0;
    virtual int close() = 0;
    virtual int reset() = 0;
    virtual int line() = 0;
    virtual int call_level() = 0;
    virtual char *command(char *buf, size_t buflen) = 0;
    virtual char *file(char *buf, size_t buflen) = 0;
    virtual int on_abort(int reason, const char *message) = 0;
    virtual void active_g_codes(int active_gcodes[ACTIVE_G_CODES]) = 0;
    virtual void active_m_codes(int active_mcodes[ACTIVE_M_CODES]) = 0;
    virtual void active_settings(double active_settings[ACTIVE_SETTINGS]) = 0;
    virtual void set_loglevel(int level) = 0;
};

InterpBase *interp_from_shlib(const char *shlib);
extern "C" InterpBase *makeInterp();

#endif
