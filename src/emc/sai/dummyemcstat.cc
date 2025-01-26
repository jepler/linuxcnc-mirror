/*    This is a component of LinuxCNC
 *    Copyright 2011 Michael Haberler <git@mah.priv.at>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
// keep linker happy so TaskMod can be resolved

#include "rcs.hh"		// NML classes, nmlErrorFormat()
#include "emc.hh"		// EMC NML
#include "emc_nml.hh"
#include "rs274ngc.hh"
#include "rs274ngc_interp.hh"

EMC_STAT *emcStatus = new EMC_STAT;

int emcOperatorDisplay(char const*, ...) {return 0;};

int emcOperatorText(char const*, ...) {return 0;}

// int emcOperatorError(char const*, ...) {return 0;}


int emcAbortCleanup(EMC_ABORT reason, const char *message)
{
    printf("on_abort: [%d] %s\n", (int)reason,message);
    return 0;
}

extern void emctask_quit(int /*sig*/) {};
