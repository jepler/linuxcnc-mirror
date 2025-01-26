/* Classic Ladder Project */
/* Copyright (C) 2001-2010 Marc Le Douarain */
/* http://www.multimania.com/mavati/classicladder */
/* http://www.sourceforge.net/projects/classicladder */
/* February 2001 */
/* ------------------------------ */
/* The main for Linux Application */
/* ------------------------------ */
/* This library is free software; you can redistribute it and/or */
/* modify it under the terms of the GNU Lesser General Public */
/* License as published by the Free Software Foundation; either */
/* version 2.1 of the License, or (at your option) any later version. */

/* This library is distributed in the hope that it will be useful, */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU */
/* Lesser General Public License for more details. */

/* You should have received a copy of the GNU Lesser General Public */
/* License along with this library; if not, write to the Free Software */
/* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. */

// This is an adaption of classicladder V7.124 For EMC
// most of this file is very different from the original
// it uses RTAPI realtime code and HAL code for memory allocation and input/output.
// this adaptation was started Jan 2008 by Chris Morley  

#include "../config.h"
#ifdef MODULE
#include <linux/string.h>
#else
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#endif

#include "hal.h"
#include "classicladder.h"
#include "global.h"
#include "files_project.h"
#include "calc.h"
#include "vars_access.h"
#include "manager.h"
#include "calc_sequential.h"
#include "files_sequential.h"
//#include "config.h"
// #include "hardware.h"
#include "socket_server.h"
#include "socket_modbus_master.h"
//#include "vars_system.h"

#if !defined( MODULE )
#include <gtk/gtk.h>
#include "classicladder_gtk.h"
#include "manager_gtk.h"
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <signal.h>
#include <getopt.h>
#ifdef __WIN32__
#include <windows.h>
#else
#include <pthread.h>
#endif
#ifdef __XENO__
#include <sys/mman.h>
#endif
#endif

#include <rtapi_string.h>
#ifdef GTK_INTERFACE
#include <gtk/gtk.h>
#include "menu_and_toolbar_gtk.h"
#include <libintl.h>
#include <locale.h>
#endif

#ifdef MAT_CONNECTION
#include "../../lib/plc.h"
#endif

int cl_remote;
int nogui = 0,modmaster=0,modslave=0,pathswitch=0;
int ModbusServerPort = 9502; // Standard "502" requires root privileges...
int CyclicThreadRunning = 0;
char  *NewPath;

void ClassicLadderEndOfAppli( void )
{
	CyclicThreadRunning = 0;
	if(modmaster)  {   CloseSocketModbusMaster( );   }
	if(modslave)   {   CloseSocketServer( );   }

}

void display_help (void)
{
	printf( CL_PRODUCT_NAME " v" CL_RELEASE_VER_STRING "\n" CL_RELEASE_DATE_STRING "\n"
	       "\n"
               "Copyright (C) " CL_RELEASE_COPYRIGHT_YEARS " Marc Le Douarain\nmarc . le - douarain /At\\ laposte \\DoT/ net\n"
	       "\n"
	       "Adapted to LinuxCNC\n"
			"\n"
	       "ClassicLadder comes with NO WARRANTY\n"
	       "to the extent permitted by law.\n"
	       "\n"
	       "You may redistribute copies of ClassicLadder\n"
	       "under the terms of the GNU Lesser General Public Licence.\n"
	       "See the file `lesserGPL.txt' for more information.\n");
	
    printf("This version of Classicladder is adapted for use with LinuxCNC and HAL\n"
	       "\nUsage: classicladder [OPTIONS] [PATH]\n"
	       "eg: loadusr -w classicladder  ladtest.clp\n"
	       "eg: loadusr -w classicladder  --nogui ladtest.clp\n"
	       "eg: loadusr -w classicladder  --modmaster ladtest.clp\n"
	       "\n"
	       "   --nogui            do not create a GUI, only load a configuration\n"
	       "   --modmaster        initialize modbus master I/O ( modbus config is loaded with other objects )\n"
	       "   --modslave         initialize modbus slave I/O (TCP only- B and W variables accessible\n"
	       "   --modbus_port=portnumber  used for modbus slave using TCP ( ethernet )\n"
	       "   --debug            sets the RTAPI debuglevel for printing debug messages\n"
	       "Please also note that the classicladder realtime module must be loaded first\n"
	       "eg: loadrt classicladder_rt    for default number of ladder objects\n"  
			    );
	hal_exit(compId); // add for LinuxCNC
	exit(0);
}

void process_options (int argc, char *argv[])
{
	int error = 0;

	for (;;)
	{
		int option_index = 0;
		static const struct option long_options[] = {
			{"nogui", no_argument, 0, 'n'},
			{"modmaster",no_argument,0,'m'},
			{"modslave",no_argument,0,'s'},
			{"debug",no_argument,0,'d'},
			{"modbus_port", required_argument, 0, 'p'},
			{"newpath", required_argument, 0, 'f'},
			{0, 0, 0, 0},
		};

		int c = getopt_long(argc, argv, "",
				    long_options, &option_index);
		if (c == EOF)
			break;

		switch (c)
		{
			
			case 'n':
				nogui = 1;
				break;
			case 'm':
				modmaster=1;
				break;
			case 's':
				modslave=1;
				break; 
			case 'd':
				rtapi_set_msg_level(RTAPI_MSG_ALL);
				break;
			case 'p':
				ModbusServerPort = atoi( optarg );
				break;
			case 'f':
				NewPath = ( optarg );
				pathswitch=1;
				break;
			case '?':
				error = 1;
				break;
		}
	}

	if (error)
		display_help ();

	if ((argc - optind) != 0)
		VerifyDirectorySelected (argv[optind]);
}
//for EMC: do_exit
static void do_exit(int unused) {
		(void)unused;
		hal_exit(compId);
		printf(_("ERROR CLASSICLADDER-   Error initializing classicladder user module.\n"));
		exit(0);
}

void DoPauseMilliSecs( int MilliSecsTime )
{
#ifdef __WIN32__
	Sleep( MilliSecsTime );
#else
	struct timespec time;
	int NbrSecs =0;
	int NbrNanos = MilliSecsTime*1000000;
	if ( MilliSecsTime>=1000 )
	{
		NbrSecs = MilliSecsTime/1000;
		NbrNanos = (MilliSecsTime%1000)*1000000;
	}
	time.tv_sec = NbrSecs;
	time.tv_nsec = NbrNanos;
	nanosleep( &time, NULL );
	//usleep( Time*1000 );
#endif
}

void DoFlipFlopRunStop( void )
{
	if (InfosGene->LadderState==STATE_RUN)
	{
		InfosGene->LadderState = STATE_STOP;
#ifdef GTK_INTERFACE
//		gtk_label_set_text(GTK_LABEL(GTK_BIN(ButtonRunStop)->child),"Run");
		MessageInStatusBar(_("Stopped program - press run button to continue."));
//		SetToggleMenuForRunStop( TRUE );
		SetMenuStateForRunStopSwitch( FALSE );
#endif
	}
	else
	{
		InfosGene->LadderState = STATE_RUN;
#ifdef GTK_INTERFACE
//		gtk_label_set_text(GTK_LABEL(GTK_BIN(ButtonRunStop)->child),"Stop");
		MessageInStatusBar(_("Started program - press stop to pause."));
//		SetToggleMenuForRunStop( FALSE );
		SetMenuStateForRunStopSwitch( TRUE );
#endif
	}
}

void StopRunIfRunning( void )
{
	if (InfosGene->LadderState==STATE_RUN)
	{
		debug_printf("Stopping...");
		InfosGene->LadderStoppedToRunBack = TRUE;
		InfosGene->LadderState = STATE_STOP;
		while( InfosGene->UnderCalculationPleaseWait==TRUE )
		{
			DoPauseMilliSecs( 100 );
		}
		debug_printf("done.\n");
	}
}
void RunBackIfStopped( void )
{
	if ( InfosGene->LadderStoppedToRunBack )
	{
		debug_printf("Start running!\n");
		InfosGene->LadderState = STATE_RUN;
		InfosGene->LadderStoppedToRunBack = FALSE;
	}
}
// after processing options and initializing HAL, MODBUS and registering shared memory
// the main function is divided into  NOGUI true or NOGUI FALSE
// The difference between them is mostly about checking if a program has already been loaded (only when GUI is to be shown)
// if rungs are used and a ladder program was not specified on the command line
// then a program is already in memory so we dont re initialize memory we just start the GUI

int main( int   argc, char *argv[] )
{
	int used=0, NumRung;
	static int old_level ;
         bindtextdomain("linuxcnc", EMC2_PO_DIR);
         setlocale(LC_MESSAGES,"");
         setlocale(LC_CTYPE,"");
         textdomain("linuxcnc");
	old_level = rtapi_get_msg_level();
	compId=hal_init("classicladder"); //emc
	if (compId<0) return -1; //emc
	signal(SIGTERM,do_exit); //emc
	InitModbusMasterBeforeReadConf( );
	if (ClassicLadder_AllocAll())
	{
		char ProjectLoadedOk=TRUE;		
		process_options (argc, argv);
		if (nogui==TRUE)
		{
			rtapi_print(_("INFO CLASSICLADDER-   No ladder GUI requested-Realtime runs till HAL closes.\n"));
			ClassicLadder_InitAllDatas( );
			ProjectLoadedOk = LoadProjectFiles( InfosGene->CurrentProjectFileName  );
			if (pathswitch){   rtapi_strxcpy( InfosGene->CurrentProjectFileName, NewPath );   }
			InfosGene->LadderState = STATE_RUN;
			ClassicLadder_FreeAll(TRUE);
			hal_ready(compId);
			hal_exit(compId);	
			return 0; 
		} else {	
						
				for(NumRung=0;NumRung<NBR_RUNGS;NumRung++)   {   if ( RungArray[NumRung].Used ) used++;   }
				if((used==0) || ( (argc - optind) != 0) )
					    {	
						ClassicLadder_InitAllDatas( );
						ProjectLoadedOk = LoadProjectFiles( InfosGene->CurrentProjectFileName );
						InitGtkWindows( argc, argv );
						UpdateAllGtkWindows();
						if (pathswitch){   rtapi_strxcpy( InfosGene->CurrentProjectFileName, NewPath );   }
						UpdateWindowTitleWithProjectName( );
						MessageInStatusBar( ProjectLoadedOk?_("Project loaded and running"):_("Project failed to load..."));
						if (!ProjectLoadedOk) 
						{  
							   ClassicLadder_InitAllDatas( );   
							   if (modmaster) {    PrepareModbusMaster( );    }
						}
					    }else{
							   InitGtkWindows( argc, argv );
							   UpdateAllGtkWindows();
							   if (pathswitch){   rtapi_strxcpy( InfosGene->CurrentProjectFileName, NewPath );   }
							   UpdateWindowTitleWithProjectName( );
							   MessageInStatusBar(_("GUI reloaded with existing ladder program"));
							   if (modmaster) {    PrepareModbusMaster( );    }
							} 
							
				if (modslave)         {   InitSocketServer( 0/*UseUdpMode*/, ModbusServerPort/*PortNbr*/);  }
				InfosGene->LadderState = STATE_RUN;
				hal_ready(compId);
				gtk_main();
				rtapi_print(_("INFO CLASSICLADDER-   Ladder GUI closed. Realtime runs till HAL closes\n"));
				ClassicLadder_FreeAll(TRUE);
				hal_exit(compId);
				return 0;
			}		
	}
	 rtapi_print(_("ERROR CLASSICLADDER-   Ladder memory allocation error\n"));
	ClassicLadder_FreeAll(TRUE);
	rtapi_set_msg_level(old_level);
	hal_exit(compId);		
	return 0;
}
