/* Classic Ladder Project */
/* Copyright (C) 2001-2009 Marc Le Douarain */
/* http://www.multimania.com/mavati/classicladder */
/* http://www.sourceforge.net/projects/classicladder */
/* August 2005 */
/* ------------------------------------------- */
/* Socket for Modbus master (Distributed I/O)  */
/* + making call to low-level serial functions */
/* if this is the mode used                    */
/* ------------------------------------------- */

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
// Chris Morley July 08 (EMC)

#include <locale.h>
#include <libintl.h>
#define _(x) gettext(x)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __WIN32__
#include <winsock2.h>
#include <windows.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#endif
#include <errno.h>
#include <time.h>
#include <rtapi_string.h>

#include "classicladder.h"
#include "global.h"
#include "socket_modbus_master.h"
#include "protocol_modbus_master.h"
#include "serial_common.h"

#ifdef __WIN32__
#define SOCK_FD SOCKET
#define SOCK_INVALID SOCKET_ERROR
HANDLE ThreadHandleClient = NULL;
DWORD ThreadIdClient;
#else
#define SOCK_FD int
#define SOCK_INVALID (-1)
pthread_t thread_socket_client;
#endif

#define BUF_SIZE 512

// will work, if all reqs are different IP address... (perhaps too much and not necessary...)
#define NBR_CLIENTS_MAX NBR_MODBUS_MASTER_REQ

int ClientSocketRunning = 0;

int ClientSocketOpened[ NBR_CLIENTS_MAX ]; // -1 if not opened, else req nbr (to retrieve directly IP address)
SOCK_FD ClientSockDescrip[ NBR_CLIENTS_MAX ]; 
SOCK_FD client_s = SOCK_INVALID;             // Current client socket descriptor

//static int StatsNbrErrorsNoResponse = 0;
static int StatsNbrErrorsModbusTreat = 0;
static int StatsNbrFrames = 0;

void InitSocketModbusMaster( )
{

	int Error;
	int ScanClientSock;

	// WSAStartup for Windows already done for socket server...
	
	//InitModbusMasterParams( );
	for( ScanClientSock=0; ScanClientSock<NBR_CLIENTS_MAX; ScanClientSock++ )
	{
		ClientSocketOpened[ ScanClientSock ] = -1;
		ClientSockDescrip[ ScanClientSock ] = SOCK_INVALID;
	}

	ClientSocketRunning = 1;
#ifdef __WIN32__
	ThreadHandleClient = CreateThread( NULL/*no security attributes*/, 16*1024L/*default stack size*/,                                                   
			(LPTHREAD_START_ROUTINE)SocketModbusMasterLoop/* thread function*/, 
			NULL/*argument to thread function*/,                
			THREAD_QUERY_INFORMATION/*use default creation flags*/,                           
			&ThreadIdClient/* returns the thread identifier*/);                
	if ( ThreadHandleClient==NULL )
#else
	Error = pthread_create( &thread_socket_client, NULL, (void *(*)(void *))SocketModbusMasterLoop, (void *)NULL );
	if (Error)
#endif
	{
		printf(_("ERROR CLASSICLADDER-   Failed to create thread I/O Modbus master...\n"));
		CloseSocketModbusMaster( );
	}
	else
	{
		ConfigSerialModbusMaster( );
	}
}

void ConfigSerialModbusMaster( void )
{
	int Error = 0;
	if ( ModbusConfig.ModbusSerialPortNameUsed[ 0 ]!='\0' )
	{
		if ( !SerialOpen( /*ModbusConfig.ModbusSerialPortNameUsed, ModbusConfig.ModbusSerialSpeed*/ ) )
		{
			Error = -1;
                        printf(_("INFO CLASSICLADDER---I/O Modbus master Data bits %i Stop bits %i Parity %i\n"),ModbusConfig.ModbusSerialDataBits,ModbusConfig.ModbusSerialStopBits,ModbusConfig.ModbusSerialParity);
		}
		if ( Error!=-1 )
		printf(_("INFO CLASSICLADDER---I/O Modbus master (%s) init ok !\n"), ModbusConfig.ModbusSerialPortNameUsed[ 0 ]!='\0'?_("Serial"):_("Ethernet"));
	}
}


/* if not already connected => connect to slave... */
char VerifyTcpConnection( char * SlaveAdr )
{
	char AlreadyConnected = FALSE;
	char FreeOneFound = FALSE;
	// verify all connections, to see if one is opened with the same IP address...
	int ScanClientSock = 0;
	client_s = SOCK_INVALID;
	do
	{
		if ( ClientSocketOpened[ ScanClientSock ]!=-1 )
		{
			if ( strcmp( SlaveAdr, ModbusMasterReq[ ClientSocketOpened[ ScanClientSock ] ].SlaveAdr )==0 )
			{
				AlreadyConnected = TRUE;
			}
		}
		if ( !AlreadyConnected )
			ScanClientSock++;
	}
	while( !AlreadyConnected && ScanClientSock<NBR_CLIENTS_MAX );
	
	// if not connected => find a free
	if ( !AlreadyConnected )
	{
		ScanClientSock = 0;
		do
		{
			if ( ClientSocketOpened[ ScanClientSock ]==-1 )
				FreeOneFound = TRUE;
			else
				ScanClientSock++;
		}
		while( !FreeOneFound && ScanClientSock<NBR_CLIENTS_MAX );
		
		// connect to slave now !
		if ( FreeOneFound )
		{
			struct sockaddr_in io_module_addr;          // Server Internet address

			if( ModbusConfig.ModbusDebugLevel>=2 )
				printf(_("Init socket for I/O module (%d)...\n"), ScanClientSock);	
			client_s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
			if ( client_s==SOCK_INVALID )
			{
				printf(_("Failed to open I/O socket master...\n"));
			}
			else
			{
				int NumPort = 502; // default Modbus port
				char * PosiSep;
				memset(&io_module_addr, 0, sizeof(io_module_addr));     /* Zero out structure */
				io_module_addr.sin_family = AF_INET;             /* Internet address family */
				
				// verify if port given in string ?
				PosiSep = strchr( SlaveAdr, ':' );
				if ( PosiSep==NULL )
				{
					io_module_addr.sin_addr.s_addr = inet_addr( SlaveAdr );   /* Server IP address */
				}
				else
				{
					static char Address[ 50 ];
					rtapi_strxcpy( Address, SlaveAdr );
					Address[ PosiSep-SlaveAdr ] = '\0';
					NumPort = atoi( PosiSep+1 );
					io_module_addr.sin_addr.s_addr = inet_addr( Address );   /* Server IP address */
				}
				io_module_addr.sin_port = htons( NumPort ); /* Server port */

				if( ModbusConfig.ModbusDebugLevel>=2 )
					printf(_("Connecting I/O module...\n"));
				/* Establish the connection with the I/O module */
				if (connect(client_s, (struct sockaddr *) &io_module_addr, sizeof(io_module_addr)) >= 0)
				{
					int ScanModbusTableForIpAddr = 0;
					int OffsetIpAdrFound = -1;
					do
					{
						if ( strcmp( SlaveAdr, ModbusMasterReq[ ScanModbusTableForIpAddr ].SlaveAdr )==0 )
							OffsetIpAdrFound = ScanModbusTableForIpAddr;
						else
							ScanModbusTableForIpAddr++;
					}
					while( OffsetIpAdrFound==-1 && ScanModbusTableForIpAddr<NBR_MODBUS_MASTER_REQ );
					if ( OffsetIpAdrFound!=-1 )
					{
						AlreadyConnected = TRUE;
						ClientSocketOpened[ ScanClientSock ] = OffsetIpAdrFound;
						ClientSockDescrip[ ScanClientSock ] = client_s;
					}
					else
					{
						printf(_("Not able to retrieve IP address in Modbus table, huhh?!!!\n"));
					}
				}
				else
				{
					printf(_("Failed  to connect to I/O module\n")); 
				}
			}
		}
		else
		{
			printf(_("Too much IP connections under run...\n"));
		}
	}
	else
	{
		client_s = ClientSockDescrip[ ScanClientSock ];
	}
	return AlreadyConnected;
}

int SendSocketModbusMaster( char * SlaveAdr, int NumPort, char * Frame, int LgtFrame )
{
	int Status = -1;
	int LgtSend;
	(void)NumPort;
	if ( VerifyTcpConnection( SlaveAdr ) )
	{
		if( ModbusConfig.ModbusDebugLevel>=2 )
			debug_printf(_("INFO CLASSICLADDER-   Sending frame to I/O module...\n"));
		/* Send the Modbus frame */
		LgtSend = send(client_s, Frame, LgtFrame, 0);
		if ( LgtSend==LgtFrame )
			Status = 0;
		else
			debug_printf(_("ERROR CLASSICLADDER-  FAILED TO SEND ON SOCKET !!!(LgtSend=%d)\n"),LgtSend);
	}
	return Status;
}

int WaitRespSocketModbusMaster( char * Buff, int BuffSize, int TimeOutResponseMilliSecs )
{
	int ResponseSize = 0;
	if ( client_s!=SOCK_INVALID )
	{
		int recep_descrip;
		fd_set myset;
		struct timeval tv;
		FD_ZERO( &myset);
		// add descrip to survey and set time-out wanted !
		FD_SET( client_s, &myset );
		tv.tv_sec = 0; //seconds
		tv.tv_usec = TimeOutResponseMilliSecs*1000; //micro-seconds
		recep_descrip = select( 16, &myset, NULL, NULL, &tv );
		if ( recep_descrip>0 )
		{
			int bytesRcvd;
			if( ModbusConfig.ModbusDebugLevel>=2 )
				printf(_("INFO CLASSICLADDER-   waiting for slave response...\n"));
			if ((bytesRcvd = recv(client_s, Buff, BuffSize, 0)) > 0)
				ResponseSize = bytesRcvd;
		
		}
	}
	return ResponseSize;
}


void CloseSocketModbusMaster( void )
{
	int ScanClientSock;
	ClientSocketRunning = 0;
#ifdef __WIN32__
	if ( ThreadHandleClient )
		TerminateThread( ThreadHandleClient, 0);
#endif	
	// close sockets
	for( ScanClientSock=0; ScanClientSock<NBR_CLIENTS_MAX; ScanClientSock++ )
	{
		if ( ClientSocketOpened[ ScanClientSock ]!=-1 )
		{
#ifdef __WIN32__
			closesocket( ClientSockDescrip[ ScanClientSock ] );
#else
			close( ClientSockDescrip[ ScanClientSock ] );
#endif
		}
		ClientSocketOpened[ ScanClientSock ] = -1;
		ClientSockDescrip[ ScanClientSock ] = SOCK_INVALID;
	}
	if ( ModbusConfig.ModbusSerialPortNameUsed[ 0 ]!='\0' )
		SerialClose( );
	printf(_("INFO CLASSICLADDER---I/O Modbus master closed!\n"));
}

void SocketModbusMasterLoop( void )
{
	char AdrIP[ 30 ];
	int SizeQuestionToAsk;
	static char QuestionFrame[ 800 ];
	int ResponseSize;
	static char ResponseFrame[ 800 ];
	int SendResultStatus = 0;

#ifdef __XENO__
	pthread_set_name_np(pthread_self(), __FUNCTION__);
#endif

	while( ClientSocketRunning )
	{
// TODO: added for XENO... not found why required for now...
// (task suspended otherwise with the watchdog, seen with dmesg!)
// removing this next line for EMC- we don't use XENO

//		DoPauseMilliSecs( 10 );

		if (InfosGene->LadderState!=STATE_RUN || ( ModbusConfig.ModbusSerialPortNameUsed[ 0 ]!='\0' && !SerialPortIsOpened() ) )
		{
			DoPauseMilliSecs( ModbusConfig.ModbusTimeInterFrame );
		}
		else
		{
			SizeQuestionToAsk = ModbusMasterAsk( (unsigned char*)AdrIP, (unsigned char*)QuestionFrame );
			if ( SizeQuestionToAsk>0 )
			{
				if ( ModbusConfig.ModbusSerialPortNameUsed[ 0 ]=='\0' )
				{
					SendResultStatus = SendSocketModbusMaster( AdrIP, 502, QuestionFrame, SizeQuestionToAsk );
				}
				else
				{
					int NbrCharsWaited = 1/*adr*/+GetModbusResponseLenghtToReceive()+2/*crc*/;
					// before sending question, set size of frame that will be to receive after! 
//					if( ModbusConfig.ModbusDebugLevel>=3 )
//						debug_printf(DBG_HEADER_INFO "I/O modbus master - SetResponseSize, NbrCharsToReceive=%d\n",NbrCharsWaited);
					SerialSetResponseSize( NbrCharsWaited, ModbusConfig.ModbusTimeOutReceipt );
//TEMP TEST USB-RS485 converter...
//SerialSend( "\0\0", 2 );
					SerialSend( QuestionFrame, SizeQuestionToAsk );
				}

				if ( SendResultStatus==0 )
				{
					if ( ModbusConfig.ModbusTimeAfterTransmit>0 )
					{
						// usefull for USB-RS485 dongle...
						if( ModbusConfig.ModbusDebugLevel>=3 )
							printf(_("INFO CLASSICLADDER- after transmit delay now...%i milliseconds\n"),ModbusConfig.ModbusTimeAfterTransmit);
						DoPauseMilliSecs( ModbusConfig.ModbusTimeAfterTransmit );
					}
					
					if ( ModbusConfig.ModbusSerialPortNameUsed[ 0 ]=='\0' )
						ResponseSize = WaitRespSocketModbusMaster( ResponseFrame, 800, ModbusConfig.ModbusTimeOutReceipt );
					else
						ResponseSize = SerialReceive( ResponseFrame, 800 );
					StatsNbrFrames++;
					if ( ResponseSize==0 )
						printf(_("ERROR CLASSICLADDER-  MODBUS NO RESPONSE (Errs=%d/%d) !?\n"), ++StatsNbrErrorsModbusTreat, StatsNbrFrames);
					if ( !TreatModbusMasterResponse( (unsigned char *)ResponseFrame, ResponseSize ) )
					{
						StatsNbrErrorsModbusTreat++;
						// trouble? => cleanup all pending chars (perhaps we can receive now responses for old asks
						// and are shifted between ask/resp...)
						if ( ModbusConfig.ModbusSerialPortNameUsed[ 0 ]!='\0' )
							SerialPurge( );
					}
				}
				if ( ModbusConfig.ModbusTimeInterFrame>0 )
					DoPauseMilliSecs( ModbusConfig.ModbusTimeInterFrame );
			}
			else
			{
//				sleep( 1 );
				DoPauseMilliSecs( 1000 );
			}
		}
	
	}
#ifndef __WIN32__
	pthread_exit(NULL);
#endif

}

/*
void GetSocketModbusMasterStats( char * Buff )
{
	snprintf( Buff, sizeof(Buff), "Modbus I/O module master stats: FramesSend=%d - NoResponse=%d - BadResponses=%d", StatsNbrFrames, StatsNbrErrorsNoResponse, StatsNbrErrorsModbusTreat );
}
*/

