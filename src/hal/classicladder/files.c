/* Classic Ladder Project */
/* Copyright (C) 2001-2009 Marc Le Douarain */
/* http://membres.lycos.fr/mavati/classicladder/ */
/* http://www.sourceforge.net/projects/classicladder */
/* February 2001 */
/* -------------------------------------------------------------------------------------------- */
/* Load/Save Rungs, Timers, Monostables, Counters, Arithmetic expressions & Sections parameters */
/* -------------------------------------------------------------------------------------------- */
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

#include <locale.h>
#include <libintl.h>
#define _(x) gettext(x)
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <dirent.h>
#include <errno.h>
// for mkdir( ) Linux + unistd.h for rmdir()
#if !defined(__WIN32__)
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#else
#include <dir.h>
#endif
#include "classicladder.h"
#include "global.h"
#include "calc.h"
#include "calc_sequential.h"
#include "files_sequential.h"
#include "files.h"
#include "vars_access.h"
#include "manager.h"
//#include "log.h"

#include "emc_mods.h"
#include <rtapi_string.h>

#ifdef debug
#define dbg_printf printf
#else
static inline int dbg_printf(char *f, ...) { (void)f; return 0;}
#endif

StrDatasForBase CorresDatasForBase[3] = { {BASE_MINS , TIME_BASE_MINS , "%.1fmn" , "Mins" } ,
                                   {BASE_SECS , TIME_BASE_SECS , "%.1fs" , "Secs" } ,
                                   {BASE_100MS , TIME_BASE_100MS , "%.0f00ms" , "100msecs" } };
char * TimersModesStrings[ NBR_TIMERSMODES ] = { "TON", "TOF", "TP" };

char TmpDirectory[ 400 ] = "";


char *cl_fgets(char *s, int size, FILE *stream)
{
	char * res;
	s[0] = '\0';
	res = fgets( s, size, stream );

	// While last character in string is either CR or LF, remove.
	while (strlen(s)>=1 && ((s[strlen(s)-1]=='\r') || (s[strlen(s)-1]=='\n')))
		s[strlen(s)-1] = '\0';
#if S_LINE_INUSE
	if ( strlen( S_LINE )>0 && strlen(s)>strlen( S_LINE )+strlen( E_LINE ) )
	{
		strcpy( s, s+strlen( S_LINE ) );
		s[ strlen(s)-strlen(E_LINE) ] = '\0';
	}
#endif
	return res;
}

char ConvRawLineOfElements(char * RawLine,int y,StrRung * StorageRung)
{
	char * StartOfValue;
	char * EndOfValue;
	int x = 0;

	char EndOfLine;
	char IndexedVarFound;

	StartOfValue = RawLine;
	EndOfValue = RawLine;

	do
	{
		/* Extract Element Type */
		StartOfValue = EndOfValue;
		do
		{
			EndOfValue++;
		}
		while(*EndOfValue!='-');
		*EndOfValue++ = '\0';
		StorageRung->Element[x][y].Type = atoi(StartOfValue);

		/* Extract ConnectedWithTop */
		StartOfValue = EndOfValue;
		do
		{
			EndOfValue++;
		}
		while(*EndOfValue!='-');
		*EndOfValue++ = '\0';
		StorageRung->Element[x][y].ConnectedWithTop = atoi(StartOfValue);

		IndexedVarFound = FALSE;
		/* Extract Var Type */
		StartOfValue = EndOfValue;
		do
		{
			EndOfValue++;
		}
		while(*EndOfValue!='/');
		*EndOfValue++ = '\0';
		StorageRung->Element[x][y].VarType = atoi(StartOfValue);

		/* Extract Var Offset in the type table */
		StartOfValue = EndOfValue;
		do
		{
			EndOfValue++;
		}
		while( (*EndOfValue!=',') && (*EndOfValue!='[') && (*EndOfValue!=10) && (*EndOfValue!='\0') );
		EndOfLine = TRUE;
		if (*EndOfValue==',')
			EndOfLine = FALSE;
		if ( *EndOfValue=='[' )
			IndexedVarFound = TRUE;
		*EndOfValue++ = '\0';
		StorageRung->Element[x][y].VarNum = atoi(StartOfValue);
		if ( IndexedVarFound )
		{
			/* Extract Indexed Var Type */
			StartOfValue = EndOfValue;
			do
			{
				EndOfValue++;
			}
			while(*EndOfValue!='/');
			*EndOfValue++ = '\0';
			StorageRung->Element[x][y].IndexedVarType = atoi(StartOfValue);

			/* Extract Indexed Var Offset in the type table */
			StartOfValue = EndOfValue;
			do
			{
				EndOfValue++;
			}
			while( (*EndOfValue!=',') && (*EndOfValue!=10) && (*EndOfValue!='\0') );
			EndOfLine = TRUE;
			if (*EndOfValue==',')
				EndOfLine = FALSE;
			*EndOfValue++ = '\0';
			StorageRung->Element[x][y].IndexedVarNum = atoi(StartOfValue);
		}

		/* Next Element */
		x++;

	}
	while(!EndOfLine);
	return (x);
}

/*void RemoveEndLine( char * line )
{
	if (strlen( line )>0 && line[ strlen(line)-1 ]=='\n')
		line[ strlen(line)-1 ]='\0';
	if (strlen( line )>0 && line[ strlen(line)-1 ]=='\r')
		line[ strlen(line)-1 ]='\0';
	if (strlen( line )>0 && line[ strlen(line)-1 ]=='\r')
		line[ strlen(line)-1 ]='\0';
}*/

char LoadRung(char * FileName,StrRung * BufRung)
{
    FILE * File;
    char Okay = FALSE;
    char Line[300];
    char * LineOk;
    int y = 0;
    File = fopen(FileName,"rt");
    if (File)
    {
        do
        {
            LineOk = cl_fgets(Line,300,File);
            if (LineOk)
            {
                switch(Line[0])
                {
                    case ';':
                        break;
                    case '#':
                        if(strncmp(&Line[1],"VER=",4)==0)
                        {
                            if (atoi(&Line[5])>2)
                            {
                                printf(_("Rung version not supported...\n"));
                                LineOk = FALSE;
                            }
                        }
                        if(strncmp(&Line[1],"LABEL=",6)==0)
                        {
                            rtapi_strxcpy(BufRung->Label,&Line[7]);
//WIN32PORT
//							RemoveEndLine( BufRung->Label );
                        }
                        if(strncmp(&Line[1],"COMMENT=",8)==0)
                        {
                            rtapi_strxcpy(BufRung->Comment,&Line[9]);
//WIN32PORT
//							RemoveEndLine( BufRung->Comment );
                        }
                        if(strncmp(&Line[1],"PREVRUNG=",9)==0)
                            BufRung->PrevRung = atoi( &Line[10] );
                        if(strncmp(&Line[1],"NEXTRUNG=",9)==0)
                            BufRung->NextRung = atoi( &Line[10] );
                        break;
                    default:
					ConvRawLineOfElements(Line,y,BufRung);
                    y++;
                }
            }
        }
        while(LineOk);
        fclose(File);
        Okay = TRUE;
    }
    return (Okay);
}
char SaveRung(char * FileName,StrRung * BufRung)
{
    FILE * File;
    char Okay = FALSE;
    int x,y;
    File = fopen(FileName,"wt");
    if (File)
    {
//        fprintf(File,"; Rung :\n");
//        fprintf(File,"; all the blocks with the following format :\n");
//        fprintf(File,"; type (see classicladder.h) - ConnectedWithTop - VarType (see classicladder.h) / VarOffset\n");
        fprintf(File,S_LINE "#VER=2.0" E_LINE "\n");
        fprintf(File,S_LINE "#LABEL=%s" E_LINE "\n",BufRung->Label);
        fprintf(File,S_LINE "#COMMENT=%s" E_LINE "\n",BufRung->Comment);
        fprintf(File,S_LINE "#PREVRUNG=%d" E_LINE "\n",BufRung->PrevRung);
        fprintf(File,S_LINE "#NEXTRUNG=%d" E_LINE "\n",BufRung->NextRung);
        for (y=0;y<RUNG_HEIGHT;y++)
        {
            fprintf(File, "%s", S_LINE );
            for(x=0;x<RUNG_WIDTH;x++)
            {
                fprintf(File,"%d-%d-%d/%d",BufRung->Element[x][y].Type, BufRung->Element[x][y].ConnectedWithTop ,
                                    BufRung->Element[x][y].VarType , BufRung->Element[x][y].VarNum);
                if ( BufRung->Element[x][y].IndexedVarType!=-1 )
                    fprintf( File, "[%d/%d]", BufRung->Element[x][y].IndexedVarType, BufRung->Element[x][y].IndexedVarNum );
                if (x<RUNG_WIDTH-1)
                    fprintf(File," , ");
            }
            fprintf(File,E_LINE "\n");
        }
        fclose(File);
        Okay = TRUE;
    }
    return (Okay);
}

void LoadAllRungs_V1(char * BaseName,StrRung * Rungs,int * TheFirst,int * TheLast,int * TheCurrent)
{
    int NumRung;
    StrRung * PrevRung = NULL;
    int PrevNumRung = 0;
    *TheFirst = -1;
    *TheCurrent = -1;
    for(NumRung=0; NumRung<NBR_RUNGS; NumRung++)
    {
        char RungFile[400];
        snprintf(RungFile, sizeof(RungFile),"%s%d.csv",BaseName,NumRung);
        dbg_printf(_("Loading file : %s"),RungFile);
        if (LoadRung(RungFile,Rungs))
        {
            if (*TheFirst==-1)
            {
                *TheFirst = NumRung;
                *TheCurrent = NumRung;
            }
            if (PrevRung)
            {
                PrevRung->NextRung = NumRung;
            }
            Rungs->Used = TRUE;
            Rungs->PrevRung = PrevNumRung;
            *TheLast = NumRung;
            PrevNumRung = NumRung;
            PrevRung = Rungs;
            dbg_printf(_(" - ok.\n"));
        }
        else
            dbg_printf(_(" - not found.\n"));
        //DumpRung(Rungs);
        Rungs++;
    }
    /* no rungs loaded ? */
    /* we must keep at least one empty ! */
    if (*TheFirst<0)
    {
        *TheFirst = 0;
        *TheCurrent = 0;
    }
}
void LoadAllRungs(char * BaseName,StrRung * Rungs)
{
    int NumRung;
    for(NumRung=0; NumRung<NBR_RUNGS; NumRung++)
    {
        char RungFile[400];
        snprintf(RungFile, sizeof(RungFile),"%s%d.csv",BaseName,NumRung);
        dbg_printf(_("Loading file : %s"),RungFile);
        if (LoadRung(RungFile,Rungs))
        {
            Rungs->Used = TRUE;
            dbg_printf(_(" - ok.\n"));
        }
        else
            dbg_printf(_(" - not found.\n"));
        //DumpRung(Rungs);
        Rungs++;
    }
}

void SaveAllRungs(char * BaseName)
{
    int NumRung;
    char RungFile[400];
    /* delete all before */
    for(NumRung=0;NumRung<NBR_RUNGS;NumRung++)
    {
        snprintf(RungFile, sizeof(RungFile),"%s%d.csv",BaseName,NumRung);
        remove(RungFile);
    }
    /* save rungs (only defined ones are saved) */
    /* Since v0.5.5, the number of the rungs do not change : */
    /* it's easier, and with the sections added it's indispensable */
    for(NumRung=0;NumRung<NBR_RUNGS;NumRung++)
    {
        if ( RungArray[ NumRung ].Used )
        {
            snprintf(RungFile, sizeof(RungFile),"%s%d.csv",BaseName,NumRung);
            dbg_printf(_("Saving file : %s"),RungFile);
            if (SaveRung(RungFile,&RungArray[NumRung]))
                dbg_printf(_(" - ok.\n"));
            else
                dbg_printf(_(" - failed.\n"));
        }
    }
}

void DumpRung(StrRung * TheRung)
{
    int x,y;
    printf(_("Used=%d\n"),TheRung->Used);
    for (y=0;y<RUNG_HEIGHT;y++)
    {
        for(x=0;x<RUNG_WIDTH;x++)
        {
            printf("%d:%d:%d=%d , ",TheRung->Element[x][y].Type,TheRung->Element[x][y].ConnectedWithTop,TheRung->Element[x][y].VarNum,TheRung->Element[x][y].DynamicOutput);
        }
        printf("\n");
    }
}

char * ConvRawLineOfNumbers(char * RawLine,char NbrParams,int * ValuesFnd)
{
	char * StartOfValue;
	char * EndOfValue;
	char Num = 0;
	
	char EndOfLine;
	
	StartOfValue = RawLine;
	EndOfValue = RawLine;
	EndOfLine = FALSE;
	
	do
	{
		/* Extract Value */
		StartOfValue = EndOfValue;
		do
		{
		EndOfValue++;
		}
		while( (*EndOfValue!=',') && (*EndOfValue!=10) && (*EndOfValue!='\0') );
		if (*EndOfValue==10 || *EndOfValue=='\0')
			EndOfLine = TRUE;
		*EndOfValue++ = '\0';
		*ValuesFnd++ = atoi(StartOfValue);
		Num++;
		StartOfValue = EndOfValue;
	}
	while( (!EndOfLine) && (Num<NbrParams) );
		return EndOfValue;
}

int ConvBaseInMilliSecsToId(int NbrMilliSecs)
{
	switch(NbrMilliSecs)
	{
		case TIME_BASE_MINS:
		return BASE_MINS;
		case TIME_BASE_SECS:
		return BASE_SECS;
		case TIME_BASE_100MS:
		return BASE_100MS;
		default:
		printf(_("!!!Error in ConvBaseInMilliSecsToInt()\n"));
		return BASE_SECS;
	}
}

char * ConvRawLineOfStringsOrNumbers(char * RawLine,int * LgtParams,char ** ParamsStringsFnd,int ** ParamsIntFnd)
{
	char * StartOfValue;
	char * EndOfValue;
	int Num = 0;
	
	char EndOfLine;

	StartOfValue = RawLine;
	EndOfValue = RawLine;
	EndOfLine = FALSE;
	
	do
	{
		/* Extract Value */
		StartOfValue = EndOfValue;
		do
		{
			EndOfValue++;
		}
		while( (*EndOfValue!=',') && (*EndOfValue!=10) && (*EndOfValue!='\0') );
		if (*EndOfValue==10 || *EndOfValue=='\0')
			EndOfLine = TRUE;
		*EndOfValue++ = '\0';
		if ( ParamsStringsFnd[Num]!=NULL )
		{
			if ( strlen( StartOfValue )<(unsigned int)LgtParams[ Num ] )
				strcpy( ParamsStringsFnd[Num], StartOfValue );
		}
		else if ( ParamsIntFnd!=NULL )
		{
			if ( ParamsIntFnd[ Num ]!=NULL )
				*ParamsIntFnd[ Num ] = atoi( StartOfValue );
		}
		Num++;
		StartOfValue = EndOfValue;
	}
	while( (!EndOfLine) && (LgtParams[ Num ]>0) );
	return EndOfValue;
}

#ifdef OLD_TIMERS_MONOS_SUPPORT
char LoadTimersParams(char * FileName,StrTimer * BufTimers)
{
    FILE * File;
    char Okay = FALSE;
    char Line[300];
    char * LineOk;
    int Params[3];
    File = fopen(FileName,"rt");
    if (File)
    {
        do
        {
            LineOk = cl_fgets(Line,300,File);
            if (LineOk)
            {
                if (Line[0]!=';')
                {
                    ConvRawLineOfNumbers(Line,2,Params);
                    switch(Params[0])
                    {
                        case BASE_MINS:
                        case BASE_SECS:
                        case BASE_100MS:
                            BufTimers->Base = CorresDatasForBase[Params[0]].ValueInMS;
                            BufTimers->Preset = Params[1] * BufTimers->Base;
                            rtapi_strxcpy(BufTimers->DisplayFormat,CorresDatasForBase[Params[0]].DisplayFormat);
                            break;
                        default:
                            BufTimers->Base = 1;
                            BufTimers->Preset = 10;
                            rtapi_strxcpy(BufTimers->DisplayFormat,"%f?");
                            printf(_("!!! Error loading parameter base in %s\n"),FileName);
                            break;
                    }
dbg_printf(_("Timer => Base = %d , Preset = %d\n"),BufTimers->Base,BufTimers->Preset);
                    BufTimers++;
                }
            }
        }
        while(LineOk);
        fclose(File);
        Okay = TRUE;
    }
    return (Okay);
}

char SaveTimersParams(char * FileName,StrTimer * BufTimers)
{
    FILE * File;
    char Okay = FALSE;
    int NumTimer = 0;
    File = fopen(FileName,"wt");
    if (File)
    {
//        fprintf(File,"; Timers :\n");
//        fprintf(File,"; Base(see classicladder.h),Preset\n");
        do
        {
            fprintf(File,S_LINE "%d,%d" E_LINE  "\n",ConvBaseInMilliSecsToId(BufTimers->Base),BufTimers->Preset/BufTimers->Base);
            BufTimers++;
            NumTimer++;
        }
        while(NumTimer<NBR_TIMERS);
        fclose(File);
        Okay = TRUE;
    }
printf( _(" - result=%d\n"), Okay );
    return (Okay);
}

char LoadMonostablesParams(char * FileName,StrMonostable * BufMonostables)
{
    FILE * File;
    char Okay = FALSE;
    char Line[300];
    char * LineOk;
    int Params[3];
    File = fopen(FileName,"rt");
    if (File)
    {
        do
        {
            LineOk = cl_fgets(Line,300,File);
            if (LineOk)
            {
                if (Line[0]!=';')
                {
                    ConvRawLineOfNumbers(Line,2,Params);
                    switch(Params[0])
                    {
                        case BASE_MINS:
                        case BASE_SECS:
                        case BASE_100MS:
                            BufMonostables->Base = CorresDatasForBase[Params[0]].ValueInMS;
                            BufMonostables->Preset = Params[1] * BufMonostables->Base;
                            rtapi_strxcpy(BufMonostables->DisplayFormat,CorresDatasForBase[Params[0]].DisplayFormat);
                            break;
                        default:
                            BufMonostables->Base = 1;
                            BufMonostables->Preset = 10;
                            rtapi_strxcpy(BufMonostables->DisplayFormat,"%f?");
                            printf(_("!!! Error loading parameter base in %s\n"),FileName);
                            break;
                    }
dbg_printf(_("Monostable => Base = %d , Preset = %d\n"),BufMonostables->Base,BufMonostables->Preset);
                    BufMonostables++;
                }
            }
        }
        while(LineOk);
        fclose(File);
        Okay = TRUE;
    }
    return (Okay);
}

char SaveMonostablesParams(char * FileName,StrMonostable * BufMonostables)
{
    FILE * File;
    char Okay = FALSE;
    int NumMonostable = 0;
    File = fopen(FileName,"wt");
    if (File)
    {
//        fprintf(File,"; Monostables :\n");
//        fprintf(File,"; Base(see classicladder.h),Preset\n");
        do
        {
            fprintf(File,S_LINE "%d,%d" E_LINE  "\n",ConvBaseInMilliSecsToId(BufMonostables->Base),BufMonostables->Preset/BufMonostables->Base);
            BufMonostables++;
            NumMonostable++;
        }
        while(NumMonostable<NBR_MONOSTABLES);
        fclose(File);
        Okay = TRUE;
    }
    return (Okay);
}
#endif

char LoadCountersParams(char * FileName)
{
    FILE * File;
    char Okay = FALSE;
    char Line[300];
    char * LineOk;
    int Params[1];
    int ScanCounter = 0;
    File = fopen(FileName,"rt");
    if (File)
    {
        do
        {
            LineOk = cl_fgets(Line,300,File);
            if (LineOk)
            {
                if (Line[0]!=';')
                {
                    ConvRawLineOfNumbers(Line,1,Params);
					WriteVar( VAR_COUNTER_PRESET, ScanCounter, Params[ 0 ] );
					ScanCounter++;

                }
            }
        }
        while(LineOk);
        fclose(File);
        Okay = TRUE;
    }
    return (Okay);
}

char SaveCountersParams(char * FileName)
{
    FILE * File;
    char Okay = FALSE;
    int ScanCounter;
    File = fopen(FileName,"wt");
    if (File)
    {
//        fprintf(File,"; Counters :\n");
//        fprintf(File,"; Preset\n");
        for( ScanCounter=0; ScanCounter<NBR_COUNTERS; ScanCounter++ )
        {
            fprintf(File,S_LINE "%d" E_LINE "\n",CounterArray[ ScanCounter ].Preset);
        }
        fclose(File);
        Okay = TRUE;
    }
    return (Okay);
}

char LoadNewTimersParams(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	char Line[300];
	char * LineOk;
	int Params[3];
    int ScanTimerIEC = 0;
	File = fopen(FileName,"rt");
	if (File)
	{
		do
		{
			LineOk = cl_fgets(Line,300,File);
			if (LineOk)
			{
				if (Line[0]!=';')
				{
					StrTimerIEC * TimerIEC = &NewTimerArray[ ScanTimerIEC ];
					ConvRawLineOfNumbers(Line,3,Params);
					switch(Params[0])
					{
						case BASE_MINS:
						case BASE_SECS:
						case BASE_100MS:
							TimerIEC->Base = CorresDatasForBase[Params[0]].ValueInMS;
							WriteVar( VAR_TIMER_IEC_PRESET, ScanTimerIEC, Params[1] );
							rtapi_strxcpy(TimerIEC->DisplayFormat,CorresDatasForBase[Params[0]].DisplayFormat);
							break;
						default:
							TimerIEC->Base = 1;
							WriteVar( VAR_TIMER_IEC_PRESET, ScanTimerIEC, 10 );
							rtapi_strxcpy(TimerIEC->DisplayFormat,"%f?");
							printf(_("!!! Error loading parameter base in %s\n"),FileName);
							break;
					}
					TimerIEC->TimerMode = (char)Params[2];
					ScanTimerIEC++;
				}
			}
		}
		while(LineOk);
		fclose(File);
		Okay = TRUE;
	}
	return (Okay);
}

char SaveNewTimersParams(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	int NumTimerIEC = 0;
	File = fopen(FileName,"wt");
	if (File)
	{
//		fprintf(File,"; Timers IEC :\n");
//		fprintf(File,"; Base(see classicladder.h),Preset,TimerMode(see classicladder.h)\n");
		do
		{
			StrTimerIEC * TimerIEC = &NewTimerArray[ NumTimerIEC ];
			fprintf(File,S_LINE "%d,%d,%d" E_LINE "\n",ConvBaseInMilliSecsToId(TimerIEC->Base),TimerIEC->Preset,TimerIEC->TimerMode);
			NumTimerIEC++;
		}
		while(NumTimerIEC<NBR_TIMERS_IEC);
		fclose(File);
		Okay = TRUE;
	}
	return (Okay);
}

char LoadArithmeticExpr(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	char Line[300];
	char * LineOk;
	int NumExpr = 0;
	File = fopen(FileName,"rt");
	if (File)
	{
		do
		{
			LineOk = cl_fgets(Line,300,File);
			if (LineOk)
			{
				if (Line[0]!=';' && Line[0]!='#')
				{
//WIN32PORT
//					RemoveEndLine( Line );
					// new format with number at the start ?
					// (only not blank lines saved!)
					if ( Line[0]>='0' && Line[0]<='9' )
					{
						NumExpr = atoi(Line);
						rtapi_strxcpy(ArithmExpr[NumExpr].Expr,Line+strlen("xxxx,"));
					}
					else
					{
						rtapi_strxcpy(ArithmExpr[NumExpr].Expr,Line);
						NumExpr++;
					}
				}
			}
		}
		while(LineOk);
		fclose(File);
		Okay = TRUE;
	}
	return (Okay);
}

char SaveArithmeticExpr(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	int NumExpr;
	File = fopen(FileName,"wt");
	if (File)
	{
//		fprintf(File,"; Arithmetic expressions :\n");
//		fprintf(File,"; Compare or Operate ones\n");
		fprintf(File,S_LINE "#VER=2.0" E_LINE "\n");
		for(NumExpr=0; NumExpr<NBR_ARITHM_EXPR; NumExpr++)
		{
			if ( ArithmExpr[NumExpr].Expr[0]!='\0')
				fprintf(File,S_LINE "%04d,%s" E_LINE "\n",NumExpr,ArithmExpr[NumExpr].Expr);
		}
		fclose(File);
		Okay = TRUE;
	}
	return (Okay);
}

char LoadSectionsParams(char * FileName)
{
    FILE * File;
    char Okay = FALSE;
    char Line[300];
    char * LineOk;
    int NumSection;
    StrSection * pSection;
    int Params[10];
    File = fopen(FileName,"rt");
    if (File)
    {
        do
        {
            LineOk = cl_fgets(Line,300,File);
            if (LineOk)
            {
                switch(Line[0])
                {
                    case ';':
                        break;
                    case '#':
                        if(strncmp(&Line[1],"VER=",4)==0)
                        {
                            if (atoi(&Line[5])>1)
                            {
                                printf(_("Sections file version not supported...\n"));
                                LineOk = FALSE;
                            }
                        }
                        // #NAMExxx=....
                        if(strncmp(&Line[1],"NAME",4)==0)
                        {
                            Line[ 8 ] = '\0';
                            NumSection = atoi( &Line[5] );
                            rtapi_strxcpy(SectionArray[ NumSection ].Name, &Line[9]);
//WIN32PORT
//							RemoveEndLine( SectionArray[ NumSection ].Name );
                        }
                        break;
                    default:
                        ConvRawLineOfNumbers(Line,6,Params);
                        NumSection = Params[ 0 ];
                        pSection = &SectionArray[ NumSection ];
                        pSection->Used = TRUE;
                        pSection->Language = Params[ 1 ];
                        pSection->SubRoutineNumber = Params[ 2 ];
                        pSection->FirstRung = Params[ 3 ];
                        pSection->LastRung = Params[ 4 ];
                        pSection->SequentialPage = Params[ 5 ];
dbg_printf(_("Section %d => Name=%s, Language=%d, SRnbr=%d, FirstRung=%d, LastRung=%d, SequentialPage=%d\n"), NumSection,
pSection->Name, pSection->Language, pSection->SubRoutineNumber, pSection->FirstRung, pSection->LastRung, pSection->SequentialPage);
                        break;
                }
            }
        }
        while(LineOk);
        fclose(File);
        Okay = TRUE;
    }
    return (Okay);
}

char SaveSectionsParams(char * FileName)
{
    FILE * File;
    char Okay = FALSE;
    int NumSection;
    StrSection * pSection;
    File = fopen(FileName,"wt");
    if (File)
    {
//        fprintf(File,"; Sections\n");
        fprintf(File,S_LINE "#VER=1.0" E_LINE "\n");
        NumSection = 0;
        do
        {
            pSection = &SectionArray[ NumSection ];
            if ( pSection->Used )
                fprintf(File,S_LINE "#NAME%03d=%s" E_LINE "\n", NumSection, pSection->Name);
            NumSection++;
        }
        while(NumSection<NBR_SECTIONS);
        NumSection = 0;
        do
        {
            pSection = &SectionArray[ NumSection ];
            if ( pSection->Used )
                fprintf(File,S_LINE "%03d,%d,%d,%d,%d,%d" E_LINE "\n", NumSection, pSection->Language, pSection->SubRoutineNumber, pSection->FirstRung, pSection->LastRung, pSection->SequentialPage );
            NumSection++;
        }
        while(NumSection<NBR_SECTIONS);
        fclose(File);
        Okay = TRUE;
    }
    return (Okay);
}



char LoadIOConfParams(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	char Line[300];
	char * LineOk;
	StrIOConf * pConfInput = &InfosGene->InputsConf[ 0 ];
	StrIOConf * pConfOutput = &InfosGene->OutputsConf[ 0 ];
	File = fopen(FileName,"rt");
	if (File)
	{
		do
		{
			LineOk = cl_fgets(Line,300,File);
			if (LineOk)
			{
				if (Line[0]!=';' && Line[0]!='#')
				{
					/* input/output depending of the first character */
					if ( Line[0]=='0' )
						ConvRawLineOfNumbers(Line+2,6,(int*)pConfInput++);
					if ( Line[0]=='1' )
						ConvRawLineOfNumbers(Line+2,6,(int*)pConfOutput++);
				}
			}
		}
		while(LineOk);
		fclose(File);
		Okay = TRUE;
	}
	return (Okay);
}

char SaveIOConfParams(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	int NumConf;
	int NbrConf;
	int Pass;
	StrIOConf * pConf;
	File = fopen(FileName,"wt");
	if (File)
	{
//		fprintf(File,"; I/O Configuration\n");
		fprintf(File,S_LINE "#VER=1.0" E_LINE "\n");
		for( Pass=0; Pass<2; Pass++)
		{
			NbrConf = (Pass==0)?NBR_INPUTS_CONF:NBR_OUTPUTS_CONF;
			for( NumConf=0; NumConf<NbrConf; NumConf++ )
			{
				pConf = (Pass==0)?&InfosGene->InputsConf[ NumConf ]:&InfosGene->OutputsConf[ NumConf ];
				/* valid mapping ? */
				if ( pConf->FirstClassicLadderIO!=-1 )
				{
					fprintf(File,S_LINE "%d,%d,%d,%d,%d,%d,%d" E_LINE "\n", Pass, pConf->FirstClassicLadderIO,
						pConf->DeviceType, pConf->SubDevOrAdr, pConf->FirstChannel,
						pConf->NbrConsecutivesChannels, pConf->FlagInverted );
				}
			}
		}
		fclose(File);
		Okay = TRUE;
	}
	return (Okay);
}


#ifdef MODBUS_IO_MASTER
char LoadModbusIOConfParams(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	char Line[300];
	int IntDatas[ 6 ];
	char * LineOk;
	StrModbusMasterReq * pConf = &ModbusMasterReq[ 0 ];
	File = fopen(FileName,"rt");
	if (File)
	{
		do
		{
			LineOk = cl_fgets(Line,300,File);
			if (LineOk)
			{
				if (Line[0]!=';' && Line[0]!='#')
				{
					// if dotted IP address is used, special first field
					if ( strchr( Line, '.' ) )
					{
						char * EndFirstField = strchr( Line, ',' );
						// search end of the first field
						if ( EndFirstField )
						{
							ConvRawLineOfNumbers( EndFirstField+1, 5, &IntDatas[1] );
							*EndFirstField = '\0';
							rtapi_strxcpy( pConf->SlaveAdr, Line );
						}
						else
						{
							LineOk = FALSE;
						}
					}
					else
					{
						ConvRawLineOfNumbers( Line, 6, IntDatas );
						snprintf(pConf->SlaveAdr, sizeof(pConf->SlaveAdr), "%d", IntDatas[0] );
					}
					if ( LineOk )
					{
						pConf->TypeReq = (char)IntDatas[ 1 ];
						pConf->FirstModbusElement = IntDatas[ 2 ];
						pConf->NbrModbusElements = IntDatas[ 3 ];
						pConf->LogicInverted = (char)IntDatas[ 4 ];
						pConf->OffsetVarMapped = IntDatas[ 5 ];
						pConf++;
					}
				}
			}
		}
		while(LineOk);
		fclose(File);
		Okay = TRUE;
	}
	return (Okay);
}

char SaveModbusIOConfParams(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	int NumLine;
	StrModbusMasterReq * pConf;
	File = fopen(FileName,"wt");
	if (File)
	{
//		fprintf(File,"; Modbus Distributed I/O Configuration\n");
        fprintf(File,S_LINE "#VER=1.0" E_LINE "\n");
		for (NumLine=0; NumLine<NBR_MODBUS_MASTER_REQ; NumLine++ )
		{
			pConf = &ModbusMasterReq[ NumLine ];
			/* valid request ? */
			if ( pConf->SlaveAdr[0]!='\0' )
			{
				fprintf(File,S_LINE "%s,%d,%d,%d,%d,%d" E_LINE "\n", pConf->SlaveAdr, pConf->TypeReq,
					pConf->FirstModbusElement, pConf->NbrModbusElements, pConf->LogicInverted, pConf->OffsetVarMapped );
			}
		}
		fclose(File);
		Okay = TRUE;
	}
	return (Okay);
}
#endif

char LoadSymbols(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	char Line[300];
	char * LineOk;
	int NumSymbol = 0;
	char *PtrStrings[ 4 ];
	int LgtMaxStrings[ 4 ];
	StrSymbol * pSymbol;
	File = fopen(FileName,"rt");
	if (File)
	{
		do
		{
			LineOk = cl_fgets(Line,300,File);
			if (LineOk)
			{
				switch(Line[0])
				{
					case ';':
						break;
					case '#':
						if(strncmp(&Line[1],"VER=",4)==0)
						{
							if (atoi(&Line[5])>1)
							{
								printf(_("Symbols file version not supported...\n"));
								LineOk = FALSE;
							}
						}
						break;
					default:
						pSymbol = &SymbolArray[ NumSymbol ];
						PtrStrings[ 0 ] = pSymbol->VarName; LgtMaxStrings[ 0 ] = LGT_VAR_NAME;
						PtrStrings[ 1 ] = pSymbol->Symbol; LgtMaxStrings[ 1 ] = LGT_SYMBOL_STRING;
						PtrStrings[ 2 ] = pSymbol->Comment; LgtMaxStrings[ 2 ] = LGT_SYMBOL_COMMENT;
						PtrStrings[ 3 ] = NULL; LgtMaxStrings[ 3 ] = 0;
						ConvRawLineOfStringsOrNumbers( Line, LgtMaxStrings, PtrStrings, NULL );
//						RemoveEndLine( pSymbol->Comment );
						dbg_printf(_("Symbol: %s - %s - %s\n"), pSymbol->VarName, pSymbol->Symbol, pSymbol->Comment);
						NumSymbol++;
						break;
				}
			}
		}
		while(LineOk);
		fclose(File);
		Okay = TRUE;
	}
SymbolsAutoAssign();	
return (Okay);
}

char SaveSymbols(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	int NumSymbol = 0;
	StrSymbol * pSymbol;
	File = fopen(FileName,"wt");
	if (File)
	{
//		fprintf(File,"; Symbols\n");
		fprintf(File,S_LINE "#VER=1.0" E_LINE "\n");
		do
		{
			pSymbol = &SymbolArray[ NumSymbol ];
			if ( pSymbol->VarName[0]!='\0')
				fprintf(File,S_LINE "%s,%s,%s" E_LINE "\n", pSymbol->VarName, pSymbol->Symbol, pSymbol->Comment );
			NumSymbol++;
		}
		while(NumSymbol<NBR_SYMBOLS);
		fclose(File);
		Okay = TRUE;
	}
	return (Okay);
}

// This function is not used because parameters are loaded with real time module in EMC
char LoadGeneralParameters(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	char Line[300];
	char * LineOk;
	File = fopen(FileName,"rt");
	if (File)
	{
		do
		{
			LineOk = cl_fgets(Line,300,File);
			if (LineOk)
			{
				char * pParameter;
				pParameter = "PERIODIC_REFRESH=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					GeneralParamsMirror.PeriodicRefreshMilliSecs = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "SIZE_NBR_RUNGS=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					GeneralParamsMirror.SizesInfos.nbr_rungs = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "SIZE_NBR_BITS=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					GeneralParamsMirror.SizesInfos.nbr_bits = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "SIZE_NBR_WORDS=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					GeneralParamsMirror.SizesInfos.nbr_words = atoi( &Line[ strlen( pParameter) ] );
#ifdef OLD_TIMERS_MONOS_SUPPORT
				pParameter = "SIZE_NBR_TIMERS=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					GeneralParamsMirror.SizesInfos.nbr_timers = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "SIZE_NBR_MONOSTABLES=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					GeneralParamsMirror.SizesInfos.nbr_monostables = atoi( &Line[ strlen( pParameter) ] );
#endif
				pParameter = "SIZE_NBR_COUNTERS=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					GeneralParamsMirror.SizesInfos.nbr_counters = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "SIZE_NBR_TIMERS_IEC=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					GeneralParamsMirror.SizesInfos.nbr_timers_iec = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "SIZE_NBR_PHYS_INPUTS=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					GeneralParamsMirror.SizesInfos.nbr_phys_inputs = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "SIZE_NBR_PHYS_OUTPUTS=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					GeneralParamsMirror.SizesInfos.nbr_phys_outputs = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "SIZE_NBR_ARITHM_EXPR=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					GeneralParamsMirror.SizesInfos.nbr_arithm_expr = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "SIZE_NBR_SECTIONS=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					GeneralParamsMirror.SizesInfos.nbr_sections = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "SIZE_NBR_SYMBOLS=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					GeneralParamsMirror.SizesInfos.nbr_symbols = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "SIZE_NBR_PHYS_WORDS_INPUTS=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					GeneralParamsMirror.SizesInfos.nbr_phys_words_inputs = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "SIZE_NBR_PHYS_WORDS_OUTPUTS=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					GeneralParamsMirror.SizesInfos.nbr_phys_words_outputs = atoi( &Line[ strlen( pParameter) ] );
			}
		}
		while(LineOk);
		fclose(File);
		Okay = TRUE;
	}
	return (Okay);
}

char SaveGeneralParameters(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	File = fopen(FileName,"wt");
	if (File)
	{
		fprintf( File,S_LINE "PERIODIC_REFRESH=%d" E_LINE "\n", GeneralParamsMirror.PeriodicRefreshMilliSecs );
		fprintf( File,S_LINE "SIZE_NBR_RUNGS=%d" E_LINE "\n", GeneralParamsMirror.SizesInfos.nbr_rungs );
		fprintf( File,S_LINE "SIZE_NBR_BITS=%d" E_LINE "\n", GeneralParamsMirror.SizesInfos.nbr_bits );
		fprintf( File,S_LINE "SIZE_NBR_WORDS=%d" E_LINE "\n", GeneralParamsMirror.SizesInfos.nbr_words );
#ifdef OLD_TIMERS_MONOS_SUPPORT
		fprintf( File,S_LINE "SIZE_NBR_TIMERS=%d" E_LINE "\n", GeneralParamsMirror.SizesInfos.nbr_timers );
		fprintf( File,S_LINE "SIZE_NBR_MONOSTABLES=%d" E_LINE "\n", GeneralParamsMirror.SizesInfos.nbr_monostables );
#endif
		fprintf( File,S_LINE "SIZE_NBR_COUNTERS=%d" E_LINE "\n", GeneralParamsMirror.SizesInfos.nbr_counters );
		fprintf( File,S_LINE "SIZE_NBR_TIMERS_IEC=%d" E_LINE "\n", GeneralParamsMirror.SizesInfos.nbr_timers_iec );
		fprintf( File,S_LINE "SIZE_NBR_PHYS_INPUTS=%d" E_LINE "\n", GeneralParamsMirror.SizesInfos.nbr_phys_inputs );
		fprintf( File,S_LINE "SIZE_NBR_PHYS_OUTPUTS=%d" E_LINE "\n", GeneralParamsMirror.SizesInfos.nbr_phys_outputs );
		fprintf( File,S_LINE "SIZE_NBR_ARITHM_EXPR=%d" E_LINE "\n", GeneralParamsMirror.SizesInfos.nbr_arithm_expr );
		fprintf( File,S_LINE "SIZE_NBR_SECTIONS=%d" E_LINE "\n", GeneralParamsMirror.SizesInfos.nbr_sections );
		fprintf( File,S_LINE "SIZE_NBR_SYMBOLS=%d" E_LINE "\n", GeneralParamsMirror.SizesInfos.nbr_symbols );
		fprintf( File,S_LINE "SIZE_NBR_PHYS_WORDS_INPUTS=%d" E_LINE "\n", GeneralParamsMirror.SizesInfos.nbr_phys_words_inputs );
		fprintf( File,S_LINE "SIZE_NBR_PHYS_WORDS_OUTPUTS=%d" E_LINE "\n", GeneralParamsMirror.SizesInfos.nbr_phys_words_outputs );
		fclose(File);
		Okay = TRUE;
	}
	return (Okay);
}

char LoadComParameters(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	char Line[300];
	char * LineOk;
	File = fopen(FileName,"rt");
	if (File)
	{
		do
		{
			LineOk = cl_fgets(Line,300,File);
			if (LineOk)
			{
				char * pParameter;
				pParameter = "MODBUS_MASTER_SERIAL_PORT=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					 rtapi_strxcpy(ModbusConfig.ModbusSerialPortNameUsed,&Line[strlen( pParameter) ] );
                                pParameter = "MODBUS_MASTER_SERIAL_SPEED=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.ModbusSerialSpeed = atoi( &Line[ strlen( pParameter) ] );
                                pParameter = "MODBUS_MASTER_SERIAL_DATABITS=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.ModbusSerialDataBits = atoi( &Line[ strlen( pParameter) ] );
                                pParameter = "MODBUS_MASTER_SERIAL_STOPBITS=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.ModbusSerialStopBits = atoi( &Line[ strlen( pParameter) ] );
                                pParameter = "MODBUS_MASTER_SERIAL_PARITY=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.ModbusSerialParity = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "MODBUS_ELEMENT_OFFSET=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.ModbusEleOffset = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "MODBUS_MASTER_SERIAL_USE_RTS_TO_SEND=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.ModbusSerialUseRtsToSend = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "MODBUS_MASTER_TIME_INTER_FRAME=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.ModbusTimeInterFrame = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "MODBUS_MASTER_TIME_OUT_RECEIPT=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.ModbusTimeOutReceipt = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "MODBUS_MASTER_TIME_AFTER_TRANSMIT=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.ModbusTimeAfterTransmit = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "MODBUS_DEBUG_LEVEL=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.ModbusDebugLevel = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "MODBUS_MAP_COIL_READ=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.MapTypeForReadCoils = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "MODBUS_MAP_COIL_WRITE=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.MapTypeForWriteCoils = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "MODBUS_MAP_INPUT=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.MapTypeForReadInputs = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "MODBUS_MAP_HOLDING=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.MapTypeForReadInputRegs = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "MODBUS_MAP_REGISTER_READ=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.MapTypeForReadHoldRegs = atoi( &Line[ strlen( pParameter) ] );
				pParameter = "MODBUS_MAP_REGISTER_WRITE=";
				if ( strncmp( Line, pParameter, strlen( pParameter) )==0 )
					ModbusConfig.MapTypeForWriteHoldRegs = atoi( &Line[ strlen( pParameter) ] );
			}
		}
		while(LineOk);
		fclose(File);
		Okay = TRUE;
	}
	return (Okay);
}

char SaveComParameters(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	File = fopen(FileName,"wt");
	if (File)
	{
		fprintf( File,S_LINE "MODBUS_MASTER_SERIAL_PORT=%s" E_LINE "\n",ModbusConfig.ModbusSerialPortNameUsed  );
                fprintf( File,S_LINE "MODBUS_MASTER_SERIAL_SPEED=%d" E_LINE "\n",ModbusConfig.ModbusSerialSpeed  );
                fprintf( File,S_LINE "MODBUS_MASTER_SERIAL_DATABITS=%d" E_LINE "\n",ModbusConfig.ModbusSerialDataBits );
                fprintf( File,S_LINE "MODBUS_MASTER_SERIAL_STOPBITS=%d" E_LINE "\n",ModbusConfig.ModbusSerialStopBits  );
                fprintf( File,S_LINE "MODBUS_MASTER_SERIAL_PARITY=%d" E_LINE "\n",ModbusConfig.ModbusSerialParity  );
                fprintf( File,S_LINE "MODBUS_ELEMENT_OFFSET=%d" E_LINE "\n", ModbusConfig.ModbusEleOffset );
		fprintf( File,S_LINE "MODBUS_MASTER_SERIAL_USE_RTS_TO_SEND=%d" E_LINE "\n", ModbusConfig.ModbusSerialUseRtsToSend );
		fprintf( File,S_LINE "MODBUS_MASTER_TIME_INTER_FRAME=%d" E_LINE "\n", ModbusConfig.ModbusTimeInterFrame );
		fprintf( File,S_LINE "MODBUS_MASTER_TIME_OUT_RECEIPT=%d" E_LINE "\n", ModbusConfig.ModbusTimeOutReceipt );
		fprintf( File,S_LINE "MODBUS_MASTER_TIME_AFTER_TRANSMIT=%d" E_LINE "\n", ModbusConfig.ModbusTimeAfterTransmit );
		fprintf( File,S_LINE "MODBUS_DEBUG_LEVEL=%d" E_LINE "\n", ModbusConfig.ModbusDebugLevel );
		fprintf( File,S_LINE "MODBUS_MAP_COIL_READ=%d" E_LINE "\n", ModbusConfig.MapTypeForReadCoils );
		fprintf( File,S_LINE "MODBUS_MAP_COIL_WRITE=%d" E_LINE "\n", ModbusConfig.MapTypeForWriteCoils );
		fprintf( File,S_LINE "MODBUS_MAP_INPUT=%d" E_LINE "\n", ModbusConfig.MapTypeForReadInputs );
		fprintf( File,S_LINE "MODBUS_MAP_HOLDING=%d" E_LINE "\n", ModbusConfig.MapTypeForReadInputRegs );
		fprintf( File,S_LINE "MODBUS_MAP_REGISTER_READ=%d" E_LINE "\n", ModbusConfig.MapTypeForReadHoldRegs );
		fprintf( File,S_LINE "MODBUS_MAP_REGISTER_WRITE=%d" E_LINE "\n", ModbusConfig.MapTypeForWriteHoldRegs );
		fclose(File);
		Okay = TRUE;
	}
	return (Okay);
}

//XXX log functionality is not implemented.
/*
char LoadConfigEventsLog(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	char Line[300];
	char * LineOk;
	int NumConfigEvtLog = 0;
	char *PtrStrings[ 7 ];
	int LgtMaxStrings[ 7 ];
	int *PtrInts[ 7 ];
	StrConfigEventLog * pCfgEvtLog;
	File = fopen(FileName,"rt");
	if (File)
	{
		do
		{
			LineOk = cl_fgets(Line,300,File);
			if (LineOk)
			{
				switch(Line[0])
				{
					case ';':
						break;
					case '#':
						if(strncmp(&Line[1],"VER=",4)==0)
						{
							if (atoi(&Line[5])>1)
							{
								printf("Config Events file version not supported...\n");
								LineOk = FALSE;
							}
						}
						break;
					default:
						NumConfigEvtLog = atoi( Line );
						pCfgEvtLog = &ConfigEventLog[ NumConfigEvtLog ];
						PtrStrings[ 0 ] = NULL; LgtMaxStrings[ 0 ] = 1; PtrInts[ 0 ] = NULL; //first field already read in previous atoi()
						PtrStrings[ 1 ] = NULL; LgtMaxStrings[ 1 ] = 1; PtrInts[ 1 ] = &pCfgEvtLog->FirstVarNum;
						PtrStrings[ 2 ] = NULL; LgtMaxStrings[ 2 ] = 1; PtrInts[ 2 ] = &pCfgEvtLog->NbrVars;
						PtrStrings[ 3 ] = NULL; LgtMaxStrings[ 3 ] = 1; PtrInts[ 3 ] = &pCfgEvtLog->EventType;
						PtrStrings[ 4 ] = pCfgEvtLog->Symbol; LgtMaxStrings[ 4 ] = EVENT_SYMBOL_LGT; PtrInts[ 4 ] = NULL;
						PtrStrings[ 5 ] = pCfgEvtLog->Text; LgtMaxStrings[ 5 ] = EVENT_TEXT_LGT; PtrInts[ 5 ] = NULL;
						PtrStrings[ 6 ] = NULL; LgtMaxStrings[ 6 ] = 0; PtrInts[ 6 ] = NULL;
						ConvRawLineOfStringsOrNumbers( Line, LgtMaxStrings, PtrStrings, PtrInts );
dbg_printf("CfgEvt: %d: %d - %d - %s - %s\n", NumConfigEvtLog, pCfgEvtLog->FirstVarNum, pCfgEvtLog->NbrVars, pCfgEvtLog->Symbol, pCfgEvtLog->Text);
						break;
				}
			}
		}
		while(LineOk);
		fclose(File);
		Okay = TRUE;
	}
	return (Okay);
}

char SaveConfigEventsLog(char * FileName)
{
	FILE * File;
	char Okay = FALSE;
	int Scan = 0;
	StrConfigEventLog * pCfgEvtLog;
	File = fopen(FileName,"wt");
	if (File)
	{
		fprintf(File,S_LINE "#VER=1.0" E_LINE "\n");
		do
		{
			pCfgEvtLog = &ConfigEventLog[ Scan ];
			if ( pCfgEvtLog->FirstVarNum!=-1 && pCfgEvtLog->NbrVars!=0 )
				fprintf(File,S_LINE "%d,%d,%d,%d,%s,%s" E_LINE "\n", Scan, pCfgEvtLog->FirstVarNum, pCfgEvtLog->NbrVars, pCfgEvtLog->EventType, pCfgEvtLog->Symbol, pCfgEvtLog->Text );
			Scan++;
		}
		while(Scan<NBR_CONFIG_EVENTS_LOG);
		fclose(File);
		Okay = TRUE;
	}
	return (Okay);
}
*/

void DeleteTheDefaultSection( )
{
	RungArray[ 0 ].Used = FALSE;
	SectionArray[ 0 ].Used = FALSE;
}

static char FileName[500];
void LoadAllLadderDatas(char * DatasDirectory)
{
	ClassicLadder_InitAllDatas( );
	// not necessary to have the default section, as we will load a working project
	// and annoying if the section (with internal number 0) has been deleted in this project !
	DeleteTheDefaultSection( );

	//printf("Loading data from %s...\n", DatasDirectory);
// this function call is not wanted because in EMC parameters are loaded with the realtime module
//	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"general.txt",DatasDirectory);
//	LoadGeneralParameters( FileName );
        snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"com_params.txt",DatasDirectory);
//      printf("Loading modbus com data \n");
	LoadComParameters( FileName );
#ifdef OLD_TIMERS_MONOS_SUPPORT
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"timers.csv",DatasDirectory);
//	printf("Loading timers data from %s\n",FileName);
	LoadTimersParams(FileName,TimerArray);
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"monostables.csv",DatasDirectory);
//	printf("Loading monostables data from %s\n",FileName);
	LoadMonostablesParams(FileName,MonostableArray);
#endif
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"counters.csv",DatasDirectory);
//	printf("Loading counters data from %s\n",FileName);
	LoadCountersParams(FileName);
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"timers_iec.csv",DatasDirectory);
	LoadNewTimersParams(FileName);

	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"arithmetic_expressions.csv",DatasDirectory);
//	printf("Loading arithmetic expressions from %s\n",FileName);
	LoadArithmeticExpr(FileName);

	// Sections added since v0.5.5, the format of files has a little changed :
	// before the prev/next rungs were not saved in each rung...
	// and the number of rungs changed when saved...
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"sections.csv",DatasDirectory);
//	printf("Loading sections data from %s\n",FileName);
	if ( LoadSectionsParams(FileName) )
	{
		snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"rung_",DatasDirectory);
		LoadAllRungs(FileName,RungArray);
	}
	else
	{
		//printf("Rungs with old format found (no sections)\n");
		snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"rung_",DatasDirectory);
		LoadAllRungs_V1(FileName,RungArray,&InfosGene->FirstRung,&InfosGene->LastRung,&InfosGene->CurrentRung);
		// if we load old format files, sections wasn't created, so we must write this info...
		SectionArray[ 0 ].FirstRung = InfosGene->FirstRung;
		SectionArray[ 0 ].LastRung = InfosGene->LastRung;
	}
#ifdef SEQUENTIAL_SUPPORT
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"sequential.csv",DatasDirectory);
//	printf("Loading sequential data from %s\n",FileName);
	LoadSequential( FileName );
#endif
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"ioconf.csv",DatasDirectory);
//	printf("Loading I/O configuration data from %s\n",FileName);
	LoadIOConfParams( FileName );
#ifdef MODBUS_IO_MASTER
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"modbusioconf.csv",DatasDirectory);
//	printf("Loading modbus distributed I/O configuration data from %s\n",FileName);
	if (modmaster) {    PrepareModbusMaster( );    }
	LoadModbusIOConfParams( FileName );
#endif
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"symbols.csv",DatasDirectory);
//	printf("Loading symbols data from %s\n",FileName);
	LoadSymbols(FileName);
	//sprintf(FileName,"%s/"FILE_PREFIX"config_events.csv",DatasDirectory);
	//LoadConfigEventsLog(FileName);

	// security if empty file...
#ifdef GTK_INTERFACE
	if ( NbrSectionsDefined()==0 )
		AddSection("Prog1", SECTION_IN_LADDER, -1 );
#endif	

//printf("Prepare all data before run...\n");
	PrepareAllDatasBeforeRun( );
	// update the tags list of the variables that the user want to log (after to have load the config file...)
	//InitVarsArrayLogTags( );
}

void SaveAllLadderDatas(char * DatasDirectory)
{
	CleanTmpLadderDirectory( FALSE/*DestroyDir*/ );
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"general.txt",DatasDirectory);
	SaveGeneralParameters( FileName );
        snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"com_params.txt",DatasDirectory);
	SaveComParameters( FileName );
#ifdef OLD_TIMERS_MONOS_SUPPORT
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"timers.csv",DatasDirectory);
	SaveTimersParams(FileName,TimerArray);
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"monostables.csv",DatasDirectory);
	SaveMonostablesParams(FileName,MonostableArray);
#endif
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"counters.csv",DatasDirectory);
	SaveCountersParams(FileName);
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"timers_iec.csv",DatasDirectory);
	SaveNewTimersParams(FileName);
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"arithmetic_expressions.csv",DatasDirectory);
	SaveArithmeticExpr(FileName);
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"rung_",DatasDirectory);
	SaveAllRungs(FileName);
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"sections.csv",DatasDirectory);
	SaveSectionsParams( FileName );
#ifdef SEQUENTIAL_SUPPORT
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"sequential.csv",DatasDirectory);
	SaveSequential( FileName );
#endif
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"ioconf.csv",DatasDirectory);
	SaveIOConfParams( FileName );
#ifdef MODBUS_IO_MASTER
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"modbusioconf.csv",DatasDirectory);
	SaveModbusIOConfParams( FileName );
#endif
	snprintf(FileName, sizeof(FileName),"%s/"FILE_PREFIX"symbols.csv",DatasDirectory);
	SaveSymbols( FileName );
	//sprintf(FileName,"%s/"FILE_PREFIX"config_events.csv",DatasDirectory);
	//SaveConfigEventsLog( FileName );
	InfosGene->AskConfirmationToQuit = FALSE;
}


/* clean the tmp directory of the parameters files */
void CleanTmpLadderDirectory( char DestroyDir )
{
	DIR *pDir;
	struct dirent *pEnt;
	char Buff[400];

	if (TmpDirectory[0]!='\0')
	{
		pDir = opendir(TmpDirectory);
		if (pDir)
		{
			while ((pEnt = readdir(pDir)) != NULL)
			{
				if ( strcmp(pEnt->d_name,".") && strcmp(pEnt->d_name,"..") )
				{
					char cRemoveIt = TRUE;
					// if a file prefix defined, only remove the classicladder files...
					if ( strlen(FILE_PREFIX)>0 )
					{
						if ( strncmp( pEnt->d_name, FILE_PREFIX, strlen(FILE_PREFIX) )!=0 )
							cRemoveIt = FALSE;
					}
					if ( cRemoveIt )
					{
						size_t ret = snprintf(Buff, sizeof(Buff), "%s/%s", TmpDirectory,pEnt->d_name);
						if (ret <= sizeof(Buff)){
						    remove(Buff);
						}
					}
				}
			}
		}
		closedir(pDir);
		/* delete the temp directory if wanted */
#ifndef __WIN32__
		if ( DestroyDir )
			rmdir(TmpDirectory);
#else
		if ( DestroyDir )
			_rmdir(TmpDirectory);
#endif
	}
}
