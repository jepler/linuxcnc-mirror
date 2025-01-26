
/* Classic Ladder Project */
/* Copyright (C) 2001-2009 Marc Le Douarain */
/* http://membres.lycos.fr/mavati/classicladder/ */
/* http://www.sourceforge.net/projects/classicladder */
/* April 2009 */
/* ----------------------------------------- */
/* Copy part of a rung (selection then copy) */
/* ----------------------------------------- */
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
/* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA */

#include <stdio.h>
#include <string.h>

#include "classicladder.h"
#include "global.h"
#include "edit.h"
#include "edit_copy.h"

#include <rtapi_string.h>

int NumRungForSelection;
int PartSelectionPosiX;
int PartSelectionPosiY;
int PartSelectionSizeX;
int PartSelectionSizeY;
char UnderSelection = FALSE;
char LastColumnOfSelectionIsOutput = FALSE;

void StartOrMotionPartSelection(double x,double y, char StartToClick)
{
	int CurrentX,CurrentY;
	if ( ConvertDoublesToRungCoor( x, y, &CurrentX, &CurrentY ) )
	{
		if ( StartToClick )
		{
			EditDatas.GhostZonePosiX = CurrentX;
			EditDatas.GhostZonePosiY = CurrentY;
			EditDatas.GhostZoneSizeX = 1;
			EditDatas.GhostZoneSizeY = 1;
printf("start selection %d,%d (%d,%d)\n", EditDatas.GhostZonePosiX, EditDatas.GhostZonePosiY, EditDatas.GhostZoneSizeX, EditDatas.GhostZoneSizeY );
			UnderSelection = TRUE;
		}
		else
		{
			if ( UnderSelection )
			{
				if ( CurrentX>=EditDatas.GhostZonePosiX )
					EditDatas.GhostZoneSizeX = CurrentX-EditDatas.GhostZonePosiX+1;
				if ( CurrentY>=EditDatas.GhostZonePosiY )
					EditDatas.GhostZoneSizeY = CurrentY-EditDatas.GhostZonePosiY+1;
//printf("modified size selection %d,%d (%d,%d)\n", EditDatas.GhostZonePosiX, EditDatas.GhostZonePosiY, EditDatas.GhostZoneSizeX, EditDatas.GhostZoneSizeY );
			}
		}
	}
}

/* called when mouse button released (end of the rung part selection) */
void EndPartSelection( )
{
	UnderSelection = FALSE;
	NumRungForSelection = InfosGene->CurrentRung;
	PartSelectionPosiX = EditDatas.GhostZonePosiX;
	PartSelectionPosiY = EditDatas.GhostZonePosiY;
	PartSelectionSizeX = EditDatas.GhostZoneSizeX;
	PartSelectionSizeY = EditDatas.GhostZoneSizeY;
	LastColumnOfSelectionIsOutput = FALSE;
	if ( PartSelectionPosiX+PartSelectionSizeX-1==RUNG_WIDTH-1 )
		LastColumnOfSelectionIsOutput =  TRUE;
printf("store selection %d,%d (%d,%d) LastColumnOutput=%d\n", PartSelectionPosiX, PartSelectionPosiY, PartSelectionSizeX, PartSelectionSizeY, LastColumnOfSelectionIsOutput );
}

void GetSizesOfTheSelectionToCopy( int * pSizeX, int * pSizeY )
{
	*pSizeX = PartSelectionSizeX;
	*pSizeY = PartSelectionSizeY;
//printf("get size selection : (%d,%d)\n", PartSelectionSizeX, PartSelectionSizeY );
}
char GetIsOutputEleLastColumnSelection( )
{
	return LastColumnOfSelectionIsOutput;
}

void CopyFunctionBlockPropertiesOrExpr( int NumTypeEle, int NumBlockSrc, int NumBlockDest )
{
	switch( NumTypeEle )
	{
#ifdef OLD_TIMERS_MONOS_SUPPORT
		case ELE_TIMER:
			TimerArray[NumBlockDest].Base = TimerArray[NumBlockSrc].Base;
			TimerArray[NumBlockDest].Preset = TimerArray[NumBlockSrc].Preset;
			break;
		case ELE_MONOSTABLE:
			MonostableArray[NumBlockDest].Base = MonostableArray[NumBlockSrc].Base;
			MonostableArray[NumBlockDest].Preset = MonostableArray[NumBlockSrc].Preset;
			break;
#endif
		case ELE_COUNTER:
			CounterArray[NumBlockDest].Preset = CounterArray[NumBlockSrc].Preset;
			break;
		case ELE_TIMER_IEC:
			NewTimerArray[NumBlockDest].Base = NewTimerArray[NumBlockSrc].Base;
			NewTimerArray[NumBlockDest].Preset = NewTimerArray[NumBlockSrc].Preset;
			NewTimerArray[NumBlockDest].TimerMode = NewTimerArray[NumBlockSrc].TimerMode;
			break;
		case ELE_COMPAR:
		case ELE_OUTPUT_OPERATE:
			rtapi_strxcpy( EditArithmExpr[NumBlockDest].Expr, EditArithmExpr[NumBlockSrc].Expr );
			break;
	}	
}

void CopyNowPartSelected( double x,double y )
{
	(void)x;
	(void)y;
	if ( EditDatas.GhostZonePosiX!=-1 && EditDatas.GhostZonePosiY!=-1 )
	{
		int x,srcx,srcy;
		int y = EditDatas.GhostZonePosiY;
		StrRung * RungSrc = &RungArray[ NumRungForSelection ];
		// if copying from current, perhaps already modified !
		if ( NumRungForSelection==InfosGene->CurrentRung )
			RungSrc = &EditDatas.Rung;
printf("copying...\n");
		for (srcy=PartSelectionPosiY;srcy<PartSelectionPosiY+PartSelectionSizeY;srcy++)
		{
			x = EditDatas.GhostZonePosiX;
			for (srcx=PartSelectionPosiX;srcx<PartSelectionPosiX+PartSelectionSizeX;srcx++)
			{
				int TypeSrc = RungSrc->Element[srcx][srcy].Type;
				EditDatas.Rung.Element[x][y].VarNum = -1; //to detect if it has been modified (alloc of a complex...)
				//TODO: if alloc failed for a complex one, the corresponding "unusable" blocks should not be copied...
				if ( PrepBeforeSettingTypeEleForComplexBlocsAndExpr( TypeSrc, x, y ) )
				{
					EditDatas.Rung.Element[x][y].Type = TypeSrc;
					EditDatas.Rung.Element[x][y].ConnectedWithTop = RungSrc->Element[srcx][srcy].ConnectedWithTop;
					EditDatas.Rung.Element[x][y].VarType = RungSrc->Element[srcx][srcy].VarType;
					if ( EditDatas.Rung.Element[x][y].VarNum==-1 )
						EditDatas.Rung.Element[x][y].VarNum = RungSrc->Element[srcx][srcy].VarNum;
					else
						CopyFunctionBlockPropertiesOrExpr( TypeSrc, RungSrc->Element[srcx][srcy].VarNum, EditDatas.Rung.Element[x][y].VarNum );
					EditDatas.Rung.Element[x][y].IndexedVarType = RungSrc->Element[srcx][srcy].IndexedVarType;
					EditDatas.Rung.Element[x][y].IndexedVarNum = RungSrc->Element[srcx][srcy].IndexedVarNum;
				}
				x++;
			}
			y++;
		}
	}
}


