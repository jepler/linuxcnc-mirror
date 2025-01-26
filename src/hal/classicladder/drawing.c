/* Classic Ladder Project */
/* Copyright (C) 2001-2009 Marc Le Douarain */
/* http://membres.lycos.fr/mavati/classicladder/ */
/* http://www.sourceforge.net/projects/classicladder */
/* February 2001 */
/* ---------------------------------------- */
/* Draw the ladder rungs                    */
/* + current section draw function          */
/* ======================================== */
/* Switched to CAIRO library in august 2008 */
/* ---------------------------------------- */
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <gtk/gtk.h>
#include <pango/pango.h>

#include "classicladder.h"
#include "global.h"
#include "arithm_eval.h"
#include "classicladder_gtk.h"
#include "edit.h"
#ifdef SEQUENTIAL_SUPPORT
#include "drawing_sequential.h"
#endif
#include "symbols.h"
#include "vars_names.h"
#include "drawing.h"

#include <rtapi_string.h>

PangoLayout * pFontLayout = NULL;
int BlockPxHeightUsedForFont = -1;

int PrintRightMarginExprPosiY = 0;
int PrintRightMarginNumExpr = 0;
int PrintRightMarginPosiX;
int PrintRightMarginWidth;

void CreateVarNameForElement( char * pBuffToWrite, size_t BuffSize, StrElement * pElem, char SymbolsVarsNamesIfAvail )
{
	char VarIndexBuffer[20];
	if ( pElem->IndexedVarType!=-1 )
	{
		// buffer for index required as CreateVarName() returns on a static buffer !
		rtapi_strxcpy( VarIndexBuffer, CreateVarName(pElem->IndexedVarType,pElem->IndexedVarNum,SymbolsVarsNamesIfAvail) );
		snprintf( pBuffToWrite, BuffSize, "%s", CreateVarName(pElem->VarType,pElem->VarNum,SymbolsVarsNamesIfAvail) );
	}
	else
	{
		strcpy( pBuffToWrite, CreateVarName(pElem->VarType,pElem->VarNum,SymbolsVarsNamesIfAvail) );
	}
}

char * DisplayArithmExpr(char * Expr, char SymbolsVarsNamesIfAvail)
{
	static char Buffer[ARITHM_EXPR_SIZE+30];
	char * Ptr = Expr;
	int Fill = 0;
	Buffer[0] = '\0';
	/* null expression ? */
	if (Expr[0]=='\0')
		return Buffer;
	do
	{
		/* start of a variable ? */
		if (*Ptr=='@')
		{
			int NumVar,TypeVar;
			int IndexNumVar,IndexTypeVar;
			char VarBuffer[20];
			char VarIndexBuffer[20];
			if ( IdentifyVarIndexedOrNot( Ptr, &TypeVar, &NumVar, &IndexTypeVar, &IndexNumVar ) )
			{
				if ( IndexTypeVar!=-1 && IndexNumVar!=-1 )
				{
					// buffer for index required as CreateVarName() returns on a static buffer !
					rtapi_strxcpy( VarIndexBuffer, CreateVarName(IndexTypeVar,IndexNumVar,SymbolsVarsNamesIfAvail) );
					size_t ret = snprintf(VarBuffer, sizeof(VarBuffer), "%s[%s]", CreateVarName(TypeVar,NumVar,SymbolsVarsNamesIfAvail), VarIndexBuffer );
					if (ret >= sizeof(VarBuffer)) snprintf(VarBuffer, sizeof(VarBuffer), "<format too long>");
				}
				else
				{
					rtapi_strxcpy(VarBuffer,CreateVarName(TypeVar,NumVar,SymbolsVarsNamesIfAvail));
				}
			}
			else
				rtapi_strxcpy(VarBuffer,"??");
			strcpy(&Buffer[Fill],VarBuffer);
			/* flush until end of a variable */
			do
			{
				Ptr++;
			}
			while(*Ptr!='@');
			Ptr++;
			Fill = Fill+strlen(VarBuffer);
		}
		else
		{
			Buffer[Fill++] = *Ptr++;
		}
	}
	while(*Ptr!='\0');
	Buffer[Fill] = '\0';
	return Buffer;
}

void CreateFontPangoLayout( cairo_t *cr, int BlockPxHeight, char DrawingOption )
{
	if ( pFontLayout==NULL || BlockPxHeightUsedForFont!=BlockPxHeight )
	{
		char BuffFontDesc[ 25 ];
		PangoFontDescription *FontDesc;
		int FontHeight = 8;
		if ( DrawingOption==DRAW_FOR_PRINT )
			FontHeight = BlockPxHeight*8/BLOCK_HEIGHT_DEF;
//TODO: cleanup also needed on exit?
		if ( pFontLayout!=NULL )
			g_object_unref( pFontLayout );
		pFontLayout = pango_cairo_create_layout( cr );

//	FontDesc = pango_font_description_from_string( "Andale Mono 8" );
//Cairo		FontDesc = pango_font_description_from_string( "Courier New 8" );
//	FontDesc = pango_font_description_from_string( "Lucida Bright 8" );
//printf("Font: BlockPxHeight:%d, BLOCK_HEIGHT_DEF:%d\n", BlockPxHeight, BLOCK_HEIGHT_DEF );
		snprintf( BuffFontDesc, sizeof(BuffFontDesc), "Lucida Sans %d", FontHeight );
		FontDesc = pango_font_description_from_string( BuffFontDesc );
		pango_layout_set_font_description( pFontLayout, FontDesc );
		pango_font_description_free( FontDesc );
		BlockPxHeightUsedForFont = BlockPxHeight;
		
		pango_layout_set_wrap( pFontLayout, PANGO_WRAP_CHAR );
	}
}

/* Drawing text with Pango. */
/* if Height is -1, then drawing bottom text on top of BaseY given... */
/* if Height is 0, then drawing directly on BaseY */
/* return pixels height required if more than one line should be displayed (if height space available)
	else 0 (used to print) */
int DrawPangoTextOptions( cairo_t * cr, int BaseX, int BaseY, int Width, int Height, char * Text, char CenterAlignment ) 
{
	int SizeX, SizeY;
	int PosiY = BaseY;
	int TotalHeightRequired = 0;

	if ( pFontLayout==NULL )
		return 0;

	pango_layout_set_width( pFontLayout, Width>0?Width*PANGO_SCALE:-1 );
	pango_layout_set_alignment( pFontLayout, CenterAlignment?PANGO_ALIGN_CENTER:PANGO_ALIGN_LEFT );
	pango_layout_set_text( pFontLayout, Text, -1 );

	pango_layout_get_pixel_size( pFontLayout, &SizeX, &SizeY );
//printf("Pango sizeX=%d,sizeY=%d - width=%d,height=%d text=%s\n",SizeX,SizeY,Width,Height,Text);
	if ( Height<=0 )
	{
		if ( Height==-1 )
			PosiY = BaseY - SizeY; //BaseY is the bottom if Height is -1.
		TotalHeightRequired = SizeY;
	}
	else
	{
		if ( SizeY>Height )
		{
			TotalHeightRequired = SizeY;
			// just display the first line
			PangoLayoutLine * playout_line = pango_layout_get_line( pFontLayout, 0 );
			if ( playout_line!=NULL )
			{
				pango_layout_set_text( pFontLayout, Text, playout_line->length );
				pango_layout_get_pixel_size( pFontLayout, &SizeX, &SizeY );
			}
		}
		// vertical centering
		if ( SizeY<Height )
			PosiY = BaseY + (Height-SizeY)/2;
	}
	cairo_move_to( cr, BaseX, PosiY );
	pango_cairo_show_layout( cr, pFontLayout );
	return TotalHeightRequired;
}
// draw a text centered
int DrawPangoText( cairo_t * cr, int BaseX, int BaseY, int Width, int Height, char * Text ) 
{
	return DrawPangoTextOptions( cr, BaseX, BaseY, Width, Height, Text, /*CenterAlignment*/Width>0?TRUE:FALSE );
}

// if expr not drawed entirely, draw it complete in the right margin
// return report number full expression for margin (added after word OPERATE/COMPARE)
char * DrawExprForCompareOperate( cairo_t * cr, int BaseX, int BaseY, int Width, int Height, char * Text,char DrawingOption )
{
	static char tcBuffNumExpr[10];
	tcBuffNumExpr[ 0 ] = '\0';
	int NbrPixHeight = DrawPangoTextOptions( cr, BaseX, BaseY, Width, Height, Text, FALSE/*CenterAlignment*/ );
	// expression not printed entirely ?
	if ( NbrPixHeight>0 )
	{
		if ( DrawingOption==DRAW_FOR_PRINT )
		{
			// print the expression entirely in the right margin
			size_t TextSize;
			char * pReportNumAndText = malloc( TextSize = strlen(Text)+10 );
			if ( pReportNumAndText )
			{
				int Hgt;
				snprintf( pReportNumAndText, TextSize, "(*%d) ", PrintRightMarginNumExpr );
				strcat( pReportNumAndText, Text );
				Hgt = DrawPangoTextOptions( cr, PrintRightMarginPosiX, PrintRightMarginExprPosiY, PrintRightMarginWidth, 0/*Height*/, pReportNumAndText, FALSE/*CenterAlignment*/ );
				PrintRightMarginExprPosiY += Hgt;
				free( pReportNumAndText );
			}
			snprintf( tcBuffNumExpr, sizeof(tcBuffNumExpr), "(*%d)", PrintRightMarginNumExpr++ );
		}
		else
		{
			rtapi_strxcpy( tcBuffNumExpr, "(...)" );
		}
	}
	return tcBuffNumExpr;
}

void DrawCommonElementForToolbar( cairo_t * cr,int x,int y,int Size,int NumElement )
{
	int SizeDiv2 = Size/2;
	int SizeDiv3 = Size/3;
	int SizeDiv4 = Size/4;
	cairo_set_source_rgb( cr, 0.0, 0.0, 0.0 );
	switch(NumElement)
	{
		case EDIT_POINTER:
			my_cairo_draw_line(cr,
				x+SizeDiv4,y+SizeDiv4, x+Size-SizeDiv4,y+Size-SizeDiv4); /* \ */
			my_cairo_draw_line(cr,
				x+SizeDiv4,y+SizeDiv4, x+SizeDiv3,y+SizeDiv2);  /* | */
			my_cairo_draw_line(cr,
				x+SizeDiv4,y+SizeDiv4, x+SizeDiv2,y+SizeDiv3);  /* _ */
			my_cairo_draw_line(cr,
				x+SizeDiv3,y+SizeDiv2, x+SizeDiv2,y+SizeDiv3);
			break;
		case EDIT_ERASER:
			my_cairo_draw_line(cr,
				x+SizeDiv3,y+SizeDiv4, x+Size-SizeDiv4,y+SizeDiv4);
			my_cairo_draw_line(cr,
				x+SizeDiv4,y+Size-SizeDiv4, x+Size-SizeDiv3,y+Size-SizeDiv4);
			my_cairo_draw_line(cr,
				x+SizeDiv3,y+SizeDiv4, x+SizeDiv4,y+Size-SizeDiv4);
			my_cairo_draw_line(cr,
				x+Size-SizeDiv4,y+SizeDiv4, x+Size-SizeDiv3,y+Size-SizeDiv4);
			my_cairo_draw_line(cr,
				x+SizeDiv4 +2,y+Size-SizeDiv4 +2, x+Size-SizeDiv3 +2,y+Size-SizeDiv4 +2);
			my_cairo_draw_line(cr,
				x+Size-SizeDiv4 +2,y+SizeDiv4 +2, x+Size-SizeDiv3 +2,y+Size-SizeDiv4 +2);
			break;
		case EDIT_SELECTION:
			{
				const double lgt_dashes[] = { 2.0 };
				cairo_set_dash( cr, lgt_dashes, 1, 0);
				cairo_rectangle( cr, x+SizeDiv4, y+SizeDiv4, Size-2*SizeDiv4, Size-2*SizeDiv4 );
				cairo_stroke( cr );
			}
			break;
		case EDIT_COPY:
			{
				const double lgt_dashes[] = { 2.0 };
				cairo_rectangle( cr, x+SizeDiv4, y+SizeDiv4, SizeDiv2, SizeDiv2 );
				cairo_stroke( cr );
				cairo_set_dash( cr, lgt_dashes, 1, 0);
				cairo_rectangle( cr, x+SizeDiv3, y+SizeDiv3, SizeDiv2, SizeDiv2 );
				cairo_stroke( cr );
			}
			break;
	}
}

void my_cairo_line( cairo_t *cr, double x1, double y1, double x2, double y2 )
{
	cairo_move_to( cr, x1, y1 );
	cairo_line_to( cr, x2, y2 );
}
void my_cairo_draw_line( cairo_t *cr, double x1, double y1, double x2, double y2 )
{
	cairo_move_to( cr, x1, y1 );
	cairo_line_to( cr, x2, y2 );
	cairo_stroke( cr );
}
void my_cairo_draw_color_line( cairo_t *cr, char cColor, double x1, double y1, double x2, double y2 )
{
	cairo_save( cr );
	if ( cColor )
		cairo_set_source_rgb( cr, 1.0, 0.13, 1.0 );
	else
		cairo_set_source_rgb( cr, 0.0, 0.0, 0.0 );
	my_cairo_draw_line( cr, x1, y1, x2, y2 );
	cairo_restore( cr );
}
void my_cairo_draw_black_rectangle( cairo_t *cr, double x, double y, double w, double h )
{
	cairo_save( cr );
	cairo_set_source_rgb( cr, 0.0, 0.0, 0.0 );
	cairo_rectangle( cr, x, y, w, h );
	cairo_stroke( cr );
	cairo_restore( cr );
}

char CharUnitForTimer( int BaseTimeValue )
{
	if ( BaseTimeValue==TIME_BASE_MINS)
		return 'm';
	else if ( BaseTimeValue==TIME_BASE_SECS )
		return 's';
	else if ( BaseTimeValue==TIME_BASE_100MS )
		return 't';
	else
		return '?';
}

void DrawElement( cairo_t * cr,int x,int y,int Width,int Height,StrElement Element,char DrawingOption )
{
	char BufTxt[50];
    char BufTxt2[50];
#ifdef OLD_TIMERS_MONOS_SUPPORT
	StrTimer * Timer;
	StrMonostable * Monostable;
#endif
	StrCounter * Counter;
	StrTimerIEC * TimerIEC;
	int WidDiv2 = Width/2;
	int WidDiv3 = Width/3;
	int WidDiv4 = Width/4;
	int HeiDiv2 = Height/2;
	int HeiDiv3 = Height/3;
	int HeiDiv4 = Height/4;
	int Thickness = Width/16;
	char * ptcBuffNumExprMargin;
	char DisplayColorState = ( DrawingOption==DRAW_NORMAL && !EditDatas.ModeEdit && InfosGene->LadderState==STATE_RUN );

//Cairo....
#ifdef AAAAAAAAAAAAA
	GdkGC * DynaGcOff;
	GdkGC * TheGc;
	GdkColor DynaGdkColor;
	GdkGC * DynaGcOn;
	DynaGdkColor.pixel = 0xFF22FF;
	DynaGdkColor.red = 0xFF;
	DynaGdkColor.green = 0x22;
	DynaGdkColor.blue = 0xFF;

	DynaGcOn = gdk_gc_new(DrawPixmap);
	gdk_gc_set_foreground(DynaGcOn,&DynaGdkColor);
	#ifdef THICK_LINE_ELE_ACTIVATED
	gdk_gc_set_line_attributes(DynaGcOn, THICK_LINE_ELE_ACTIVATED,
		GDK_LINE_SOLID, GDK_CAP_BUTT, GDK_JOIN_MITER);
	#endif
	DynaGcOff = drawing_area->style->black_gc;
	/* State with color */
	TheGc = drawing_area->style->black_gc;
	if ( (DrawingOption==DRAW_NORMAL) && (!EditDatas.ModeEdit) && (Element.DynamicState) )
		TheGc = DynaGcOn;
	if (EditDatas.ModeEdit || DrawingOption==DRAW_FOR_PRINT)
	{
		gdk_gc_unref(DynaGcOn);
		DynaGcOn = drawing_area->style->black_gc;
	}
#endif

//	cairo_save( cr );
	if ( DisplayColorState && (Element.DynamicState) )
	{
		cairo_set_source_rgb( cr, 1.0, 0.13, 1.0 );
//		cairo_set_line_width( cr, 1.5*cairo_get_line_width( cr ) );
	}
	else
	{
		cairo_set_source_rgb( cr, 0.0, 0.0, 0.0 );
//		cairo_set_line_width( cr, 0.1*cairo_get_line_width( cr ) );
	}

	/* modified: drawing this ones first (before -  -) because trouble under Windows printing... */
	/* Drawing / */
	switch(Element.Type)
	{
		case ELE_INPUT_NOT:
		case ELE_OUTPUT_NOT:
				my_cairo_draw_line( cr,
								x+WidDiv3,y+Height-HeiDiv4, x+WidDiv3*2,y+HeiDiv4 );
	}
	/* Drawing ^ or \/  */
	switch(Element.Type)
	{
		case ELE_RISING_INPUT:
				my_cairo_draw_line( cr,
								x+WidDiv3,y+HeiDiv3*2, x+WidDiv4*2,y+HeiDiv3 );
				my_cairo_draw_line( cr,
								x+WidDiv4*2,y+HeiDiv3, x+WidDiv3*2,y+HeiDiv3*2 );
				break;
		case ELE_FALLING_INPUT:
				my_cairo_draw_line( cr,
								x+WidDiv3,y+HeiDiv3, x+WidDiv4*2,y+HeiDiv3*2 );
				my_cairo_draw_line( cr,
								x+WidDiv4*2,y+HeiDiv3*2, x+WidDiv3*2,y+HeiDiv3 );
				break;
	}

	/* Drawing - - */
	switch(Element.Type)
	{
		case ELE_INPUT:
		case ELE_INPUT_NOT:
		case ELE_RISING_INPUT:
		case ELE_FALLING_INPUT:
		case ELE_OUTPUT:
		case ELE_OUTPUT_NOT:
		case ELE_OUTPUT_SET:
		case ELE_OUTPUT_RESET:
		case ELE_OUTPUT_JUMP:
		case ELE_OUTPUT_CALL:
				my_cairo_draw_line( cr,
								x,y+HeiDiv2, x+WidDiv3,y+HeiDiv2 );
				my_cairo_draw_line( cr,
								x+WidDiv3*2,y+HeiDiv2, x+Width,y+HeiDiv2 );
	}
	/* Drawing || or () or --- */
	switch(Element.Type)
	{
		case ELE_INPUT:
		case ELE_INPUT_NOT:
		case ELE_RISING_INPUT:
		case ELE_FALLING_INPUT:
				my_cairo_draw_line( cr,
								x+WidDiv3,y+HeiDiv4, x+WidDiv3,y+Height-HeiDiv4);
				my_cairo_draw_line( cr,
								x+WidDiv3*2,y+HeiDiv4, x+WidDiv3*2,y+Height-HeiDiv4 );
				break;
		case ELE_CONNECTION:
				my_cairo_draw_line( cr,
								x,y+HeiDiv2, x+Width,y+HeiDiv2 );
				break;
		case ELE_OUTPUT:
		case ELE_OUTPUT_NOT:
		case ELE_OUTPUT_SET:
		case ELE_OUTPUT_RESET:
		case ELE_OUTPUT_JUMP:
		case ELE_OUTPUT_CALL:
				/* hide the too much of lines - - before drawing arcs */
//Cairo				gdk_draw_rectangle(DrawPixmap, (DrawingOption!=DRAW_FOR_TOOLBAR)?drawing_area->style->white_gc:drawing_area->style->bg_gc[0], TRUE,
//Cairo								x+WidDiv4,y+HeiDiv2-1, WidDiv2,3);
				/* draw the 2 arcs of the outputs */
//Cairo				gdk_draw_arc (DrawPixmap, TheGc, FALSE,
//Cairo								x+WidDiv4, y+HeiDiv4, WidDiv2, HeiDiv2, (90+20)*64, 150*64);
//Cairo				gdk_draw_arc (DrawPixmap, TheGc, FALSE,
//Cairo								x+WidDiv4, y+HeiDiv4, WidDiv2, HeiDiv2, (270+20)*64, 150*64);
cairo_save (cr);
cairo_translate (cr, x + WidDiv2, y + HeiDiv2);
cairo_scale (cr, WidDiv4-4, HeiDiv4);
//cairo_arc (cr, 0, 0, 1., 0., 2 * M_PI);
cairo_new_sub_path( cr );
cairo_arc (cr, 0, 0, 1., M_PI/2.+0.3, M_PI+M_PI/2.-0.3);
cairo_new_sub_path( cr );
cairo_arc (cr, 0, 0, 1., M_PI+M_PI/2.+0.3, M_PI/2-0.3 );
cairo_restore (cr);
cairo_stroke( cr );
				break;
	}

	/* Drawing 'S'et or 'R'eset or 'J'ump or 'C'all for outputs */
	switch(Element.Type)
	{
		case ELE_OUTPUT_SET:
			DrawPangoText( cr, x, y, Width, Height, "S" );
			break;
		case ELE_OUTPUT_RESET:
			DrawPangoText( cr, x, y, Width, Height, "R" );
			break;
		case ELE_OUTPUT_JUMP:
			DrawPangoText( cr, x, y, Width, Height, "J" );
			break;
		case ELE_OUTPUT_CALL:
			DrawPangoText( cr, x, y, Width, Height, "C" );
			break;
	}

	/* Drawing complex ones : Timer, Monostable, Compar, Operate */
	switch(Element.Type)
	{
#ifdef OLD_TIMERS_MONOS_SUPPORT
		case ELE_TIMER:
			if (DrawingOption==DRAW_FOR_TOOLBAR)
				break;
			Timer = &TimerArray[Element.VarNum];
			/* the box */
//Cairo			gdk_draw_rectangle(DrawPixmap, drawing_area->style->white_gc, TRUE,
//								x+WidDiv3-Width, y+HeiDiv3,
//								Width+1*WidDiv3, Height+1*HeiDiv3);
			my_cairo_draw_black_rectangle( cr,
								x+WidDiv3-Width, y+HeiDiv3,
								Width+1*WidDiv3, Height+1*HeiDiv3 );
			/* input : enable */
			my_cairo_draw_color_line( cr, Timer->InputEnable && DisplayColorState,
								x-Width,y+HeiDiv2, x-Width+WidDiv3,y+HeiDiv2 );
			DrawPangoText( cr, x-Width+2, y+HeiDiv2-1, -1, -1, "E" );
			/* input : control */
			my_cairo_draw_color_line( cr, Timer->InputControl && DisplayColorState,
								x-Width,y+HeiDiv2+Height, x-Width+WidDiv3,y+HeiDiv2+Height );
			DrawPangoText( cr, x-Width+2, y+HeiDiv2-1+Height, -1, -1, "C" );
			/* output : done */
			my_cairo_draw_color_line( cr, Timer->OutputDone && DisplayColorState,
								x+WidDiv3*2,y+HeiDiv2, x+Width,y+HeiDiv2 );
			DrawPangoText( cr, x+Width-WidDiv4,y+HeiDiv2-1, -1, -1, "D" );
			/* output : running */
			my_cairo_draw_color_line( cr, Timer->OutputRunning && DisplayColorState,
								x+WidDiv3*2,y+HeiDiv2+Height, x+Width,y+HeiDiv2+Height );
			DrawPangoText( cr, x+Width-WidDiv4,y+HeiDiv2-1+Height, -1, -1, "R" );
			/* Timer Number */
			snprintf( BufTxt, sizeof(BufTxt),"%cT%d", '%', Element.VarNum );
			if ( InfosGene->DisplaySymbols )
			{
				StrSymbol * SymbolName = ConvVarNameInSymbolPtr( BufTxt );
				if ( SymbolName )
					rtapi_strxcpy( BufTxt, SymbolName->Symbol );
			}
			DrawPangoText( cr, x+WidDiv3-Width,y+HeiDiv4+2, (Width-WidDiv3)*2, -1, BufTxt );
			/* Current Value (or Preset if print/edit) */
			if ( DrawingOption!=DRAW_FOR_PRINT && !EditDatas.ModeEdit )
				snprintf(BufTxt, sizeof(BufTxt),Timer->DisplayFormat,(double)Timer->Value/(double)Timer->Base);
			else
				snprintf(BufTxt, sizeof(BufTxt),Timer->DisplayFormat,(double)Timer->Preset/(double)Timer->Base);
			DrawPangoText( cr, x-Width, y, Width*2, Height*2, BufTxt );
			break;

		case ELE_MONOSTABLE:
			if (DrawingOption==DRAW_FOR_TOOLBAR)
				break;
			Monostable = &MonostableArray[Element.VarNum];
			/* the box */
//Cairo			gdk_draw_rectangle(DrawPixmap, drawing_area->style->white_gc, TRUE,
//								x+WidDiv3-Width, y+HeiDiv3,
//								Width+1*WidDiv3, Height+1*HeiDiv3);
			my_cairo_draw_black_rectangle( cr,
								x+WidDiv3-Width, y+HeiDiv3,
								Width+1*WidDiv3, Height+1*HeiDiv3 );
			/* input */
			my_cairo_draw_color_line( cr, Monostable->Input && DisplayColorState,
								x-Width,y+HeiDiv2, x-Width+WidDiv3,y+HeiDiv2 );
			DrawPangoText( cr, x-Width,y+HeiDiv2-1, -1, -1, "I^" );
			/* output : running */
			my_cairo_draw_color_line( cr, Monostable->OutputRunning && DisplayColorState,
								x+WidDiv3*2,y+HeiDiv2, x+Width,y+HeiDiv2 );

			DrawPangoText( cr, x+Width-WidDiv4,y+HeiDiv2-1, -1, -1, "R" );
			/* Monostable Number */
			snprintf( BufTxt, sizeof(BufTxt),"%cM%d", '%', Element.VarNum );
			if ( InfosGene->DisplaySymbols )
			{
				StrSymbol * SymbolName = ConvVarNameInSymbolPtr( BufTxt );
				if ( SymbolName )
					rtapi_strxcpy( BufTxt, SymbolName->Symbol );
			}
			DrawPangoText( cr, x+WidDiv3-Width,y+HeiDiv4+2, (Width-WidDiv3)*2, -1, BufTxt );
			/* Current Value (or Preset if print/edit) */
			if ( DrawingOption!=DRAW_FOR_PRINT && !EditDatas.ModeEdit )
				snprintf(BufTxt, sizeof(BufTxt),Monostable->DisplayFormat,(double)Monostable->Value/(double)Monostable->Base);
			else
				snprintf(BufTxt, sizeof(BufTxt),Monostable->DisplayFormat,(double)Monostable->Preset/(double)Monostable->Base);
			DrawPangoText( cr, x-Width, y, Width*2, Height*2, BufTxt );
			break;
#endif

		case ELE_COUNTER:
			if (DrawingOption==DRAW_FOR_TOOLBAR)
				break;
			Counter = &CounterArray[Element.VarNum];
			/* the box */
//Cairo			gdk_draw_rectangle(DrawPixmap, drawing_area->style->white_gc, TRUE,
//								x+WidDiv3-Width, y+HeiDiv3,
//								Width+1*WidDiv3, 3*Height+1*HeiDiv3);
			my_cairo_draw_black_rectangle( cr, 
								x+WidDiv3-Width, y+HeiDiv3,
								Width+1*WidDiv3, 3*Height+1*HeiDiv3 );
			/* input : reset */
			my_cairo_draw_color_line( cr, Counter->InputReset && DisplayColorState,
								x-Width,y+HeiDiv2, x-Width+WidDiv3,y+HeiDiv2 );
			DrawPangoText( cr, x-Width+WidDiv4/2, y+HeiDiv2-1, -1, -1, "R" );
			/* input : preset */
			my_cairo_draw_color_line( cr, Counter->InputPreset && DisplayColorState,
								x-Width,y+HeiDiv2+Height, x-Width+WidDiv3,y+HeiDiv2+Height );
			DrawPangoText( cr, x-Width+WidDiv4/2, y+HeiDiv2-1+Height, -1, -1, "P" );
			/* input : count up */
			my_cairo_draw_color_line( cr, Counter->InputCountUp && DisplayColorState,
								x-Width,y+HeiDiv2+Height*2, x-Width+WidDiv3,y+HeiDiv2+Height*2 );
			DrawPangoText( cr, x-Width+WidDiv4/2, y+HeiDiv2-1+Height*2, -1, -1, "U" );
			/* input : count down */
			my_cairo_draw_color_line( cr, Counter->InputCountDown && DisplayColorState,
								x-Width,y+HeiDiv2+Height*3, x-Width+WidDiv3,y+HeiDiv2+Height*3 );
			DrawPangoText( cr, x-Width+WidDiv4/2, y+HeiDiv2-1+Height*3, -1, -1, "D" );
			/* output : empty */
			my_cairo_draw_color_line( cr, Counter->OutputEmpty && DisplayColorState,
								x+WidDiv3*2,y+HeiDiv2, x+Width,y+HeiDiv2 );
			DrawPangoText( cr, x+Width-WidDiv4,y+HeiDiv2-1, -1, -1, "E" );
			/* output : done */
			my_cairo_draw_color_line( cr, Counter->OutputDone && DisplayColorState,
								x+WidDiv3*2,y+HeiDiv2+Height, x+Width,y+HeiDiv2+Height );
			DrawPangoText( cr, x+Width-WidDiv4,y+HeiDiv2-1+Height, -1, -1, "D" );
			/* output : full */
			my_cairo_draw_color_line( cr, Counter->OutputFull && DisplayColorState,
								x+WidDiv3*2,y+HeiDiv2+Height*2, x+Width,y+HeiDiv2+Height*2 );
			DrawPangoText( cr, x+Width-WidDiv4,y+HeiDiv2-1+Height*2, -1, -1, "F" );
			/* Counter Number */
			snprintf( BufTxt, sizeof(BufTxt),"%cC%d", '%', Element.VarNum );
			if ( InfosGene->DisplaySymbols )
			{
				StrSymbol * SymbolName = ConvVarNameInSymbolPtr( BufTxt );
				if ( SymbolName )
					rtapi_strxcpy( BufTxt, SymbolName->Symbol );
			}
			DrawPangoText( cr, x+WidDiv3-Width,y+HeiDiv4+2, (Width-WidDiv3)*2, -1, BufTxt );
            /* Current and Preset values */
			snprintf(BufTxt, sizeof(BufTxt),"%d",Counter->Value);
            snprintf(BufTxt2, sizeof(BufTxt2), "%d",Counter->Preset);
			if ( DisplayColorState )
				DrawPangoText( cr, x-Width, y+HeiDiv2, Width*2, Height*2, BufTxt );
			DrawPangoText( cr, x-Width, y+HeiDiv2+HeiDiv3, Width*2, Height*2, BufTxt2 );
			break;

		case ELE_TIMER_IEC:
			if (DrawingOption==DRAW_FOR_TOOLBAR)
				break;
			TimerIEC = &NewTimerArray[Element.VarNum];
			/* the box */
//Cairo			gdk_draw_rectangle(DrawPixmap, drawing_area->style->white_gc, TRUE,
//								x+WidDiv3-Width, y+HeiDiv3,
//								Width+1*WidDiv3, Height+1*HeiDiv3);
			my_cairo_draw_black_rectangle( cr,
								x+WidDiv3-Width, y+HeiDiv3,
								Width+1*WidDiv3, Height+1*HeiDiv3 );
			/* input : enable */
			my_cairo_draw_color_line( cr, TimerIEC->Input && DisplayColorState,
								x-Width,y+HeiDiv2, x-Width+WidDiv3,y+HeiDiv2 );
			DrawPangoText( cr, x-Width+WidDiv4/1.5, y+HeiDiv2-1, -1, -1, "I" );
			/* output : done */
			my_cairo_draw_color_line( cr, TimerIEC->Output && DisplayColorState,
								x+WidDiv3*2,y+HeiDiv2, x+Width,y+HeiDiv2 );
			DrawPangoText( cr, x+Width-WidDiv4,y+HeiDiv2-1, -1, -1, "Q" );
			/* Timer IEC Number */
			snprintf( BufTxt, sizeof(BufTxt),"%cTM%d", '%', Element.VarNum );
			if ( InfosGene->DisplaySymbols )
			{
				StrSymbol * SymbolName = ConvVarNameInSymbolPtr( BufTxt );
				if ( SymbolName )
					rtapi_strxcpy( BufTxt, SymbolName->Symbol );
			}
			DrawPangoText( cr, x+WidDiv3-Width,y+HeiDiv4+2, (Width-WidDiv3)*2, -1, BufTxt );
			/* Timer mode */
			snprintf( BufTxt, sizeof(BufTxt), "%s", TimersModesStrings[ (int)TimerIEC->TimerMode ] );
			DrawPangoText( cr, x-Width, y+HeiDiv3, Width*2, Height-HeiDiv3, BufTxt );

            /* Current and Preset values */
			snprintf(BufTxt, sizeof(BufTxt),/*TimerIEC->DisplayFormat*/"%d%c", TimerIEC->Value, CharUnitForTimer(TimerIEC->Base ));
            snprintf(BufTxt2, sizeof(BufTxt2),/*TimerIEC->DisplayFormat*/"%d%c", TimerIEC->Preset, CharUnitForTimer(TimerIEC->Base));
			if ( DisplayColorState )
				DrawPangoText( cr, x-Width, y+Height+HeiDiv3, Width*2, -1/*Height-HeiDiv3*/, BufTxt );
			DrawPangoText( cr, x-Width, y+Height+2*HeiDiv3, Width*2, -1/*Height-HeiDiv3*/, BufTxt2 );
			break;

		case ELE_COMPAR:
			if (DrawingOption==DRAW_FOR_TOOLBAR)
				break;
			/* the box */
//Cairo			gdk_draw_rectangle(DrawPixmap, drawing_area->style->white_gc, TRUE,
//								x+WidDiv3-(Width*2), y+HeiDiv4,
//								Width*2+1*WidDiv3, 2*HeiDiv4);
			my_cairo_draw_black_rectangle( cr, 
								x+WidDiv3-(Width*2), y+HeiDiv4,
								Width*2+1*WidDiv3, 2*HeiDiv4 );
			/* input */
			my_cairo_draw_line( cr,
								x-Width*2,y+HeiDiv2, x-Width*2+WidDiv3,y+HeiDiv2 );
			/* output */
			my_cairo_draw_line( cr,
								x+WidDiv3*2,y+HeiDiv2, x+Width,y+HeiDiv2 );
			/* arithmetic expression */
			if (!EditDatas.ModeEdit)
				DisplayArithmExpr(ArithmExpr[Element.VarNum].Expr,InfosGene->DisplaySymbols);
			else
				DisplayArithmExpr(EditArithmExpr[Element.VarNum].Expr,InfosGene->DisplaySymbols);
			ptcBuffNumExprMargin = DrawExprForCompareOperate( cr, x+WidDiv3-(Width*2)+2,y+HeiDiv4, Width*2+1*WidDiv3-Thickness, 2*HeiDiv4, BufTxt, DrawingOption );
			cairo_set_source_rgb( cr, 0.0, 0.0, 0.0 );
			snprintf( BufTxt, sizeof(BufTxt), "COMPARE%s", ptcBuffNumExprMargin );
			DrawPangoText( cr, x+WidDiv4-(Width*2)+3, y+HeiDiv4+1, -1, -1, BufTxt );
			break;
		case ELE_OUTPUT_OPERATE:
			if (DrawingOption==DRAW_FOR_TOOLBAR)
				break;
			/* the box */
//Cairo			gdk_draw_rectangle(DrawPixmap, drawing_area->style->white_gc, TRUE,
//								x+WidDiv3-(Width*2), y+HeiDiv4,
//								Width*2+1*WidDiv3, 2*HeiDiv4);
			my_cairo_draw_black_rectangle( cr,
								x+WidDiv3-(Width*2), y+HeiDiv4,
								Width*2+1*WidDiv3, 2*HeiDiv4 );
			/* input */
			my_cairo_draw_line( cr,
								x-Width*2,y+HeiDiv2, x-Width*2+WidDiv3,y+HeiDiv2 );
			/* output */
			my_cairo_draw_line( cr,
								x+WidDiv3*2,y+HeiDiv2, x+Width,y+HeiDiv2 );
			/* arithmetic expression */
			if (!EditDatas.ModeEdit)
				DisplayArithmExpr(ArithmExpr[Element.VarNum].Expr,InfosGene->DisplaySymbols);
			else
				DisplayArithmExpr(EditArithmExpr[Element.VarNum].Expr,InfosGene->DisplaySymbols);
			ptcBuffNumExprMargin = DrawExprForCompareOperate( cr,x+WidDiv3-(Width*2)+2,y+HeiDiv4, Width*2+1*WidDiv3-Thickness, 2*HeiDiv4, BufTxt, DrawingOption );
			cairo_set_source_rgb( cr, 0.0, 0.0, 0.0 );
			snprintf( BufTxt, sizeof(BufTxt), "OPERATE%s", ptcBuffNumExprMargin );
			DrawPangoText( cr, x+WidDiv4-(Width*2)+3, y+HeiDiv4+1, -1, -1, BufTxt );
			break;
	}

	/* Drawing Var */
	if (DrawingOption!=DRAW_FOR_TOOLBAR)
	{
		switch(Element.Type)
		{
			case ELE_INPUT:
			case ELE_INPUT_NOT:
			case ELE_RISING_INPUT:
			case ELE_FALLING_INPUT:
			case ELE_OUTPUT:
			case ELE_OUTPUT_NOT:
			case ELE_OUTPUT_SET:
			case ELE_OUTPUT_RESET:
				CreateVarNameForElement( BufTxt, sizeof(BufTxt), &Element, InfosGene->DisplaySymbols );
				DrawPangoText( cr, x, y+HeiDiv4+1, Width, -1, BufTxt );
				break;
			case ELE_OUTPUT_JUMP:
				DrawPangoText( cr, x, y+HeiDiv4+1, Width, -1, RungArray[Element.VarNum].Label );
				break;
			case ELE_OUTPUT_CALL:
				snprintf( BufTxt,sizeof(BufTxt), "SR%d", Element.VarNum );
				DrawPangoText( cr, x, y+HeiDiv4+1, Width, -1, BufTxt );
				break;
		}
	}
	/* Drawing cnx with top */
	if (Element.ConnectedWithTop)
	{
		if (Element.DynamicInput && DisplayColorState )
			my_cairo_draw_color_line( cr, 1,
				x,y+HeiDiv2 +1, x,y-HeiDiv2 );
		else
			my_cairo_draw_color_line( cr, 0,
				x,y+HeiDiv2 +1, x,y-HeiDiv2 );
	}

	/* specials used for Editor */
	if (DrawingOption==DRAW_FOR_TOOLBAR)
	{
		switch(Element.Type)
		{
			case EDIT_CNX_WITH_TOP:
				my_cairo_draw_color_line( cr, 0,
					x+WidDiv2,y+HeiDiv4, x+WidDiv2,y+Height-HeiDiv4 );
				break;
			case EDIT_LONG_CONNECTION:
				my_cairo_draw_line( cr,
								x,y+HeiDiv2, x+Width-1,y+HeiDiv2 );
				my_cairo_draw_line( cr,
								x+3*WidDiv4-1,y+HeiDiv4, x+Width-1,y+HeiDiv2 );
				my_cairo_draw_line( cr,
								x+3*WidDiv4-1,y+3*HeiDiv4, x+Width-1,y+HeiDiv2 );
				my_cairo_draw_line( cr,
								x+3*WidDiv4-1,y+HeiDiv4, x+3*WidDiv4 +3,y+HeiDiv2 );
				my_cairo_draw_line( cr,
								x+3*WidDiv4 +3,y+HeiDiv2, x+3*WidDiv4-1,y+3*HeiDiv4 );
				break;
			/* little display used for the toolbar */
			case ELE_TIMER:
			case ELE_MONOSTABLE:
			case ELE_COUNTER:
			case ELE_TIMER_IEC:
				{
					char * Letter = "T";
					if ( Element.Type==ELE_MONOSTABLE )
						Letter = "M";
					if ( Element.Type==ELE_COUNTER )
						Letter = "C";
					if ( Element.Type==ELE_TIMER_IEC )
						Letter = "TM";
					my_cairo_draw_black_rectangle( cr,
									x/*+WidDiv4*/, y+HeiDiv4,
									Width-2/**WidDiv4*/, Height-2*HeiDiv4 );
					DrawPangoText( cr, x, y, Width, Height, Letter );
				}
				break;
			case ELE_COMPAR:
				my_cairo_draw_black_rectangle( cr,
									x+WidDiv4, y+HeiDiv4,
									Width-2*WidDiv4, Height-2*HeiDiv4 );
				DrawPangoText( cr, x, y, Width, Height, ">" );
				break;
			case ELE_OUTPUT_OPERATE:
				my_cairo_draw_black_rectangle( cr,
									x+WidDiv4, y+HeiDiv4,
									Width-2*WidDiv4, Height-2*HeiDiv4 );
				DrawPangoText( cr, x, y, Width, Height, "=" );
				break;
			default:
				DrawCommonElementForToolbar( cr, x, y, Width, Element.Type );
				break;
		}
	}
	else
	{
		// only useful to see abnormal elements in a rung (eraser bug in versions < 0.7.124)
		if ( Element.Type>=EDIT_CNX_WITH_TOP )
		{
			snprintf(BufTxt, sizeof(BufTxt), "(%d)", Element.Type );
			DrawPangoText( cr, x, y, Width, Height, BufTxt );
		}
	}
//Cairo	if (!EditDatas.ModeEdit && DrawingOption!=DRAW_FOR_PRINT )
//Cairo		gdk_gc_unref(DynaGcOn);
//	cairo_restore( cr );
}

void DrawLeftRightBars( cairo_t * cr, int OffX, int PosiY, int BlockWidth, int BlockHeight, int HeaderLabelAndCommentHeight, int LeftRightBarsWidth, int IsTheCurrentRung )
{
	int LeftRightBarsHeight = HeaderLabelAndCommentHeight+BlockHeight*RUNG_HEIGHT;

	if (!IsTheCurrentRung )
	{
		cairo_set_source_rgb( cr, 0.0, 0.0, 0.0 );
	}
	else
	{
		// light yellow background for current rung
		cairo_set_source_rgb( cr, 1.0, 1.0, 0.85 );
		cairo_rectangle( cr, OffX+LeftRightBarsWidth, PosiY, BlockWidth*RUNG_WIDTH, LeftRightBarsHeight );
		cairo_fill( cr );
		cairo_set_source_rgb( cr, 0.93, 0.33, 0.33 );
	}
	cairo_rectangle( cr, OffX, PosiY, LeftRightBarsWidth, LeftRightBarsHeight );
	cairo_rectangle( cr, OffX+BlockWidth*RUNG_WIDTH+LeftRightBarsWidth, PosiY, LeftRightBarsWidth, LeftRightBarsHeight );
	cairo_fill( cr );
}

void DrawGrid( cairo_t * cr, int OffX, int PosiY )
{
	int x,y;
	PosiY += InfosGene->HeaderLabelCommentHeight;
	cairo_save( cr );
	cairo_set_line_width( cr, 0.25*cairo_get_line_width( cr ) );
	cairo_set_source_rgb( cr, 0.63, 0.82, 0.82 );
	for(x=InfosGene->BlockWidth; x<RUNG_WIDTH*InfosGene->BlockWidth; x=x+InfosGene->BlockWidth)
	{
		cairo_move_to( cr, x+OffX, PosiY );
		cairo_line_to( cr, x+OffX, TOTAL_PX_RUNG_HEIGHT+PosiY );
		cairo_stroke( cr );
	}
	for(y=InfosGene->BlockHeight; y<TOTAL_PX_RUNG_HEIGHT; y=y+InfosGene->BlockHeight)
	{
		cairo_move_to( cr, OffX, y+PosiY );
		cairo_line_to( cr, RUNG_WIDTH*InfosGene->BlockWidth+OffX, y+PosiY );
		cairo_stroke( cr );
	}
	cairo_restore( cr );
}

void DrawRungPartition( cairo_t * cr, int OffX, int PosiY )
{
	cairo_set_source_rgb( cr, 0.8, 0.8, 0.8 );
	cairo_move_to( cr, 0, PosiY );
	cairo_line_to( cr, RUNG_WIDTH*InfosGene->BlockWidth+OffX, PosiY );
	cairo_stroke( cr );
}

void DrawCurrentElementEdited( cairo_t * cr, int OffX, int AddPosiY )
{
	if ( EditDatas.CurrentElementSizeX>0 && EditDatas.CurrentElementSizeY>0 )
	{
		int Left = (EditDatas.CurrentElementPosiX-EditDatas.CurrentElementSizeX+1)*InfosGene->BlockWidth +OffX;
		int Top = EditDatas.CurrentElementPosiY*InfosGene->BlockHeight +AddPosiY +InfosGene->HeaderLabelCommentHeight;
		int Width = EditDatas.CurrentElementSizeX*InfosGene->BlockWidth;
		int Height = EditDatas.CurrentElementSizeY*InfosGene->BlockHeight;
		cairo_set_source_rgb( cr, 0.99, 0.19, 0.19 );
//		cairo_rectangle( cr, Left, Top, Width, Height );
//		cairo_stroke( cr );
		cairo_rectangle( cr, Left, Top, Width, Height );
		cairo_clip( cr );
		cairo_paint_with_alpha( cr, 0.3 );
		cairo_reset_clip( cr );
	}
}
void DrawGhostZone( cairo_t * cr, int OffX, int AddPosiY )
{
	int Left = EditDatas.GhostZonePosiX*InfosGene->BlockWidth +OffX;
	int Top = EditDatas.GhostZonePosiY*InfosGene->BlockHeight +AddPosiY +InfosGene->HeaderLabelCommentHeight;
	int Width = EditDatas.GhostZoneSizeX*InfosGene->BlockWidth;
	int Height = EditDatas.GhostZoneSizeY*InfosGene->BlockHeight;
	cairo_save( cr );
	cairo_set_source_rgb( cr, 0.50, 0.20, 0.90 );
	if ( EditDatas.NumElementSelectedInToolBar==EDIT_SELECTION )
	{
		const double lgt_dashes[] = { 5.0 };
		cairo_set_dash( cr, lgt_dashes, 1, 0);
	}
	cairo_rectangle( cr, Left, Top, Width, Height );
	cairo_stroke( cr );
	cairo_restore( cr );
}

void DrawPathLabelCommentHeader( cairo_t * cr, StrRung * Rung, int OffX, int PosiY, int BlockWidth, int HeaderHeight, char Background, char DrawingOption )
{
	(void)DrawingOption;
	int HeaderBarHeight = HeaderHeight*70/100;
	int OffsetStartY = HeaderHeight*15/100;
	int Margin = BlockWidth/4;
	int Oblique = BlockWidth/2;
	PosiY = PosiY+OffsetStartY;
	if ( !Background )
		my_cairo_draw_line( cr, OffX+Margin+Oblique+(BlockWidth*2), PosiY, OffX+Margin+(BlockWidth*2), PosiY+HeaderBarHeight );
	cairo_move_to( cr, OffX+Margin+Oblique, PosiY  );
	cairo_line_to( cr, OffX+BlockWidth*RUNG_WIDTH-Margin, PosiY );
	cairo_line_to( cr, OffX+BlockWidth*RUNG_WIDTH-Margin-Oblique, PosiY+HeaderBarHeight );
	cairo_line_to( cr, OffX+Margin, PosiY+HeaderBarHeight );
	cairo_line_to( cr, OffX+Margin+Oblique, PosiY  );
	if ( Background )
	{
		cairo_fill( cr );
	}
	else
	{
		cairo_stroke( cr );
		DrawPangoTextOptions( cr, OffX+Margin+Oblique, PosiY, -1, HeaderBarHeight, Rung->Label, FALSE/*CenterAlignment*/ );
		DrawPangoTextOptions( cr, OffX+BlockWidth*3, PosiY, -1, HeaderBarHeight, Rung->Comment, FALSE/*CenterAlignment*/ );
	}
}
void DrawRung( cairo_t * cr, StrRung * Rung, int OffX, int PosiY, int BlockWidth, int BlockHeight, int HeaderLabelAndCommentHeight, char DrawingOption )
{
	int x,y;
	PrintRightMarginExprPosiY = PosiY;
	PrintRightMarginNumExpr = 1;
	CreateFontPangoLayout( cr, BlockHeight, DrawingOption );
	// drawing label & comment at the top of this rung in a box
	if ( DrawingOption!=DRAW_FOR_PRINT )
	{
		cairo_set_source_rgb( cr, 0.92, 0.92, 0.92 );
		DrawPathLabelCommentHeader( cr, Rung, OffX, PosiY, BlockWidth, HeaderLabelAndCommentHeight, 1/*Background*/, DrawingOption );
	}
	cairo_set_source_rgb( cr, 0.0, 0.0, 0.0 );
	DrawPathLabelCommentHeader( cr, Rung, OffX, PosiY, BlockWidth, HeaderLabelAndCommentHeight, 0/*Background*/, DrawingOption );
	// drawing elements of the rung
	for (y=0;y<RUNG_HEIGHT;y++)
	{
		for(x=0;x<RUNG_WIDTH;x++)
		{
			DrawElement( cr, x*BlockWidth+OffX, HeaderLabelAndCommentHeight+y*BlockHeight+PosiY, BlockWidth,BlockHeight, Rung->Element[x][y], DrawingOption );
		}
	}
}

void DrawRungs( cairo_t * cr )
{
	int ScanRung = InfosGene->TopRungDisplayed;
	int ScanY = InfosGene->OffsetHiddenTopRungDisplayed;
	StrRung * PtrRung;
	int FlagAddOrInsertRung = FALSE;
	int TheEnd = FALSE;
	int LeftRightBarsWidth = InfosGene->BlockWidth/16;

	// If we are editing and adding a rung, we set a flag to indicate
	// that there is an extra rung to add (before or after current rung)
	if ( EditDatas.ModeEdit && EditDatas.DoBeforeFinalCopy == MODE_ADD )
	{
		// beware not the first rung displayed at the top...
		if ( InfosGene->FirstRung!=InfosGene->CurrentRung )
		{
			if ( ScanRung!=InfosGene->LastRung )
				ScanRung = RungArray[ ScanRung ].NextRung;
			FlagAddOrInsertRung = TRUE;
		}
	}
	if ( EditDatas.ModeEdit && EditDatas.DoBeforeFinalCopy == MODE_INSERT )
		FlagAddOrInsertRung = TRUE;

	// Clean all
//Cairo	gdk_draw_rectangle(pixmap, drawing_area->style->white_gc, TRUE,
//Cairo						0, 0, InfosGene->BlockWidth*RUNG_WIDTH+50, InfosGene->PageHeight+50);

	for (ScanY = InfosGene->OffsetHiddenTopRungDisplayed*-1; ScanY<InfosGene->PageHeight && !TheEnd; ScanY=ScanY+(TOTAL_PX_RUNG_HEIGHT))
	{
		char cInEdit = FALSE;
		PtrRung = &RungArray[ ScanRung ];
		DrawLeftRightBars( cr, 0, ScanY, InfosGene->BlockWidth, InfosGene->BlockHeight, InfosGene->HeaderLabelCommentHeight, LeftRightBarsWidth, ScanRung==InfosGene->CurrentRung );
		/* displaying the current rung - in edit ? */
		if ( ScanRung==InfosGene->CurrentRung && EditDatas.ModeEdit )
		{
			if ( ( FlagAddOrInsertRung && EditDatas.DoBeforeFinalCopy==MODE_INSERT )
				|| ( !FlagAddOrInsertRung && EditDatas.DoBeforeFinalCopy==MODE_ADD )
				|| EditDatas.DoBeforeFinalCopy==MODE_MODIFY )
			{
				/* grid for edit mode and display the rung under edition */
				DrawGrid( cr, LeftRightBarsWidth, ScanY );
				PtrRung = &EditDatas.Rung;
				cInEdit = TRUE;
			}
		}
		DrawRung( cr, PtrRung, LeftRightBarsWidth, ScanY, InfosGene->BlockWidth, InfosGene->BlockHeight, InfosGene->HeaderLabelCommentHeight, DRAW_NORMAL );
		// "alpha" draw background of the current selected element.
		if ( cInEdit && EditDatas.ElementUnderEdit!=NULL )
			DrawCurrentElementEdited( cr, LeftRightBarsWidth, ScanY );
		// ghost zone
		if ( cInEdit && EditDatas.GhostZonePosiX!=-1 && EditDatas.GhostZonePosiY!=-1 && EditDatas.GhostZoneSizeX>0 && EditDatas.GhostZoneSizeY>0 )
			DrawGhostZone( cr, LeftRightBarsWidth, ScanY );

		// if we are adding or inserting a rung, it is as if we have 2 current rungs...
		// If inserting : display edited one, then the current.
		// if adding : display current, then the edited one.
		// Edited one is displayed at the current position (current before... well I hope
		// you still follow, because I'm not sure for me ;-) )
		if ( ! (ScanRung==InfosGene->CurrentRung  && FlagAddOrInsertRung) )
		{
			if ( ScanRung!=InfosGene->LastRung )
				ScanRung = RungArray[ ScanRung ].NextRung;
			else
				TheEnd = TRUE;
		}
		else
		{
			FlagAddOrInsertRung = FALSE;
		}
		DrawRungPartition( cr, LeftRightBarsWidth, ScanY );
		DrawRungPartition( cr, LeftRightBarsWidth, ScanY + TOTAL_PX_RUNG_HEIGHT );
	}
}

void DrawCurrentSection( cairo_t * cr )
{
//Cairo		GdkRectangle update_rect;
	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
	if ( iCurrentLanguage==SECTION_IN_LADDER )
		DrawRungs( cr );
#ifdef SEQUENTIAL_SUPPORT
	if ( iCurrentLanguage==SECTION_IN_SEQUENTIAL )
		DrawSequentialPage( cr, SectionArray[ InfosGene->CurrentSection ].SequentialPage, SEQ_SIZE_DEF, 0 );
#endif
//Cairo	update_rect.x = 0;
//Cairo	update_rect.y = 0;
//Cairo	update_rect.width = GTK_WIDGET(drawing_area)->allocation.width;
//Cairo	update_rect.height = GTK_WIDGET(drawing_area)->allocation.height;
//Cairo	gtk_widget_draw (drawing_area, &update_rect);
}
