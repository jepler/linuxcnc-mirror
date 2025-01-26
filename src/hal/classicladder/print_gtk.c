/* Classic Ladder Project */
/* Copyright (C) 2001-2008 Marc Le Douarain */
/* http://membres.lycos.fr/mavati/classicladder/ */
/* http://www.sourceforge.net/projects/classicladder */
/* August 2008 */
/* ---------------------------------------------------- */
/* Printer output (using Gtk-print and Cairo rendering) */
/* ---------------------------------------------------- */
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
#include <unistd.h>
#include <math.h>
#include <gtk/gtk.h>
#include <pango/pango.h>
#include <libintl.h> // i18n
#include <locale.h> // i18n

#include "classicladder.h"
#include "global.h"
#include "drawing.h"
#include "drawing_sequential.h"
#include "classicladder_gtk.h"

static GtkPrintSettings *settings = NULL;
static int OffsetPrintX = 20;
static int OffsetPrintY = 20;
int NbrRungsPerPage = 0;
int NbrPagesToPrint = 0;
int ScanRungToPrint = 0;
int PrintLadderBlockWidth = BLOCK_WIDTH_DEF;
int PrintLadderBlockHeight = BLOCK_HEIGHT_DEF;
int PrintSeqSize = 32;
int PrintHeaderLabelCommentHeight = BLOCK_HEIGHT_DEF/2;
int SpaceBetweenRungsY = 25;


static void begin_print(GtkPrintOperation *operation, GtkPrintContext   *context, gpointer           user_data)
{
	(void)user_data;
	NbrPagesToPrint = 0;
	int PageWidth = gtk_print_context_get_width(context);
	int PageHeight = gtk_print_context_get_height(context);

	int FinalPageWidth = (PageWidth*75/100) - (OffsetPrintX*2);

	// sizes elements calculation...
	PrintLadderBlockWidth = FinalPageWidth/RUNG_WIDTH;
	PrintLadderBlockHeight = PrintLadderBlockWidth*BLOCK_HEIGHT_DEF/BLOCK_WIDTH_DEF;
	PrintHeaderLabelCommentHeight = PrintLadderBlockWidth/2;
	PrintSeqSize = FinalPageWidth/SEQ_PAGE_WIDTH;
	SpaceBetweenRungsY = PrintLadderBlockHeight*80/100;
	
	// right margin
	PrintRightMarginPosiX = FinalPageWidth + OffsetPrintX;
	PrintRightMarginWidth = PageWidth-FinalPageWidth - (OffsetPrintX*2);

	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
	if ( iCurrentLanguage==SECTION_IN_LADDER )
	{
		int the_width = RUNG_WIDTH*PrintLadderBlockWidth;
		int the_height = PrintHeaderLabelCommentHeight + RUNG_HEIGHT*PrintLadderBlockHeight;

		int NbrTotalRungs = 0;
		GetCurrentNumAndNbrRungsForCurrentSection( NULL, &NbrTotalRungs );
		NbrRungsPerPage = PageHeight/(the_height+SpaceBetweenRungsY);
		NbrPagesToPrint = (NbrTotalRungs+NbrRungsPerPage-1)/NbrRungsPerPage;
printf( "rung page: w=%d, h=%d, PageWidth=%d, NbrTotalRungs=%d NbrRungsPerPage=%d  NbrPagesToPrint=%d\n",the_width,the_height,PageWidth,NbrTotalRungs,NbrRungsPerPage,NbrPagesToPrint );
printf( "print sizes: PrintLadderBlockWidth=%d, PrintLadderBlockHeight=%d, PrintSeqSize=%d\n", PrintLadderBlockWidth, PrintLadderBlockHeight, PrintSeqSize );
		ScanRungToPrint = InfosGene->FirstRung;
	}
#ifdef SEQUENTIAL_SUPPORT
	if ( iCurrentLanguage==SECTION_IN_SEQUENTIAL )
		NbrPagesToPrint = 1;
#endif
	gtk_print_operation_set_n_pages( operation, NbrPagesToPrint );
}

static void draw_page( GtkPrintOperation *operation, GtkPrintContext   *context, int page_nr)
{
	(void)operation;
	cairo_t *cr;
	char Buffer[ 80 ];
//	int the_width = RUNG_WIDTH*bl_width;
//	int the_height = RUNG_HEIGHT*bl_height;
	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;

	cr = gtk_print_context_get_cairo_context( context );

	CreateFontPangoLayout( cr, (iCurrentLanguage==SECTION_IN_SEQUENTIAL)?PrintSeqSize:PrintLadderBlockHeight, DRAW_FOR_PRINT );
	snprintf( Buffer, sizeof(Buffer), CL_PRODUCT_NAME CL_RELEASE_VER_STRING "  -  %s:%s  -  %s:%d/%d", _("Section"), SectionArray[ InfosGene->CurrentSection ].Name, _("Page"),  page_nr+1, NbrPagesToPrint );
	DrawPangoText( cr, OffsetPrintX, gtk_print_context_get_height(context)-10, -1, -1, Buffer );

	if ( iCurrentLanguage==SECTION_IN_LADDER )
	{
		int NumRung = 0;
		int ScanPosiY = OffsetPrintY;
		char TheEnd = FALSE;
		int PrintLeftRightBarsWidth = PrintLadderBlockWidth/16;
		do
		{
			DrawLeftRightBars( cr, OffsetPrintX, ScanPosiY, PrintLadderBlockWidth, PrintLadderBlockHeight, PrintHeaderLabelCommentHeight, PrintLeftRightBarsWidth, FALSE );
			DrawRung( cr, &RungArray[ ScanRungToPrint ], OffsetPrintX+PrintLeftRightBarsWidth, ScanPosiY, PrintLadderBlockWidth, PrintLadderBlockHeight, PrintHeaderLabelCommentHeight, DRAW_FOR_PRINT );
			if ( ScanRungToPrint!=InfosGene->LastRung )
			{
				ScanRungToPrint = RungArray[ ScanRungToPrint ].NextRung;
				int the_height = PrintHeaderLabelCommentHeight + RUNG_HEIGHT*PrintLadderBlockHeight;
				ScanPosiY = ScanPosiY+the_height+SpaceBetweenRungsY;
				NumRung++;
			}
			else
			{
				TheEnd = TRUE;
			}
		}
		while( NumRung<NbrRungsPerPage && !TheEnd );
	}
#ifdef SEQUENTIAL_SUPPORT
	if ( iCurrentLanguage==SECTION_IN_SEQUENTIAL )
		DrawSequentialPage( cr, SectionArray[ InfosGene->CurrentSection ].SequentialPage, PrintSeqSize, DRAW_FOR_PRINT );
#endif
}

static void end_print(GtkPrintOperation *operation, GtkPrintContext   *context, gpointer           user_data)
{
	(void)operation;
	(void)context;
	(void)user_data;
}

void DoPrint( char DoPreview )
{
	GtkPrintOperation *print;
	GtkPrintOperationResult res;
	
	print = gtk_print_operation_new ();
	gtk_print_operation_set_job_name( print, CL_PRODUCT_NAME );
	
	if (settings != NULL) 
		gtk_print_operation_set_print_settings (print, settings);
	
	g_signal_connect (print, "begin_print", G_CALLBACK (begin_print), NULL);
	g_signal_connect (print, "draw_page", G_CALLBACK (draw_page), NULL);
	g_signal_connect (print, "end_print", G_CALLBACK (end_print), NULL);
	
	res = gtk_print_operation_run (print, DoPreview?GTK_PRINT_OPERATION_ACTION_PREVIEW:GTK_PRINT_OPERATION_ACTION_PRINT_DIALOG,
									GTK_WINDOW (MainSectionWindow), NULL);
	
	if (res == GTK_PRINT_OPERATION_RESULT_ERROR)
	{
		ShowMessageBox( _("Print"), _("Failed to print..."), _("Ok") );
	}
	else if (res == GTK_PRINT_OPERATION_RESULT_APPLY)
	{
		if (settings != NULL)
			g_object_unref (settings);
		settings = g_object_ref (gtk_print_operation_get_print_settings (print));
	}
	
	g_object_unref (print);
}

void PrintGtk( )
{
	DoPrint( FALSE/*DoPreview*/ );
}
void PrintPreviewGtk( void )
{
	DoPrint( TRUE/*DoPreview*/ );
}

