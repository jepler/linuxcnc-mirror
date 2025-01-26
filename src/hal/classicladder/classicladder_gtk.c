/* Classic Ladder Project */
/* Copyright (C) 2001-2010 Marc Le Douarain */
/* http://membres.lycos.fr/mavati/classicladder/ */
/* http://www.sourceforge.net/projects/classicladder */
/* February 2001 */
/* ---------------------------- */
/* GTK Interface & Main */
/* Inspired (at the start) from the scribble example. */
/* ---------------------------------- */
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
// Chris Morley (LinuxCNC) Jan 08

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <gtk/gtk.h>
#include <gdk/gdk.h>
#include <cairo.h>
#include <cairo-svg.h>

#include <libintl.h> // i18n
#include <locale.h> // i18n

#include "classicladder.h"
#include "global.h"
#include "classicladder_gtk.h"
#ifdef SEQUENTIAL_SUPPORT
#include "drawing_sequential.h"
#endif
#include "menu_and_toolbar_gtk.h"

#include <rtapi_string.h>

//Cairo GdkPixmap *pixmap = NULL;
GtkWidget *drawing_area = NULL;
GtkWidget *entrylabel,*entrycomment;
GtkWidget *CheckDispSymbols;
#if defined( RT_SUPPORT ) || defined( __XENO__ )
GtkWidget *DurationOfLastScan;
#endif
//Since menu/toolbar... GtkWidget *ButtonRunStop;
GtkWidget *VScrollBar;
GtkWidget *HScrollBar;
GtkAdjustment * AdjustVScrollBar;
GtkAdjustment * AdjustHScrollBar;
GtkWidget *FileSelector;
GtkWidget *ConfirmDialog;
GtkWidget *MainSectionWindow;
GtkWidget *StatusBar;
GtkWidget *dialog,*label, *okay_button;
gint StatusBarContextId;

#include "drawing.h"
#include "vars_access.h"
#include "calc.h"
#include "files_project.h"
#include "edit.h"
#include "edit_gtk.h"
#include "editproperties_gtk.h"
#include "manager_gtk.h"
#include "config_gtk.h"
#include "socket_server.h"
#include "socket_modbus_master.h"
#ifdef SEQUENTIAL_SUPPORT
#include "calc_sequential.h"
#endif
//#ifdef GNOME_PRINT_USE
//#include "print_gnome.h"
//#endif
#include "symbols_gtk.h"
#include "spy_vars_gtk.h"
#include "print_gtk.h"
//#include "vars_system.h"

void CairoDrawCurrentSectionOnDrawingArea( cairo_t *cr )
{
	/* clean up */
	double w,h;
//ForGTK3
#if GTK_MAJOR_VERSION>=3
	w = gtk_widget_get_allocated_width( drawing_area );
	h = gtk_widget_get_allocated_height( drawing_area );
#else
	w = drawing_area->allocation.width;
	h = drawing_area->allocation.height;
#endif
	cairo_set_source_rgb( cr, 1, 1 ,1 );
	cairo_rectangle( cr, 0.0, 0.0, w, h );
	cairo_fill( cr );
	GetTheSizesForRung( );
	DrawCurrentSection( cr );
}

/* Create a new backing pixmap of the appropriate size */
/*static gint configure_event( GtkWidget         *widget,
                            GdkEventConfigure *event )
{
	if (pixmap)
		gdk_pixmap_unref(pixmap);

	pixmap = gdk_pixmap_new(widget->window,
							widget->allocation.width,
							widget->allocation.height,
							-1);
	gdk_draw_rectangle (pixmap,
						widget->style->white_gc,
						TRUE,
						0, 0,
						widget->allocation.width,
						widget->allocation.height);
	return TRUE;
}*/
/* Redraw the screen with Cairo */
#if GTK_MAJOR_VERSION>=3
void draw_callback( GtkWidget *widget, cairo_t *cr, gpointer data)
{
	(void)widget;
	(void)data;
	CairoDrawCurrentSectionOnDrawingArea( cr );
}
#else
static gint expose_event( GtkWidget	  *widget,
						GdkEventExpose *event )
{
/*	gdk_draw_pixmap(widget->window,
					widget->style->fg_gc[GTK_WIDGET_STATE (widget)],
					pixmap,
					event->area.x, event->area.y,
					event->area.x, event->area.y,
					event->area.width, event->area.height);
*/
	cairo_t *cr = gdk_cairo_create( drawing_area->window );
	CairoDrawCurrentSectionOnDrawingArea( cr );
	cairo_destroy( cr );
	return FALSE;
}
#endif

void GetTheSizesForRung( void )
{
	static int PageHeightBak = 0;
	static int BlockHeightBak = 0;
//ForGTK3
#if GTK_MAJOR_VERSION>=3
	InfosGene->BlockWidth = ((gtk_widget_get_allocated_width( drawing_area )*995/1000) / RUNG_WIDTH);
#else
	InfosGene->BlockWidth = ((GTK_WIDGET(drawing_area)->allocation.width*995/1000) / RUNG_WIDTH);
#endif
	// keep ratio aspect (if defaults values of size block not square)
	InfosGene->BlockHeight = InfosGene->BlockWidth*BLOCK_HEIGHT_DEF/BLOCK_WIDTH_DEF;

//ForGTK3
#if GTK_MAJOR_VERSION>=3
	InfosGene->PageHeight = gtk_widget_get_allocated_height( drawing_area );
#else
	InfosGene->PageHeight = GTK_WIDGET(drawing_area)->allocation.height;
#endif
	// used for sequential
//ForGTK3
#if GTK_MAJOR_VERSION>=3
	InfosGene->PageWidth = gtk_widget_get_allocated_width( drawing_area );
#else
	InfosGene->PageWidth = GTK_WIDGET(drawing_area)->allocation.width;
#endif

	// size of the page or block changed ?
	if ( InfosGene->PageHeight!=PageHeightBak || InfosGene->BlockHeight!=BlockHeightBak )
		UpdateVScrollBar( /*AutoSelectCurrentRung*/ );
	PageHeightBak = InfosGene->PageHeight;
	BlockHeightBak = InfosGene->BlockHeight;
}

// calc total nbr rungs in a section, and nbr of rungs before current rung.
void GetCurrentNumAndNbrRungsForCurrentSection( int * pCurrNumRung, int * pNbrRungs )
{
	int iSecurityBreak = 0;
	int NbrRungs = 1;
	int ScanRung = InfosGene->FirstRung;
	int NumCurrentRung = 0;
	while ( ScanRung!=InfosGene->LastRung && iSecurityBreak++<=NBR_RUNGS )
	{
		NbrRungs++;
		ScanRung = RungArray[ ScanRung ].NextRung;
	}
	ScanRung = InfosGene->FirstRung;
	iSecurityBreak = 0;
	while ( ScanRung!=InfosGene->CurrentRung && iSecurityBreak++<=NBR_RUNGS )
	{
		NumCurrentRung++;
		ScanRung = RungArray[ ScanRung ].NextRung;
	}
	if ( iSecurityBreak>=NBR_RUNGS )
		debug_printf("!!!error loop in UpdateVScrollBar()!\n");
	if ( pCurrNumRung!=NULL )
		*pCurrNumRung = NumCurrentRung;
	if ( pNbrRungs!=NULL )
		*pNbrRungs = NbrRungs;
}

void UpdateVScrollBar()
{
	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
	if ( iCurrentLanguage==SECTION_IN_LADDER )
	{
		int NbrRungs = 1;
		int NumCurrentRung = 0;
		int AdjustUpper;
		int AdjustValue;
		GetCurrentNumAndNbrRungsForCurrentSection( &NumCurrentRung, &NbrRungs );
//printf("Nbr rungs=%d , NumRung=%d\n", NbrRungs, NumCurrentRung);
		AdjustUpper = NbrRungs * TOTAL_PX_RUNG_HEIGHT;
		AdjustValue = NumCurrentRung *  TOTAL_PX_RUNG_HEIGHT;
		gtk_adjustment_set_lower( AdjustVScrollBar, 0 );
		gtk_adjustment_set_upper( AdjustVScrollBar, AdjustUpper );
		// go up lift to take into account the fact that total height page is generally more than just one rung...
		while( AdjustValue+InfosGene->PageHeight > AdjustUpper )
		{
			AdjustValue = AdjustValue - TOTAL_PX_RUNG_HEIGHT;
		}
		gtk_adjustment_set_value( AdjustVScrollBar, AdjustValue );
		gtk_adjustment_set_step_increment( AdjustVScrollBar, InfosGene->BlockHeight );
		gtk_adjustment_set_page_increment( AdjustVScrollBar, TOTAL_PX_RUNG_HEIGHT );
		gtk_adjustment_set_page_size( AdjustVScrollBar, InfosGene->PageHeight );
		gtk_adjustment_changed( AdjustVScrollBar );
		gtk_adjustment_value_changed( AdjustVScrollBar );
		gtk_widget_hide( HScrollBar );
//        gtk_widget_show( entrylabel );
//        gtk_widget_show( entrycomment );
	}
#ifdef SEQUENTIAL_SUPPORT
	if ( iCurrentLanguage==SECTION_IN_SEQUENTIAL )
	{
		gtk_widget_show( HScrollBar );
//        gtk_widget_hide( entrylabel );
//        gtk_widget_hide( entrycomment );
		refresh_label_comment( );
		gtk_adjustment_set_lower( AdjustVScrollBar, 0 );
		gtk_adjustment_set_upper( AdjustVScrollBar, SEQ_PAGE_HEIGHT * SEQ_SIZE_DEF );
		gtk_adjustment_set_value( AdjustVScrollBar, 0 );
		gtk_adjustment_set_step_increment( AdjustVScrollBar, SEQ_SIZE_DEF );
		gtk_adjustment_set_page_increment( AdjustVScrollBar, InfosGene->PageHeight );
		gtk_adjustment_set_page_size( AdjustVScrollBar, InfosGene->PageHeight );
		gtk_adjustment_changed( AdjustVScrollBar );
		gtk_adjustment_value_changed( AdjustVScrollBar );
		gtk_adjustment_set_lower( AdjustHScrollBar, 0 );
		gtk_adjustment_set_upper( AdjustHScrollBar, SEQ_PAGE_WIDTH * SEQ_SIZE_DEF );
		gtk_adjustment_set_value( AdjustHScrollBar, 0 );
		gtk_adjustment_set_step_increment( AdjustHScrollBar, SEQ_SIZE_DEF );
		gtk_adjustment_set_page_increment( AdjustHScrollBar, InfosGene->PageWidth );
		gtk_adjustment_set_page_size( AdjustHScrollBar, InfosGene->PageWidth );
		gtk_adjustment_changed( AdjustHScrollBar );
		gtk_adjustment_value_changed( AdjustHScrollBar );
	}
#endif
}

void ChoiceOfTheCurrentRung( int NbrOfRungsAfterTopRung )
{
	int DecptNbrRungs = NbrOfRungsAfterTopRung;

//printf("OffsetHiddenTopRungDisplayed=%d\n", InfosGene->OffsetHiddenTopRungDisplayed);
	// per default, the current rung, is the top one displayed...
	InfosGene->CurrentRung = InfosGene->TopRungDisplayed;
	// if OffsetHiddenTopRungDisplayed==0, vertical shift of the current rung is 0,
	// else this is the y value to substract to have the vertical shift... (we will add many full rungs heights after!)
	InfosGene->OffsetCurrentRungDisplayed = -1*InfosGene->OffsetHiddenTopRungDisplayed;

//printf("=> In choice of current rung, %d passes...(CurrentRung=%d)\n",NbrOfRungsAfterTopRung, InfosGene->CurrentRung);
	while( DecptNbrRungs>0 && InfosGene->CurrentRung!=InfosGene->LastRung )
	{
		InfosGene->CurrentRung = RungArray[ InfosGene->CurrentRung ].NextRung;
		InfosGene->OffsetCurrentRungDisplayed += TOTAL_PX_RUNG_HEIGHT;
		DecptNbrRungs--;
	}

//printf("=> In %s, CurrentRung=%d , NbrOfRungsAfterTopRung=%d, OffsetCurrentRungDisplayed=%d\n", __FUNCTION__, InfosGene->CurrentRung, NbrOfRungsFnd, InfosGene->OffsetCurrentRungDisplayed);
//////test	if ( InfosGene->OffsetCurrentRungDisplayed<0 )
//////test		debug_printf( "Error in ChoiceOfTheCurrentRung( %d ) with OffsetCurrentRungDisplayed=%d\n", NbrOfRungsFnd, InfosGene->OffsetCurrentRungDisplayed );
	refresh_label_comment( );
}

static gint VScrollBar_value_changed_event( GtkAdjustment * ScrollBar, void * not_used )
{
	(void)not_used;
	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
	if ( iCurrentLanguage==SECTION_IN_LADDER )
	{
		int NumRung = ((int)gtk_adjustment_get_value( ScrollBar ))/TOTAL_PX_RUNG_HEIGHT;
		int ScanRung = 0;
		InfosGene->TopRungDisplayed = InfosGene->FirstRung;
		if ( NumRung<0 )
			NumRung = 0;
		while( ScanRung!=NumRung )
		{
			InfosGene->TopRungDisplayed = RungArray[ InfosGene->TopRungDisplayed ].NextRung;
			ScanRung++;
		}
		InfosGene->OffsetHiddenTopRungDisplayed = ((int)gtk_adjustment_get_value( ScrollBar ))%TOTAL_PX_RUNG_HEIGHT;

		// if top rung displayed entirely (no vertical offset), it's the current rung => give '0'.
		// else, search the next one => give '1'.
		ChoiceOfTheCurrentRung( (InfosGene->OffsetHiddenTopRungDisplayed>0)?1:0 );
	}
	InfosGene->VScrollValue = (int)gtk_adjustment_get_value( ScrollBar );
	return TRUE;
}
static gint HScrollBar_value_changed_event( GtkAdjustment * ScrollBar, void * not_used )
{
	(void)not_used;
	InfosGene->HScrollValue = (int)gtk_adjustment_get_value( ScrollBar );
	return TRUE;
}

// function called for keys up/down and mouse scroll with increment height (positive or negative)...
// if increment=0, just update display with new value modified before.
static void IncrementVScrollBar( int IncrementValue )
{
//printf("%s(): inc=%d\n", __FUNCTION__, IncrementValue );
	if ( IncrementValue!=0 )
	{
		gtk_adjustment_set_value( AdjustVScrollBar, gtk_adjustment_get_value(AdjustVScrollBar)+IncrementValue );
		if ( IncrementValue>0 )
		{
			if ( gtk_adjustment_get_value(AdjustVScrollBar) > (gtk_adjustment_get_upper(AdjustVScrollBar)-InfosGene->PageHeight) )
				gtk_adjustment_set_value( AdjustVScrollBar, gtk_adjustment_get_upper(AdjustVScrollBar)-InfosGene->PageHeight );
		}
		else
		{
			if ( gtk_adjustment_get_value(AdjustVScrollBar) < gtk_adjustment_get_lower(AdjustVScrollBar) )
				gtk_adjustment_set_value( AdjustVScrollBar, gtk_adjustment_get_lower(AdjustVScrollBar) );
		}
	}
	gtk_adjustment_changed( AdjustVScrollBar );
	InfosGene->OffsetHiddenTopRungDisplayed	= gtk_adjustment_get_value(AdjustVScrollBar);
	VScrollBar_value_changed_event( AdjustVScrollBar, 0 );
}

static gboolean mouse_scroll_event( GtkWidget *widget, GdkEventScroll *event )
{
	(void)widget;
	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
	if ( iCurrentLanguage==SECTION_IN_LADDER )
	{
		if (event->direction == GDK_SCROLL_DOWN)
			IncrementVScrollBar( gtk_adjustment_get_step_increment(AdjustVScrollBar) );
		else  if (event->direction == GDK_SCROLL_UP )
			IncrementVScrollBar( -1*gtk_adjustment_get_step_increment(AdjustVScrollBar) );
	}
	return TRUE;
}
static gboolean button_press_event( GtkWidget *widget, GdkEventButton *event )
{
	(void)widget;
	if (event->button == 1 /*Cairo && pixmap != NULL*/)
	{
		if (EditDatas.ModeEdit)
		{
			EditElementInThePage(event->x,event->y);
		}
		else
		{
			// we can select the current rung by clicking on one.
			// the current rung is the one that will be modified...
			int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
			if ( iCurrentLanguage==SECTION_IN_LADDER )
			{
				char DoSelection = TRUE;
				if ( InfosGene->OffsetHiddenTopRungDisplayed>0 )
				{
					if ( event->y<TOTAL_PX_RUNG_HEIGHT-InfosGene->OffsetHiddenTopRungDisplayed )
						DoSelection = FALSE;
				}
				if ( DoSelection )
				{
					int NbrRungsShift =  (event->y+InfosGene->OffsetHiddenTopRungDisplayed)/TOTAL_PX_RUNG_HEIGHT;
//printf("Select the current rung, with a shift of rungs=%d\n", NbrRungsShift );
					ChoiceOfTheCurrentRung( NbrRungsShift );
//DisplayedNowDirectlyOnMotion					MessageInStatusBar( GetLadderElePropertiesForStatusBar( event->x,event->y ) );
				}
			}
		}
	}
	return TRUE;
}
static gboolean motion_notify_event( GtkWidget *widget, GdkEventMotion *event, gpointer user_data )
{
	(void)widget;
	(void)user_data;
	//v0.9.20
	if( SectionArray==NULL )
		return TRUE;
		
	if (EditDatas.ModeEdit)
	{
		MouseMotionOnThePage( event->x, event->y );
	}
	else
	{
		if ( SectionArray[ InfosGene->CurrentSection ].Language==SECTION_IN_LADDER )
		{
			char * pLadderProperties = GetLadderElePropertiesForStatusBar( event->x,event->y );
			if ( pLadderProperties )
				MessageInStatusBar( pLadderProperties );
		}
	}
	return TRUE;
}
static gboolean button_release_event( GtkWidget *widget, GdkEventButton *event )
{
	(void)widget;
	if (event->button == 1 /*Cairo && pixmap != NULL*/)
	{
		if (EditDatas.ModeEdit)
			EditButtonReleaseEventOnThePage( );
	}
	return TRUE;
}
void refresh_label_comment( void )
{
	StrRung * RfhRung;
	if ( SectionArray[ InfosGene->CurrentSection ].Language==SECTION_IN_LADDER )
	{
		RfhRung = &RungArray[InfosGene->CurrentRung];
		gtk_entry_set_text((GtkEntry *)entrylabel,RfhRung->Label);
		gtk_entry_set_text((GtkEntry *)entrycomment,RfhRung->Comment);
	}
	else
	{
		gtk_entry_set_text((GtkEntry *)entrylabel,"");
		gtk_entry_set_text((GtkEntry *)entrycomment,"");
	}
}
void clear_label_comment()
{
	gtk_entry_set_text((GtkEntry *)entrylabel,"");
	gtk_entry_set_text((GtkEntry *)entrycomment,"");
}

void save_label_comment_edited()
{
	rtapi_strxcpy(EditDatas.Rung.Label,gtk_entry_get_text((GtkEntry *)entrylabel));
	rtapi_strxcpy(EditDatas.Rung.Comment,gtk_entry_get_text((GtkEntry *)entrycomment));
}

void autorize_prevnext_buttons(int Yes)
{
	if (Yes)
	{
		gtk_widget_set_sensitive(VScrollBar, TRUE);
		gtk_widget_set_sensitive(entrylabel, FALSE);
		gtk_widget_set_sensitive(entrycomment, FALSE);
	}
	else
	{
		gtk_widget_set_sensitive(VScrollBar, FALSE);
		gtk_widget_set_sensitive(entrylabel, TRUE);
		gtk_widget_set_sensitive(entrycomment, TRUE);
	}
}

void CheckDispSymbols_toggled( )
{
	InfosGene->DisplaySymbols = gtk_toggle_button_get_active( GTK_TOGGLE_BUTTON( CheckDispSymbols ) );
}


void StoreDirectorySelected( GtkFileChooser *selector, char cForLoadingProject)
{
	char * TempDir;
	(void)selector;

	TempDir = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER(FileSelector));

    if ( cForLoadingProject )
        VerifyDirectorySelected( TempDir );
    else
        rtapi_strxcpy( InfosGene->CurrentProjectFileName, TempDir );
}


void LoadNewLadder()
{
	char ProjectLoadedOk;
	StoreDirectorySelected( GTK_FILE_CHOOSER(FileSelector) , TRUE/*cForLoadingProject*/);

    if (InfosGene->LadderState==STATE_RUN)
    {
        DoFlipFlopRunStop( );
	}
    InfosGene->LadderState = STATE_LOADING;
	ProjectLoadedOk = LoadProjectFiles( InfosGene->CurrentProjectFileName );
	if ( !ProjectLoadedOk )
		ShowMessageBox( _("Load Error"), _("Failed to load the project file..."), _("Ok") );

	UpdateAllGtkWindows( );
	UpdateWindowTitleWithProjectName( );
	MessageInStatusBar( ProjectLoadedOk?_("Project loaded (stopped)."):_("Project failed to load..."));
#ifndef RT_SUPPORT
        OpenHardware( 0 );
//        ConfigHardware( );
	InfosGene->AskToConfHard = TRUE;
#endif
    InfosGene->LadderState = STATE_STOP;
}
void DoActionSave()
{
	if ( !SaveProjectFiles( InfosGene->CurrentProjectFileName ) )
		ShowMessageBox( _("Save Error"), _("Failed to save the project file..."), _("Ok") );
}

void SaveAsLadder(void)
{
	StoreDirectorySelected( GTK_FILE_CHOOSER(FileSelector), FALSE/*cForLoadingProject*/);

	if ( !SaveProjectFiles( InfosGene->CurrentProjectFileName ) )
		ShowMessageBox( _("Save Error"), _("Failed to save the project file..."), _("Ok") );
        UpdateWindowTitleWithProjectName( );
}

void on_filechooserdialog_save_response(GtkDialog  *dialog,gint response_id,gpointer user_data)
{
	(void)user_data;
	debug_printf(_("SAVE %s %d\n"),gtk_file_chooser_get_filename (GTK_FILE_CHOOSER(FileSelector)),response_id);

	if(response_id==GTK_RESPONSE_ACCEPT || response_id==GTK_RESPONSE_OK)
		SaveAsLadder();
	gtk_widget_destroy(GTK_WIDGET(dialog));
}
void on_filechooserdialog_load_response(GtkDialog  *dialog,gint response_id,gpointer user_data)
{
	(void)user_data;
	debug_printf(_("LOAD %s %d\n"),gtk_file_chooser_get_filename (GTK_FILE_CHOOSER(FileSelector)),response_id);

	if(response_id==GTK_RESPONSE_ACCEPT || response_id==GTK_RESPONSE_OK)
		LoadNewLadder();
	gtk_widget_destroy(GTK_WIDGET(dialog));
}


void CreateFileSelection(char * Prompt,int CreateFileSelectionType)
{
	/* Create the selector */
	GtkFileFilter *FilterOldProjects, *FilterProjects;
	if(CreateFileSelectionType==CREATE_FILE_SELECTION_TO_SAVE_PROJECT)
	{
		FileSelector = gtk_file_chooser_dialog_new (Prompt, NULL, GTK_FILE_CHOOSER_ACTION_SAVE,
			GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
			GTK_STOCK_SAVE, GTK_RESPONSE_ACCEPT, NULL);
		gtk_file_chooser_set_do_overwrite_confirmation( GTK_FILE_CHOOSER(FileSelector), TRUE );
	}
	else
	{
		FileSelector = gtk_file_chooser_dialog_new (Prompt, NULL, GTK_FILE_CHOOSER_ACTION_OPEN,
			GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
			GTK_STOCK_OPEN, GTK_RESPONSE_ACCEPT, NULL);
	}
	gtk_window_set_type_hint (GTK_WINDOW (FileSelector), GDK_WINDOW_TYPE_HINT_DIALOG);

	FilterOldProjects = gtk_file_filter_new( );
	gtk_file_filter_set_name( FilterOldProjects, _("Old directories projects") );
	gtk_file_filter_add_pattern( FilterOldProjects, "*.csv" ); // old dir projects
	gtk_file_chooser_add_filter( GTK_FILE_CHOOSER(FileSelector), FilterOldProjects );
	FilterProjects = gtk_file_filter_new( );
	gtk_file_filter_set_name( FilterProjects, _("ClassicLadder projects") );
        //XXX Upstream has added two new supported formats, for simplicity
        // we will continue to only use the old one.
	//gtk_file_filter_add_pattern( FilterProjects, "*.clprj" );
	gtk_file_filter_add_pattern( FilterProjects, "*.clp" );
	//gtk_file_filter_add_pattern( FilterProjects, "*.clprjz" );
	gtk_file_chooser_add_filter( GTK_FILE_CHOOSER(FileSelector), FilterProjects );
	gtk_file_chooser_set_filter( GTK_FILE_CHOOSER(FileSelector), FilterProjects );

	gtk_window_set_modal(GTK_WINDOW(FileSelector), TRUE );

/*
  g_signal_connect ((gpointer) filechooserdialog, "file_activated",
					G_CALLBACK (on_filechooserdialog_file_activated),
					NULL);
					*/

	if( CreateFileSelectionType==CREATE_FILE_SELECTION_TO_SAVE_PROJECT )
		g_signal_connect ((gpointer) FileSelector, "response",
					G_CALLBACK (on_filechooserdialog_save_response),
					NULL);
	else
		g_signal_connect ((gpointer) FileSelector, "response",
					G_CALLBACK (on_filechooserdialog_load_response),
					NULL);

	g_signal_connect_swapped ((gpointer) FileSelector, "close",
							G_CALLBACK (gtk_widget_destroy),
							GTK_OBJECT (FileSelector));

	/* Display that dialog */
	gtk_widget_show (FileSelector);
}

void DoNewProject( void )
{
	ClassicLadder_InitAllDatas( );
	UpdateAllGtkWindows( );
	InfosGene->AskConfirmationToQuit = TRUE;
	InfosGene->HasBeenModifiedForExitCode = TRUE;
}

void DoActionConfirmNewProject()
{
	ShowConfirmationBox(_("New"),_("Do you really want to clear all data ?"),DoNewProject);
}
void DoLoadProject()
{
	CreateFileSelection(_("Please select the project to load"),FALSE);
}

void DoActionLoadProject()
{
	if ( InfosGene->AskConfirmationToQuit )
		ShowConfirmationBox( _("Sure?"), _("Do you really want to load another project ?\nIf not saved, all modifications on the current project will be lost  \n"), DoLoadProject );
	else
		DoLoadProject( );
}

void DoActionSaveAs()
{
	CreateFileSelection(_("Please select the project to save"),TRUE);
}

void DoActionResetAndConfirmIfRunning( )
{
	if (InfosGene->LadderState==STATE_RUN)
		ShowConfirmationBox(_("Warning!"),_("Resetting a running program\ncan cause unexpected behavior\n Do you really want to reset?"),DoReset);
	else
		DoReset();
}
void DoReset()
{
//////	int StateBefore = InfosGene->LadderState;
//////	InfosGene->LadderState = STATE_STOP;
//////	// wait, to be sure calcs have ended...
//////	usleep( 100000 );
	StopRunIfRunning( );

	InitVars();
	//InitSystemVars( FALSE );
	PrepareAllDatasBeforeRun( );

//////	if ( StateBefore==STATE_RUN )
//////		InfosGene->LadderState = STATE_RUN;
	RunBackIfStopped( );
	MessageInStatusBar((InfosGene->LadderState==STATE_RUN)?_("Reset logic data - Now running."):_("Reset logic data done."));
}

//void ButtonConfig_click()
//{
//    OpenConfigWindowGtk( );
//}

void DoActionAboutClassicLadder()
{
	/*
	// From the example in gtkdialog help
	GtkWidget *dialog, *label, *okay_button;
	// Create the widgets
	dialog = gtk_dialog_new();
	label = gtk_label_new ( CL_PRODUCT_NAME " v" CL_RELEASE_VER_STRING "\n" CL_RELEASE_DATE_STRING "\n"
						"Copyright (C) " CL_RELEASE_COPYRIGHT_YEARS " Marc Le Douarain\nmarc . le - douarain /At\\ laposte \\DoT/ net\n"
						"http://www.sourceforge.net/projects/classicladder\n"
						"https://github.com/MaVaTi56/classicladder\n"
						"Released under the terms of the\nGNU Lesser General Public License v2.1\n"
						"\nAs adapted to LinuxCNC\n"
						"(Chris Morley)\n"
						"emc-users@lists.sourceforge.net");

	gtk_label_set_justify( GTK_LABEL(label), GTK_JUSTIFY_CENTER );
	okay_button = gtk_button_new_with_label("Okay");
	// Ensure that the dialog box is destroyed when the user clicks ok.
	gtk_signal_connect_object (GTK_OBJECT (okay_button), "clicked",
							GTK_SIGNAL_FUNC (gtk_widget_destroy), GTK_OBJECT(dialog));
	gtk_container_add (GTK_CONTAINER (GTK_DIALOG(dialog)->action_area),
					okay_button);
	gtk_widget_grab_focus(okay_button);
	// Add the label, and show everything we've added to the dialog.
	gtk_container_add (GTK_CONTAINER (GTK_DIALOG(dialog)->vbox),
					label);
	gtk_window_set_modal(GTK_WINDOW(dialog),TRUE);
	gtk_window_set_position(GTK_WINDOW(dialog),GTK_WIN_POS_CENTER);
	gtk_widget_show_all (dialog);
*/
	char * static_comments = "Released under the terms of the\nGNU Lesser General Public License v2.1\n\n"
							"Latest software version at:\nhttp://www.sourceforge.net/projects/classicladder\n"
							"Original site:\nhttp://membres.lycos.fr/mavati/classicladder"
                            "\nAs adapted to LinuxCNC\n"
                            "(Chris Morley)\n"
                            "emc-users@lists.sourceforge.net";
	char * comments = malloc( strlen( static_comments )+80 );
	if ( comments )
	{
		char GtkVersionString[ 30 ];
		snprintf( GtkVersionString, sizeof(GtkVersionString), "GTK+ version %d.%d.%d\n\n", gtk_major_version, gtk_minor_version, gtk_micro_version );
		strcpy( comments, "("CL_RELEASE_DATE_STRING")\n\n" );
		strcat( comments, GtkVersionString );
		strcat( comments, static_comments );
		gtk_show_about_dialog ( GTK_WINDOW( MainSectionWindow ),
							"program-name", CL_PRODUCT_NAME ,
							"version", CL_RELEASE_VER_STRING ,
							"copyright", "Copyright (C) " CL_RELEASE_COPYRIGHT_YEARS " Marc Le Douarain\nmarc . le - douarain /At\\ laposte \\DoT/ net" ,
//							"logo", example_logo,
							"title", _("About ClassicLadder"),
							"website", "http://sites.google.com/site/classicladder" ,
							"comments", comments ,
                       NULL );
		free( comments );
	}
}


cairo_surface_t *ExportSurface;
cairo_t * InitExportSurface( int SurfaceWidth, int SurfaceHeight, char * SvgFileToCreate )
{
	cairo_t *cr;
	if ( SvgFileToCreate )
	{
		ExportSurface = cairo_svg_surface_create( SvgFileToCreate, SurfaceWidth, SurfaceHeight );
		cr = cairo_create( ExportSurface );
	}
	else
	{
		ExportSurface = cairo_image_surface_create( CAIRO_FORMAT_RGB24, SurfaceWidth, SurfaceHeight );
		cr = cairo_create( ExportSurface );
		//cleanup
		cairo_set_source_rgb( cr, 1.0, 1.0, 1.0 );
		cairo_paint( cr );
	}
	return cr;	
}

void ExportSvgOrPngFile( char * FileToCreate, char GoForSvgExport )
{
	cairo_t *cr = NULL;
	int iCurrentLanguage = SectionArray[ InfosGene->CurrentSection ].Language;
	if ( iCurrentLanguage==SECTION_IN_LADDER )
	{
		int SvgWidthTotal = RUNG_WIDTH*BLOCK_WIDTH_DEF+30;
		int SvgHeightTotal = RUNG_HEIGHT*BLOCK_HEIGHT_DEF+BLOCK_HEIGHT_DEF/2;
		int LeftRightBarsWidth = BLOCK_WIDTH_DEF/16;
		cr = InitExportSurface( SvgWidthTotal, SvgHeightTotal, GoForSvgExport?FileToCreate:NULL );
		DrawLeftRightBars( cr, 0, 0, BLOCK_WIDTH_DEF, BLOCK_HEIGHT_DEF, BLOCK_HEIGHT_DEF/2, LeftRightBarsWidth, FALSE );
		DrawRung( cr, &RungArray[ InfosGene->CurrentRung ], LeftRightBarsWidth, 0, BLOCK_WIDTH_DEF, BLOCK_HEIGHT_DEF, BLOCK_HEIGHT_DEF/2, DRAW_FOR_PRINT );
	}
	if ( iCurrentLanguage==SECTION_IN_SEQUENTIAL )
	{
		int SvgWidthTotal = SEQ_PAGE_WIDTH*SEQ_SIZE_DEF;
		int SvgHeightTotal = SEQ_PAGE_HEIGHT*SEQ_SIZE_DEF;
		cr = InitExportSurface(  SvgWidthTotal, SvgHeightTotal, GoForSvgExport?FileToCreate:NULL );
		DrawSequentialPage( cr, SectionArray[ InfosGene->CurrentSection ].SequentialPage, SEQ_SIZE_DEF, DRAW_FOR_PRINT );
	}
	if ( !GoForSvgExport )
		cairo_surface_write_to_png( ExportSurface, FileToCreate );
	cairo_surface_destroy( ExportSurface );
	cairo_destroy( cr );
}
void FileRequestToExportSvgOrPng(char GoForSvg)
{
	GtkWidget *dialog;
	dialog = gtk_file_chooser_dialog_new (GoForSvg?_("Save SVG File"):_("Save PNG File"),
										GTK_WINDOW(MainSectionWindow),
										GTK_FILE_CHOOSER_ACTION_SAVE,
										GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
										GTK_STOCK_SAVE, GTK_RESPONSE_ACCEPT,
										NULL);
	gtk_file_chooser_set_do_overwrite_confirmation( GTK_FILE_CHOOSER (dialog), TRUE );
	gtk_file_chooser_set_current_name( GTK_FILE_CHOOSER (dialog), GoForSvg?"classicladder_export.svg":"classicladder_export.png" );
	if ( gtk_dialog_run( GTK_DIALOG (dialog) ) == GTK_RESPONSE_ACCEPT )
	{
		char *filename;
		filename = gtk_file_chooser_get_filename( GTK_FILE_CHOOSER(dialog) );
		ExportSvgOrPngFile( filename, GoForSvg );
		g_free( filename );
	}
	gtk_widget_destroy( dialog );
}
void DoActionExportSvg( void )
{
	FileRequestToExportSvgOrPng( TRUE );
}
void DoActionExportPng( void )
{
	FileRequestToExportSvgOrPng( FALSE );
}
// use a temp .png file to make it... (Cairo Surface -> png file -> pixbuf -> clipboard)
void DoActionCopyToClipboard( void )
{
	GError * err = NULL;
	GtkClipboard * pClipboard = gtk_clipboard_get( GDK_SELECTION_CLIPBOARD );
	ExportSvgOrPngFile( "cl_clipboard_tmp.png", FALSE/*GoForSvgExport*/ );
printf("Creating gdk pixpuf from tmp png file\n");
	GdkPixbuf * pPixBuf = gdk_pixbuf_new_from_file( "cl_clipboard_tmp.png", &err );
	if ( pPixBuf )
	{
		remove( "cl_clipboard_tmp.png" );
printf("Set pixbuf image to clipboard\n");
		gtk_clipboard_set_image( pClipboard, pPixBuf );
		g_object_unref( pPixBuf );
	}
	else
	{
		printf("Error clipboard_set_image() : %s\n", err->message);
	}
}


void ShowMessageBox(const char * title, const char * text, const char * button)
{
	/* From the example in gtkdialog help */
	GtkWidget *dialog, *label, *okay_button;
	/* Create the widgets */
	dialog = gtk_dialog_new();
	label = gtk_label_new (text);
	okay_button = gtk_button_new_with_label(button);
	/* Ensure that the dialog box is destroyed when the user clicks ok. */
//ForGTK3	gtk_signal_connect_object (GTK_OBJECT (okay_button), "clicked",
//ForGTK3							GTK_SIGNAL_FUNC (gtk_widget_destroy), GTK_OBJECT(dialog));
	g_signal_connect_swapped(GTK_OBJECT (okay_button), "clicked",
							GTK_SIGNAL_FUNC (gtk_widget_destroy), GTK_OBJECT(dialog));
//ForGTK3	gtk_container_add (GTK_CONTAINER (GTK_DIALOG(dialog)->action_area), okay_button);
	gtk_container_add (GTK_CONTAINER (gtk_dialog_get_action_area(GTK_DIALOG(dialog))), okay_button);
	gtk_widget_grab_focus(okay_button);
	/* Add the label, and show everything we've added to the dialog. */
//ForGTK3	gtk_container_add (GTK_CONTAINER (GTK_DIALOG(dialog)->vbox), label);
	gtk_container_add (GTK_CONTAINER (gtk_dialog_get_content_area(GTK_DIALOG(dialog))), label);
	gtk_window_set_modal(GTK_WINDOW(dialog),TRUE);
	gtk_window_set_title(GTK_WINDOW(dialog),title);
	gtk_window_set_position(GTK_WINDOW(dialog),GTK_WIN_POS_CENTER);
	gtk_widget_show_all (dialog);
}
void ShowMessageBoxError( const char * text )
{
	ShowMessageBox( _("Error"), text, _("Ok") );
}

void DoFunctionOfConfirmationBox( GtkWidget *widget, void * (*function_to_do)(void *) )
{
	(void)widget;
	gtk_widget_destroy(ConfirmDialog);
	(function_to_do)(NULL);
}
void ShowConfirmationBoxWithChoiceOrNot(const char * title,const char * text,void * function_if_yes, char HaveTheChoice)
{
	/* From the example in gtkdialog help */
	GtkWidget *label, *yes_button, *no_button;
	/* Create the widgets */
	ConfirmDialog = gtk_dialog_new();
	label = gtk_label_new (text);
	if ( HaveTheChoice )
	{
		yes_button = gtk_button_new_with_label(_("Yes"));
		no_button = gtk_button_new_with_label(_("No"));
	}
	else
	{
		yes_button = gtk_button_new_with_label(_("Ok"));
	}
	/* Ensure that the dialog box is destroyed when the user clicks ok. */
	if ( HaveTheChoice )
	{
//ForGTK3		gtk_signal_connect_object (GTK_OBJECT (no_button), "clicked",
//ForGTK3							GTK_SIGNAL_FUNC (gtk_widget_destroy), GTK_OBJECT(ConfirmDialog));
		g_signal_connect_swapped(GTK_OBJECT (no_button), "clicked",
							GTK_SIGNAL_FUNC (gtk_widget_destroy), GTK_OBJECT(ConfirmDialog));
//ForGTK3		gtk_container_add (GTK_CONTAINER (GTK_DIALOG(ConfirmDialog)->action_area), no_button);
		gtk_container_add (GTK_CONTAINER (gtk_dialog_get_action_area(GTK_DIALOG(ConfirmDialog))), no_button);
		gtk_widget_grab_focus(no_button);
	}
	gtk_signal_connect(GTK_OBJECT (yes_button), "clicked",
							GTK_SIGNAL_FUNC (DoFunctionOfConfirmationBox), function_if_yes);
//ForGTK3	gtk_container_add (GTK_CONTAINER (GTK_DIALOG(ConfirmDialog)->action_area), yes_button);
	gtk_container_add (GTK_CONTAINER (gtk_dialog_get_action_area(GTK_DIALOG(ConfirmDialog))), yes_button);
	/* Add the label, and show everything we've added to the dialog. */
//ForGTK3	gtk_container_add (GTK_CONTAINER (GTK_DIALOG(ConfirmDialog)->vbox), label);
	gtk_container_add (GTK_CONTAINER (gtk_dialog_get_content_area(GTK_DIALOG(ConfirmDialog))), label);
	gtk_window_set_modal(GTK_WINDOW(ConfirmDialog),TRUE);
	gtk_window_set_title(GTK_WINDOW(ConfirmDialog),title);
	gtk_window_set_position(GTK_WINDOW(ConfirmDialog),GTK_WIN_POS_CENTER);
	gtk_widget_show_all (ConfirmDialog);
}
void ShowConfirmationBox(const char * title,const char * text,void * function_if_yes)
{
	ShowConfirmationBoxWithChoiceOrNot( title, text, function_if_yes, TRUE );
}

void QuitAppliGtk()
{
	ClassicLadderEndOfAppli( );
	gtk_main_quit();
}

void DoQuitGtkApplication( void )
{
	gtk_widget_destroy( MainSectionWindow ); //sends signal "destroy" that will call QuitAppliGtk()...
}
void ConfirmQuit( void )
{
	if ( InfosGene->AskConfirmationToQuit )
		ShowConfirmationBox( _("Warning!"), _("If not saved, all modifications will be lost.\nDo you really want to quit ?\n"), DoQuitGtkApplication );
	else if (EditDatas.ModeEdit==TRUE ) {
             if (!modmaster)  
                {  
                 ShowConfirmationBox( _("Confirm!"), _("Do you really want to quit ?\n"), DoQuitGtkApplication );
                }else{
                      ShowConfirmationBox( _("Warning!"), _("MODBUS will stop if you quit.\n Do you really want to quit ?\n"), DoQuitGtkApplication );
                     }
            }
	else
		DoQuitGtkApplication( );
}
gint MainSectionWindowDeleteEvent( GtkWidget * widget, GdkEvent * event, gpointer data )
{
	(void)widget;
	(void)event;
	(void)data;
	ConfirmQuit( );
	// we do not want that the window be destroyed.
	return TRUE;
}

void MessageInStatusBar( char * msg )
{
  gtk_statusbar_pop(GTK_STATUSBAR(StatusBar), StatusBarContextId);
  gtk_statusbar_push(GTK_STATUSBAR(StatusBar), StatusBarContextId, msg);
}

void MainSectionWindowInitGtk()
{
	GtkWidget *vbox,*hboxtop; //,*hboxbottom,*hboxbottom2;
	GtkWidget *hboxmiddle;
//	GtkWidget *ButtonQuit;
//	GtkWidget *ButtonNew,*ButtonLoad,*ButtonSave,*ButtonSaveAs,*ButtonReset,*ButtonConfig,*ButtonAbout;
//	GtkWidget *ButtonEdit,*ButtonSymbols,*ButtonSpyVars,*ButtonLogBook;
//#ifdef GNOME_PRINT_USE
//	GtkWidget *ButtonPrint,*ButtonPrintPreview,*ButtonExportSVG,*ButtonExportPNG,*ButtonCopyToClipboard;
//#endif
//ForGTK3, deprecated...	GtkTooltips * TooltipsEntryLabel, * TooltipsEntryComment;
	GtkUIManager * PtrUIManager;

	MainSectionWindow = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title ( GTK_WINDOW(MainSectionWindow), _("ClassicLadder Section Display"));

	vbox = gtk_vbox_new (FALSE, 0);
	gtk_container_add (GTK_CONTAINER (MainSectionWindow), vbox);
	gtk_widget_show (vbox);

	gtk_signal_connect (GTK_OBJECT (MainSectionWindow), "destroy",
						GTK_SIGNAL_FUNC (QuitAppliGtk), NULL);

	PtrUIManager = InitMenusAndToolBar( vbox );
	gtk_window_add_accel_group( GTK_WINDOW( MainSectionWindow ), 
				  gtk_ui_manager_get_accel_group(PtrUIManager) );

	hboxtop = gtk_hbox_new (FALSE,0);
	gtk_container_add (GTK_CONTAINER (vbox), hboxtop);
	gtk_widget_show(hboxtop);
	gtk_box_set_child_packing(GTK_BOX(vbox), hboxtop,
		/*expand*/ FALSE, /*fill*/ FALSE, /*pad*/ 0, GTK_PACK_START);

//ForGTK3, deprecated...	TooltipsEntryLabel = gtk_tooltips_new();
	entrylabel = gtk_entry_new();
//GTK3	gtk_widget_set_usize((GtkWidget *)entrylabel,80,0);
	gtk_widget_set_size_request( entrylabel, 80, -1 );
	gtk_entry_set_max_length(GTK_ENTRY(entrylabel),LGT_LABEL-1);
//ForGTK3	gtk_entry_prepend_text((GtkEntry *)entrylabel,"");
	gtk_entry_set_text(GTK_ENTRY(entrylabel),"");
	gtk_box_pack_start (GTK_BOX (hboxtop), entrylabel, FALSE, FALSE, 0);
//ForGTK3, deprecated...	gtk_tooltips_set_tip ( TooltipsEntryLabel, entrylabel, "Label of the current selected rung", NULL );
	gtk_widget_set_tooltip_text( entrylabel, _("Label of the current selected rung") );
	gtk_widget_show(entrylabel);
//ForGTK3, deprecated...	TooltipsEntryComment = gtk_tooltips_new();
	entrycomment = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entrycomment),LGT_COMMENT-1);
//ForGTK3	gtk_entry_prepend_text((GtkEntry *)entrycomment,"");
	gtk_entry_set_text(GTK_ENTRY(entrycomment),"");
	gtk_box_pack_start (GTK_BOX (hboxtop), entrycomment, TRUE, TRUE, 0);
//ForGTK3, deprecated...	gtk_tooltips_set_tip ( TooltipsEntryComment, entrycomment, "Comment of the current selected rung", NULL );
	gtk_widget_set_tooltip_text( entrycomment, _("Comment of the current selected ladder rung or sequential page") );
	gtk_widget_show(entrycomment);

	CheckDispSymbols = gtk_check_button_new_with_label(_("Display symbols"));
	gtk_box_pack_start( GTK_BOX (hboxtop), CheckDispSymbols, FALSE, FALSE, 0 );
	gtk_toggle_button_set_active( GTK_TOGGLE_BUTTON( CheckDispSymbols ), InfosGene->DisplaySymbols );
	gtk_signal_connect( GTK_OBJECT(CheckDispSymbols), "toggled",
				GTK_SIGNAL_FUNC(CheckDispSymbols_toggled), NULL );
	gtk_widget_show( CheckDispSymbols );

#if defined( RT_SUPPORT ) || defined( __XENO__ )
	DurationOfLastScan = gtk_entry_new();
//GTK3	gtk_widget_set_usize(DurationOfLastScan,150,0);
	gtk_widget_set_size_request( DurationOfLastScan, 150, -1 );
//    gtk_entry_set_max_length((GtkEntry *)DurationOfLastScan,LGT_COMMENT-1);
//    gtk_entry_set_max_length((GtkEntry *)DurationOfLastScan,20);
//ForGTK3	gtk_entry_prepend_text((GtkEntry *)DurationOfLastScan,"---");
	gtk_entry_set_text(GTK_ENTRY(DurationOfLastScan),"---");
	gtk_box_pack_start (GTK_BOX (hboxtop), DurationOfLastScan, FALSE, FALSE, 0);
	gtk_widget_set_sensitive(DurationOfLastScan, FALSE);
	gtk_widget_show(DurationOfLastScan);
#endif


	hboxmiddle = gtk_hbox_new (FALSE,0);
	gtk_container_add (GTK_CONTAINER (vbox), hboxmiddle);
	gtk_widget_show(hboxmiddle);
	gtk_box_set_child_packing(GTK_BOX(vbox), hboxmiddle,
		/*expand*/ TRUE, /*fill*/ TRUE, /*pad*/ 0, GTK_PACK_START);

	/* Create the drawing area */
	drawing_area = gtk_drawing_area_new ();
//ForGTK3	gtk_drawing_area_size (GTK_DRAWING_AREA (drawing_area) ,
//ForGTK3							BLOCK_WIDTH_DEF*RUNG_WIDTH+20 ,
//ForGTK3							BLOCK_HEIGHT_DEF*RUNG_HEIGHT+45);
	gtk_widget_set_size_request(drawing_area,
							BLOCK_WIDTH_DEF*RUNG_WIDTH+20 ,
							BLOCK_HEIGHT_DEF*RUNG_HEIGHT+45);
	gtk_box_pack_start (GTK_BOX (hboxmiddle), drawing_area, TRUE, TRUE, 0);
	gtk_widget_show (drawing_area);

	AdjustVScrollBar = (GtkAdjustment *)gtk_adjustment_new( 0, 0, 0, 0, 0, 0);
	VScrollBar = gtk_vscrollbar_new( AdjustVScrollBar );
	gtk_box_pack_start (GTK_BOX (hboxmiddle), VScrollBar, FALSE, FALSE, 0);
	gtk_widget_show (VScrollBar);

	AdjustHScrollBar = (GtkAdjustment *)gtk_adjustment_new( 0, 0, 0, 0, 0, 0);
	HScrollBar = gtk_hscrollbar_new( AdjustHScrollBar );
	gtk_box_pack_start (GTK_BOX (vbox), HScrollBar, FALSE, FALSE, 0);
	gtk_widget_show (HScrollBar);
	UpdateVScrollBar();

	gtk_signal_connect(GTK_OBJECT (AdjustVScrollBar), "value-changed",
						GTK_SIGNAL_FUNC(VScrollBar_value_changed_event), 0);
	gtk_signal_connect(GTK_OBJECT (AdjustHScrollBar), "value-changed",
						GTK_SIGNAL_FUNC(HScrollBar_value_changed_event), 0);

	/* Create the status bar */
    StatusBar = gtk_statusbar_new ();
//	gtk_statusbar_set_has_resize_grip( GTK_STATUSBAR(StatusBar), FALSE );
    gtk_box_pack_start (GTK_BOX(vbox), StatusBar, FALSE, FALSE, 0);
    gtk_widget_show (StatusBar);
    StatusBarContextId = gtk_statusbar_get_context_id( GTK_STATUSBAR(StatusBar), _("Statusbar") );


//no more used since menu/toolbar added...
#ifdef AAAAAAAAAAAAAAAAAAAAAA
	hboxbottom = gtk_hbox_new (FALSE,0);
	gtk_container_add (GTK_CONTAINER (vbox), hboxbottom);
	gtk_widget_show(hboxbottom);
	gtk_box_set_child_packing(GTK_BOX(vbox), hboxbottom,
		/*expand*/ FALSE, /*fill*/ FALSE, /*pad*/ 0, GTK_PACK_START);

	hboxbottom2 = gtk_hbox_new (FALSE,0);
	gtk_container_add (GTK_CONTAINER (vbox), hboxbottom2);
	gtk_widget_show(hboxbottom2);
	gtk_box_set_child_packing(GTK_BOX(vbox), hboxbottom2,
		/*expand*/ FALSE, /*fill*/ FALSE, /*pad*/ 0, GTK_PACK_START);

	ButtonNew = gtk_button_new_with_label (_("New"));
	gtk_box_pack_start (GTK_BOX (hboxbottom), ButtonNew, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonNew), "clicked",
						(GtkSignalFunc) ButtonNew_click, 0);
	gtk_widget_show (ButtonNew);
	ButtonLoad = gtk_button_new_with_label (_("Load"));
	gtk_box_pack_start (GTK_BOX (hboxbottom), ButtonLoad, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonLoad), "clicked",
						(GtkSignalFunc) ButtonLoad_click, 0);
	gtk_widget_show (ButtonLoad);
	ButtonSave = gtk_button_new_with_label (_("Save"));
	gtk_box_pack_start (GTK_BOX (hboxbottom), ButtonSave, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonSave), "clicked",
						(GtkSignalFunc) ButtonSave_click, 0);
	gtk_widget_show (ButtonSave);
	ButtonSaveAs = gtk_button_new_with_label (_("Save As"));
	gtk_box_pack_start (GTK_BOX (hboxbottom), ButtonSaveAs, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonSaveAs), "clicked",
						(GtkSignalFunc) ButtonSaveAs_click, 0);
	gtk_widget_show (ButtonSaveAs);
	ButtonReset = gtk_button_new_with_label (_("Reset"));
	gtk_box_pack_start (GTK_BOX (hboxbottom), ButtonReset, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonReset), "clicked",
						(GtkSignalFunc) ButtonReset_click, 0);
	gtk_widget_show (ButtonReset);
	ButtonRunStop = gtk_button_new_with_label (_("Stop"));
	gtk_box_pack_start (GTK_BOX (hboxbottom), ButtonRunStop, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonRunStop), "clicked",
						(GtkSignalFunc) ButtonRunStop_click, 0);
	gtk_widget_show (ButtonRunStop);
	ButtonSpyVars = gtk_button_new_with_label (_("Vars"));
	gtk_box_pack_start (GTK_BOX (hboxbottom), ButtonSpyVars, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonSpyVars), "clicked",
						(GtkSignalFunc) OpenSpyVarsWindow, 0);
	gtk_widget_show (ButtonSpyVars);
	//ButtonLogBook = gtk_button_new_with_label ("Log");
	//gtk_box_pack_start (GTK_BOX (hboxbottom), ButtonLogBook, TRUE, TRUE, 0);
	//gtk_signal_connect(GTK_OBJECT (ButtonLogBook), "clicked",
						//(GtkSignalFunc) OpenLogBookWindow, 0);
	//gtk_widget_show (ButtonLogBook);
	ButtonAbout = gtk_button_new_with_label (_("About"));
	gtk_box_pack_start (GTK_BOX (hboxbottom), ButtonAbout, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonAbout), "clicked",
						(GtkSignalFunc) ButtonAbout_click, 0);
	gtk_widget_show (ButtonAbout);

	ButtonEdit = gtk_button_new_with_label (_("Editor"));
	gtk_box_pack_start (GTK_BOX (hboxbottom2), ButtonEdit, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonEdit), "clicked",
						(GtkSignalFunc) OpenEditWindow, 0);
	gtk_widget_show (ButtonEdit);
	ButtonSymbols = gtk_button_new_with_label (_("Symbols"));
	gtk_box_pack_start (GTK_BOX (hboxbottom2), ButtonSymbols, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonSymbols), "clicked",
						(GtkSignalFunc) OpenSymbolsWindow, 0);
	gtk_widget_show (ButtonSymbols);
	ButtonConfig = gtk_button_new_with_label (_("Config"));
	gtk_box_pack_start (GTK_BOX (hboxbottom2), ButtonConfig, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonConfig), "clicked",
						(GtkSignalFunc) ButtonConfig_click, 0);
	gtk_widget_show (ButtonConfig);
	ButtonExportSVG = gtk_button_new_with_label ("ExportSVG");
	gtk_box_pack_start (GTK_BOX (hboxbottom2), ButtonExportSVG, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonExportSVG), "clicked",
						(GtkSignalFunc) ButtonExportSvgOrPng_click, (void *)1);
	gtk_widget_show (ButtonExportSVG);
	ButtonExportPNG = gtk_button_new_with_label ("PNG");
	gtk_box_pack_start (GTK_BOX (hboxbottom2), ButtonExportPNG, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonExportPNG), "clicked",
						(GtkSignalFunc) ButtonExportSvgOrPng_click, 0);
	gtk_widget_show (ButtonExportPNG);

	ButtonCopyToClipboard = gtk_button_new_with_label ("ToClipboard");
	gtk_box_pack_start (GTK_BOX (hboxbottom2), ButtonCopyToClipboard, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonCopyToClipboard), "clicked",
						(GtkSignalFunc) ButtonCopyToClipboard_click, 0);
	gtk_widget_show (ButtonCopyToClipboard);
//#ifdef GNOME_PRINT_USE
	ButtonPrintPreview = gtk_button_new_with_label (_("Preview"));
	gtk_box_pack_start (GTK_BOX (hboxbottom2), ButtonPrintPreview, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonPrintPreview), "clicked",
						(GtkSignalFunc) PrintPreviewGnome, 0);
	gtk_widget_show (ButtonPrintPreview);
	ButtonPrint = gtk_button_new_with_label (_("Print"));
	gtk_box_pack_start (GTK_BOX (hboxbottom2), ButtonPrint, TRUE, TRUE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonPrint), "clicked",
						(GtkSignalFunc) PrintGnome, 0);
	gtk_widget_show (ButtonPrint);
//#endif
	ButtonQuit = gtk_button_new_with_label (_("Quit"));
	gtk_box_pack_start (GTK_BOX (hboxbottom2), ButtonQuit, TRUE, TRUE, 0);
//    gtk_signal_connect_object (GTK_OBJECT (ButtonQuit), "clicked",
//                                GTK_SIGNAL_FUNC (gtk_widget_destroy),
//                                GTK_OBJECT (RungWindow));
	gtk_signal_connect_object (GTK_OBJECT (ButtonQuit), "clicked",
								ConfirmQuit, NULL);
	gtk_widget_show (ButtonQuit);
#endif


	/* Signal used to redraw */
#if GTK_MAJOR_VERSION>=3
	gtk_signal_connect (GTK_OBJECT (drawing_area), "draw",
						GTK_SIGNAL_FUNC(draw_callback), NULL);
#else
	gtk_signal_connect (GTK_OBJECT (drawing_area), "expose_event",
						GTK_SIGNAL_FUNC(expose_event), NULL);
#endif
//Cairo	gtk_signal_connect (GTK_OBJECT(drawing_area),"configure_event",
//Cairo						(GtkSignalFunc) configure_event, NULL);

	/* Event signals */
	gtk_signal_connect (GTK_OBJECT (drawing_area), "button_press_event",
						GTK_SIGNAL_FUNC(button_press_event), NULL);
	gtk_signal_connect (GTK_OBJECT (drawing_area), "motion_notify_event",
						GTK_SIGNAL_FUNC(motion_notify_event), NULL);
	gtk_signal_connect (GTK_OBJECT (drawing_area), "button_release_event",
						GTK_SIGNAL_FUNC(button_release_event), NULL);
	gtk_signal_connect (GTK_OBJECT (drawing_area), "scroll_event",
						GTK_SIGNAL_FUNC(mouse_scroll_event), NULL);
	gtk_widget_set_events (drawing_area, GDK_EXPOSURE_MASK
							| GDK_LEAVE_NOTIFY_MASK
							| GDK_BUTTON_PRESS_MASK
							| GDK_BUTTON_RELEASE_MASK
                                                        | GDK_SCROLL_MASK  // mouse scroll
							| GDK_POINTER_MOTION_MASK
							| GDK_POINTER_MOTION_HINT_MASK);

	gtk_signal_connect( GTK_OBJECT(MainSectionWindow), "delete_event",
		GTK_SIGNAL_FUNC(MainSectionWindowDeleteEvent), 0 );
	gtk_widget_show (MainSectionWindow);

	GetTheSizesForRung();
}

void RedrawSignalDrawingArea( void )
{
#if GTK_MAJOR_VERSION>=3
	gtk_widget_queue_draw( drawing_area );
#else
	GdkRegion * region = gdk_drawable_get_clip_region( drawing_area->window );
	// redraw completely by exposing it
	gdk_window_invalidate_region( drawing_area->window, region, TRUE );
	gdk_window_process_updates( drawing_area->window, TRUE );
	gdk_region_destroy( region );
#endif
}

static gint PeriodicUpdateDisplay(gpointer data)
{
	(void)data;
	if (InfosGene->LadderState==STATE_RUN)
	{
#if defined( RT_SUPPORT ) || defined( __XENO__ )
		char TextBuffer[ 20 ];
		snprintf(TextBuffer, sizeof(TextBuffer) , _("%d µs"), InfosGene->DurationOfLastScan/1000);
		gtk_entry_set_text(GTK_ENTRY(DurationOfLastScan),TextBuffer);
#endif
		if (InfosGene->CmdRefreshVarsBits)
		{
			RefreshAllBoolsVars();
			InfosGene->CmdRefreshVarsBits = FALSE;
		}
		DisplayFreeVarSpy();
                //XXX These lines is for features that is not implemented.
		//if ( InfosGene->LogContentModified )
		//{
			//DisplayLogBookEvents( TRUE/*OnLogContentModified*/ );
			//InfosGene->LogContentModified = FALSE;
		//}
		//if ( InfosGene->DefaultLogListModified )
		//{
			//int NbrDefs = FindCurrentDefaults( );
			//InfosGene->DefaultLogListModified = FALSE;
			//if ( NbrDefs>0 )
			//{
				//char * ListDefaultsText = (char *)malloc( NbrDefs*(EVENT_SYMBOL_LGT+10)+10 );
				//if ( ListDefaultsText )
				//{
					//StrConfigEventLog * pCfgEvtLog;
					//int ScanList;
					//char OneEventText[ EVENT_SYMBOL_LGT+10 ];
					//sprintf( ListDefaultsText, "DEFAULT%s : ", NbrDefs>1?"S":"");
//printf("nbr defaults=%d\n", NbrDefs);
					//for( ScanList=0; ScanList<NbrDefs; ScanList++ )
					//{
						//pCfgEvtLog = &ConfigEventLog[ ListCurrentDefType[ ScanList ] ];
						//display value parameter after symbol name if many variables configured for the same event !
						//if ( ListCurrentDefParam[ ScanList ]!=-1 )
							//sprintf( OneEventText, "%s%d ", pCfgEvtLog->Symbol, ListCurrentDefParam[ ScanList ] );
						//else
							//sprintf( OneEventText, "%s ", pCfgEvtLog->Symbol );
						//strcat( ListDefaultsText, OneEventText );
					//}
					//MessageInStatusBar( ListDefaultsText );
					//free( ListDefaultsText );
				//}
			//}
			//else
			//{
				//MessageInStatusBar( "No default." );
			//}
		//}
	}
	if (InfosGene->LadderState!=STATE_LOADING )
	{
//		DrawCurrentSection( );
//		CairoDrawCurrentSection( );
		RedrawSignalDrawingArea( );
	}
	if ( InfosGene->HardwareErrMsgToDisplay[ 0 ]!='\0' )
	{
		ShowMessageBox( _("Config hardware error occurred!"), InfosGene->HardwareErrMsgToDisplay, _("Ok") );
		InfosGene->HardwareErrMsgToDisplay[ 0 ] = '\0';
	}
	return 1;
}


void InitGtkWindows( int argc, char *argv[] )
{
	//debug_printf( _("Your GTK+ version is %d.%d.%d\n"), gtk_major_version, gtk_minor_version,
			//gtk_micro_version );
//ProblemWithPrint	g_thread_init (NULL);
//ProblemWithPrint	gdk_threads_init ();
    gtk_init (&argc, &argv);

	VarsWindowInitGtk();
	MainSectionWindowInitGtk();
//moved before, else crashing when adding tooltips...?
//	VarsWindowInitGtk();
	EditorInitGtk();
	PropertiesInitGtk();
	ManagerInitGtk( );
	SymbolsInitGtk( );
SetMenuStateForRunStopSwitch( TRUE );
        ShowErrorMessage( _("Error"), _("Failed MODBUS communications"), _("Ok") );
    g_timeout_add( TIME_UPDATE_GTK_DISPLAY_MS, PeriodicUpdateDisplay, NULL );
}

void UpdateAllGtkWindows( void )
{
	ManagerDisplaySections( TRUE/*ForgetSectionSelected*/ );
//	DrawCurrentSection( );
	RedrawSignalDrawingArea( );
	refresh_label_comment( );
	autorize_prevnext_buttons( TRUE );
	UpdateVScrollBar();
//moved at top in v0.9.7	ManagerDisplaySections( );
	DisplaySymbols( );
}

void UpdateWindowTitleWithProjectName( void )
{
	char Buff[ 250 ];
	int ScanFileNameOnly = 0;
	if ( strlen(InfosGene->CurrentProjectFileName)>2 )
	{
		ScanFileNameOnly = strlen(InfosGene->CurrentProjectFileName)-1;
		while( ScanFileNameOnly>0 && InfosGene->CurrentProjectFileName[ScanFileNameOnly-1]!='/' && InfosGene->CurrentProjectFileName[ScanFileNameOnly-1]!='\\')
			ScanFileNameOnly--;
	}
	snprintf(Buff, sizeof(Buff), _("Section Display of %s"), &InfosGene->CurrentProjectFileName [ScanFileNameOnly] );
	gtk_window_set_title ((GtkWindow *)MainSectionWindow, Buff );
}

gint ErrorMessageDeleteEvent( GtkWidget * widget, GdkEvent * event, gpointer data )
{
	(void)widget;
	(void)event;
	(void)data;
	gtk_widget_hide( dialog );
	// we do not want that the window be destroyed.
	return TRUE;
}

void ShowErrorMessage(const char * title, const char * text, const char * button)
{
	/* From the example in gtkdialog help */
	//GtkWidget *label, *okay_button;
	/* Create the widgets */
	dialog = gtk_dialog_new();
	label = gtk_label_new (text);
	okay_button = gtk_button_new_with_label(button);
	/* Ensure that the dialog box is destroyed when the user clicks ok. */
	g_signal_connect_swapped (okay_button, "clicked",
							G_CALLBACK (ErrorMessageDeleteEvent), dialog);
        g_signal_connect( dialog, "delete_event",
		G_CALLBACK(ErrorMessageDeleteEvent), 0 );
	gtk_container_add (GTK_CONTAINER (gtk_dialog_get_action_area(GTK_DIALOG(dialog))),
					okay_button);
	gtk_widget_grab_focus(okay_button);
	/* Add the label, and show everything we've added to the dialog. */
	gtk_container_add (GTK_CONTAINER (gtk_dialog_get_content_area(GTK_DIALOG(dialog))),
					label);
	//gtk_window_set_modal(GTK_WINDOW(dialog),TRUE);
	gtk_window_set_title(GTK_WINDOW(dialog),title);
	gtk_window_set_position(GTK_WINDOW(dialog),GTK_WIN_POS_CENTER);
        
	gtk_widget_show_all (dialog);
        gtk_widget_hide( dialog );
}
void CheckForErrors (void)
{
    static int temp;
        if ( (ReadVar( VAR_ERROR_BIT, 0 )==TRUE) && (temp==0))
           { 
             temp=1;
             //ShowErrorMessage( _("Error"), _("Failed MODBUS communications"), _("Ok") );
             if ( !gtk_widget_get_visible( dialog ) ) {  gtk_widget_show (dialog);  }
           }
        if ( ReadVar( VAR_ERROR_BIT, 0 )==FALSE) {    temp=0;  }
}

