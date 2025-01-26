/* Classic Ladder Project */
/* Copyright (C) 2001-2008 Marc Le Douarain */
/* http://www.multimania.com/mavati/classicladder */
/* http://www.sourceforge.net/projects/classicladder */
/* May 2001 */
/* --------------------------------- */
/* Editor properties - GTK interface */
/* --------------------------------- */
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

#include <gtk/gtk.h>
#include <stdio.h>
#include <string.h>
#include "classicladder.h"
#include "global.h"
#include "editproperties_gtk.h"
#include "edit.h"
#include "classicladder_gtk.h"

#include <libintl.h> // i18n
#include <locale.h> // i18n

#include <rtapi_string.h>

GtkWidget *PropertiesWindow;
GtkWidget *PropLabelParam[NBR_PARAMS_PER_OBJ],*PropEntryParam[NBR_PARAMS_PER_OBJ];
GtkWidget *PropEntryBaseParam[NBR_PARAMS_PER_OBJ];
GtkWidget *PropEntryTimerModeParam[NBR_PARAMS_PER_OBJ];
GtkWidget *ButtonApplyProperties;
//int SavePosX = -1;
//int SavePosY = -1;
//static char FirstOpenToSetPosition = FALSE;


//convenient function by Nicolas Joseph (from faq at http://gtk.developpez.com)
void gtk_combo_box_set_active_text( GtkComboBox *p_combo_box, const gchar *text )
{
  gint i;
  gint nb_entry = 0;
  GtkTreeModel *p_tree_model = NULL;

  g_return_if_fail (p_combo_box);
  g_return_if_fail (text);

  p_tree_model = gtk_combo_box_get_model (p_combo_box);
  nb_entry = gtk_tree_model_iter_n_children (p_tree_model, NULL);
  for (i = 0; i < nb_entry; i++)
  {
    gtk_combo_box_set_active (p_combo_box, i);
    if (strcmp (gtk_combo_box_get_active_text( MY_GTK_COMBO_BOX(p_combo_box)), text) == 0)
    {
      break;
    }
  }
  if (i == nb_entry)
  {
    gtk_combo_box_set_active (p_combo_box, 0);
  }
}
// gtk_combo_box_get_active_text( ) - a newly allocated string containing the currently active text. Must be freed with g_free().

void SetProperty(int NumParam,char * LblParam,char * ValParam,char SetFocus)
{
	gtk_label_set_text((GtkLabel *)PropLabelParam[NumParam],LblParam);
	if (strcmp(LblParam,_("Base"))==0)
	{
		gtk_widget_hide(PropEntryParam[NumParam]);
		gtk_widget_show(PropEntryBaseParam[NumParam]);
		gtk_widget_hide(PropEntryTimerModeParam[NumParam]);
//		gtk_entry_set_text((GtkEntry*)((GtkCombo *)PropEntryBaseParam[NumParam])->entry,ValParam);
		gtk_combo_box_set_active_text(GTK_COMBO_BOX(PropEntryBaseParam[NumParam]),ValParam);
		if (SetFocus)
			gtk_widget_grab_focus( PropEntryBaseParam[NumParam] );
	}
	else
	{
		if (strcmp(LblParam,_("TimerMode"))==0)
		{
			gtk_widget_hide(PropEntryParam[NumParam]);
			gtk_widget_hide(PropEntryBaseParam[NumParam]);
			gtk_widget_show(PropEntryTimerModeParam[NumParam]);
//			gtk_entry_set_text((GtkEntry*)((GtkCombo *)PropEntryTimerModeParam[NumParam])->entry,ValParam);
		gtk_combo_box_set_active_text(GTK_COMBO_BOX(PropEntryTimerModeParam[NumParam]),ValParam);
		gtk_entry_set_text(GTK_ENTRY(PropEntryParam[NumParam]),ValParam);
			if (SetFocus)
				gtk_widget_grab_focus( PropEntryTimerModeParam[NumParam] );
		}
		else
		{
			// others standard parameters
			gtk_widget_show(PropEntryParam[NumParam]);
			gtk_widget_hide(PropEntryBaseParam[NumParam]);
			gtk_widget_hide(PropEntryTimerModeParam[NumParam]);
			gtk_entry_set_text(GTK_ENTRY(PropEntryParam[NumParam]),ValParam);
			if (SetFocus)
				gtk_widget_grab_focus( PropEntryParam[NumParam] );
		}
	}
	/* if no first param, means no params at all */
	/* so no sense to need to apply ! */
	if (NumParam==0)
	{
		if (strcmp(LblParam,"---")==0)
		{
//			gtk_widget_hide(ButtonApplyProperties);
			gtk_widget_set_sensitive( ButtonApplyProperties, FALSE );
		}
		else
		{
//			gtk_widget_show(ButtonApplyProperties);
			gtk_widget_set_sensitive( ButtonApplyProperties, TRUE );
			gtk_window_present( GTK_WINDOW(PropertiesWindow) );
		}
	}
}

char * GetProperty(int NumParam)
{
	static char ValTxtParameter[61];
	const gchar * TxtParameter;
	/* Convert to a number if it is a base */
//ForGTK3	gtk_label_get(GTK_LABEL(PropLabelParam[NumParam]),&TxtParameter);
	TxtParameter = gtk_label_get_text(GTK_LABEL(PropLabelParam[NumParam]));
	if (strcmp(TxtParameter,_("Base"))==0)
	{
//		strcpy( ValTxtParameter , (char *)gtk_entry_get_text((GtkEntry *)((GtkCombo *)PropEntryBaseParam[NumParam])->entry) );
		char *pGetText = gtk_combo_box_get_active_text( MY_GTK_COMBO_BOX(PropEntryBaseParam[NumParam]) );
		strcpy( ValTxtParameter, pGetText );
		g_free( pGetText );
	}
	else if (strcmp(TxtParameter,_("TimerMode"))==0)
	{
//			strcpy( ValTxtParameter , (char *)gtk_entry_get_text((GtkEntry *)((GtkCombo *)PropEntryTimerModeParam[NumParam])->entry) );
		char *pGetText = gtk_combo_box_get_active_text( MY_GTK_COMBO_BOX(PropEntryTimerModeParam[NumParam]) );
		strcpy( ValTxtParameter, pGetText );
		g_free( pGetText );
	}
	else
	{
		// others standard parameters
		strncpy( ValTxtParameter , (char *)gtk_entry_get_text((GtkEntry *)PropEntryParam[NumParam]), 60 );
		ValTxtParameter[ 60 ] = '\0';
	}
	return ValTxtParameter;
}

gint PropertiesWindowDeleteEvent( GtkWidget * widget, GdkEvent * event, gpointer data )
{
	(void)widget;
	(void)event;
	(void)data;
	// we do not want that the window be destroyed.
	return TRUE;
}

void ShowPropertiesWindow( int Visible )
{
	if ( Visible )
	{
		gtk_widget_show(PropertiesWindow);
/*		if ( !FirstOpenToSetPosition )
		{
			// try to move at a nice place the window (to top or bottom of the edit window)
			int editx,edity,editw,edith;
			int mainx,mainy;
			int winw,winh;
			int winx=0,winy=0;
			int offsetx = 0;
			gtk_window_get_size( GTK_WINDOW(EditWindow), &editw, &edith );
			gtk_window_get_position( GTK_WINDOW(EditWindow), &editx, &edity );
			gtk_window_get_position( GTK_WINDOW(MainSectionWindow), &mainx, &mainy );
			gtk_window_get_size( GTK_WINDOW(PropertiesWindow), &winw, &winh );
			// if edit on left of the main
			if ( editx<mainx && editw<winw )
			  offsetx = winw-editw;
			// place on the top ?
			if ( edity>winh+10 )
			{
				winx = editx-offsetx;
				winy = edity-winh-10;
			}
			else
			{
				// move the window on the bottom
				winx = editx-offsetx;
				winy = edity+edith+10;
			}
			printf("to move properties window: x%d,y%d,w%d,h%d => x%d,y%d,w%d,h%d\n",editx,edity,editw,edith,winx,winy,winw,winh);
			gtk_window_move( GTK_WINDOW(PropertiesWindow), winx,winh );
			FirstOpenToSetPosition = TRUE;
		}
		else if ( SavePosX!=-1 && SavePosY!=-1 )
			gtk_window_move( GTK_WINDOW(PropertiesWindow), SavePosX, SavePosY );*/

	}
	else
	{
		// test required to avoid a warning in RememberWindowPosiPrefs(), because called by edit at startup, with window not show...
//ForGTK3		if ( GTK_WIDGET_VISIBLE( GTK_WINDOW(PropertiesWindow) ) )
		if ( MY_GTK_WIDGET_VISIBLE( PropertiesWindow ) )
		{
			gtk_widget_hide(PropertiesWindow);
		}
	}
}

void PropertiesInitGtk()
{
	GtkWidget *vbox;
	GtkWidget *hbox[NBR_PARAMS_PER_OBJ + 1];
	int NumParam;
//	GList *BaseItems = NULL;
//	GList *TimersModesItems = NULL;
	int ScanBase;
	int ScanTimerMode;
/*	do
	{
		BaseItems = g_list_append(BaseItems,CorresDatasForBase[ScanBase++].ParamSelect);
	}
	while(ScanBase<NBR_TIMEBASES);
	do
	{
		TimersModesItems = g_list_append(TimersModesItems,TimersModesStrings[ScanTimerMode++]);
	}
	while(ScanTimerMode<NBR_TIMERSMODES);*/

	PropertiesWindow = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title ((GtkWindow *)PropertiesWindow, _("Properties"));

	vbox = gtk_vbox_new (FALSE, 0);
	gtk_container_add (GTK_CONTAINER (PropertiesWindow), vbox);
	gtk_widget_show (vbox);

	for (NumParam = 0 ; NumParam<NBR_PARAMS_PER_OBJ ; NumParam++)
	{
		hbox[NumParam] = gtk_hbox_new (FALSE, 0);
		gtk_container_add (GTK_CONTAINER (vbox), hbox[NumParam]);
		gtk_widget_show (hbox[NumParam]);

		PropLabelParam[NumParam] = gtk_label_new("---");
//GTK3		gtk_widget_set_usize((GtkWidget *)PropLabelParam[NumParam],85,0);
		gtk_widget_set_size_request(PropLabelParam[NumParam],85,-1);
		gtk_box_pack_start (GTK_BOX (hbox[NumParam]), PropLabelParam[NumParam], FALSE, FALSE, 0);
		gtk_widget_show (PropLabelParam[NumParam]);

		/* For numbers */
		PropEntryParam[NumParam] = gtk_entry_new();
//        gtk_widget_set_usize((GtkWidget *)PropEntryParam[NumParam],85,0);
		gtk_box_pack_start (GTK_BOX (hbox[NumParam]), PropEntryParam[NumParam], TRUE, TRUE, 0);
		gtk_widget_show (PropEntryParam[NumParam]);
		gtk_signal_connect(GTK_OBJECT (PropEntryParam[NumParam]), "activate",
							GTK_SIGNAL_FUNC(SaveElementProperties), NULL);

		/* For time base */
//		PropEntryBaseParam[NumParam] = gtk_combo_new();
//		gtk_combo_set_value_in_list(GTK_COMBO(PropEntryBaseParam[NumParam]), TRUE /*val*/, FALSE /*ok_if_empty*/);
//		gtk_combo_set_popdown_strings(GTK_COMBO(PropEntryBaseParam[NumParam]),BaseItems);
		PropEntryBaseParam[NumParam] = gtk_combo_box_new_text();
		for( ScanBase=0; ScanBase<NBR_TIMEBASES; ScanBase++ )
			gtk_combo_box_append_text( MY_GTK_COMBO_BOX(PropEntryBaseParam[NumParam]), CorresDatasForBase[ScanBase].ParamSelect );
//////        gtk_widget_set_usize((GtkWidget *)PropEntryBaseParam[NumParam],85,0);
		gtk_box_pack_start (GTK_BOX (hbox[NumParam]), PropEntryBaseParam[NumParam], FALSE, FALSE, 0);

		/* For timer mode */
//		PropEntryTimerModeParam[NumParam] = gtk_combo_new();
//		gtk_combo_set_value_in_list(GTK_COMBO(PropEntryTimerModeParam[NumParam]), TRUE /*val*/, FALSE /*ok_if_empty*/);
//		gtk_combo_set_popdown_strings(GTK_COMBO(PropEntryTimerModeParam[NumParam]),TimersModesItems);
		PropEntryTimerModeParam[NumParam] = gtk_combo_box_new_text();
		for( ScanTimerMode=0; ScanTimerMode<NBR_TIMEBASES; ScanTimerMode++ )
			gtk_combo_box_append_text( MY_GTK_COMBO_BOX(PropEntryTimerModeParam[NumParam]), TimersModesStrings[ScanTimerMode] );
//////        gtk_widget_set_usize((GtkWidget *)PropEntryTimerModeParam[NumParam],85,0);
		gtk_box_pack_start (GTK_BOX (hbox[NumParam]), PropEntryTimerModeParam[NumParam], FALSE, FALSE, 0);
	}

	/* for apply button... */
	hbox[NumParam] = gtk_hbox_new (FALSE, 0);
	gtk_container_add (GTK_CONTAINER (vbox), hbox[NumParam]);
	gtk_widget_show (hbox[NumParam]);

	ButtonApplyProperties = gtk_button_new_with_label(_("Apply"));
	gtk_box_pack_start (GTK_BOX (hbox[NumParam]), ButtonApplyProperties, TRUE, FALSE, 0);
	gtk_signal_connect(GTK_OBJECT (ButtonApplyProperties), "clicked",
						GTK_SIGNAL_FUNC(SaveElementProperties), 0);
	gtk_widget_set_sensitive( ButtonApplyProperties, FALSE );
	gtk_widget_show( ButtonApplyProperties );

//    gtk_widget_show (PropertiesWindow);

	gtk_signal_connect( GTK_OBJECT(PropertiesWindow), "delete_event",
		GTK_SIGNAL_FUNC(PropertiesWindowDeleteEvent), 0 );
}

