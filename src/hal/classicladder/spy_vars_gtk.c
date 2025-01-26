/* Classic Ladder Project */
/* Copyright (C) 2001-2010 Marc Le Douarain */
/* http://membres.lycos.fr/mavati/classicladder/ */
/* http://www.sourceforge.net/projects/classicladder */
/* February 2001 */
/* -------------------------------------------------------------------------------------------- */
/* Spy variables windows (booleans with checkboxes, and any with entry widgets) - GTK interface */
/* + Modify current value of a spy variable                                                     */
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

// modified for EMC
// Chris Morley Feb 08

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <gtk/gtk.h>
#include <libintl.h> // i18n
#include <locale.h> // i18n

#include "classicladder.h"
#include "global.h"
#include "vars_access.h"
#include "drawing.h"
#include "edit.h"
#include "classicladder_gtk.h"
#include "vars_names.h"
#include "symbols.h"
#include "menu_and_toolbar_gtk.h"
#include <rtapi_string.h>

#include "spy_vars_gtk.h"

#define NBR_BOOLS_VAR_SPY 15
#define NBR_TYPE_BOOLS_SPY 3
#define NBR_FREE_VAR_SPY 15

GtkWidget *SpyBoolVarsWindow;
GtkWidget *SpyFreeVarsWindow;
GtkWidget *ModifyVarValueWindow;

GtkWidget *offsetboolvar[ NBR_TYPE_BOOLS_SPY ];
int ValOffsetBoolVar[ NBR_TYPE_BOOLS_SPY ] = { 0, 0, 0 };
GtkWidget *chkvar[ NBR_TYPE_BOOLS_SPY ][ NBR_BOOLS_VAR_SPY ];

GtkWidget *EntryVarSpy[NBR_FREE_VAR_SPY*3], *LabelFreeVars[ NBR_FREE_VAR_SPY];

//ForGTK3, deprecated... GtkTooltips * TooltipsEntryVarSpy[ NBR_FREE_VAR_SPY ];
/* defaults vars to spy list */
int VarSpy[NBR_FREE_VAR_SPY][2] = { {VAR_MEM_WORD,0}, {VAR_MEM_WORD,1}, {VAR_MEM_WORD,2}, {VAR_MEM_WORD,3}, {VAR_MEM_WORD,4}, {VAR_MEM_WORD,5}, {VAR_MEM_WORD,6}, {VAR_MEM_WORD,7}, {VAR_MEM_WORD,8}, {VAR_MEM_WORD,9}, {VAR_MEM_WORD,10}, {VAR_MEM_WORD,11}, {VAR_MEM_WORD,12}, {VAR_MEM_WORD,13}, {VAR_MEM_WORD,14} }; 
GtkWidget *DisplayFormatVarSpy[NBR_FREE_VAR_SPY];
GtkWidget *ModifyVarSpy[NBR_FREE_VAR_SPY];

GtkWidget * ModifyVariableNameEdit;
GtkWidget * ModifyVariableValueEdit;
int CurrentModifyVarType, CurrentModifyVarOffset;
int SaveModifyVarPosX = -1;
int SaveModifyVarPosY = -1;

static gint chkvar_press_event( GtkWidget      *widget, void * numcheck )
{
	long NumCheckWidget = (long)numcheck;
	int Type = VAR_MEM_BIT;
	int Offset = ValOffsetBoolVar[ 0 ];
	int NumCheck = NumCheckWidget;
	if( NumCheckWidget>=NBR_BOOLS_VAR_SPY && NumCheckWidget<2*NBR_BOOLS_VAR_SPY )
	{
		Type = VAR_PHYS_INPUT;
		Offset = ValOffsetBoolVar[ 1 ];
		NumCheck -= NBR_BOOLS_VAR_SPY;
	} 
	if( NumCheckWidget>=2*NBR_BOOLS_VAR_SPY && NumCheckWidget<3*NBR_BOOLS_VAR_SPY )
	{
		Type = VAR_PHYS_OUTPUT;
		Offset = ValOffsetBoolVar[ 2 ];
		NumCheck -= 2*NBR_BOOLS_VAR_SPY;
	} 
	if (gtk_toggle_button_get_active((GtkToggleButton *)widget))
		WriteVar(Type,Offset+NumCheck,1);
	else
		WriteVar(Type,Offset+NumCheck,0);
	return TRUE;
}

void RefreshOneBoolVar( int Type, int Num, int Val )
{
	int Col = 0;
	switch( Type )
	{
		case VAR_PHYS_INPUT: Col = 1; break;
		case VAR_PHYS_OUTPUT: Col = 2; break;
	}
	if ( Num>=ValOffsetBoolVar[ Col ] && Num<ValOffsetBoolVar[ Col ]+NBR_BOOLS_VAR_SPY )
		gtk_toggle_button_set_active((GtkToggleButton *)chkvar[Col][Num-ValOffsetBoolVar[ Col ]],(Val!=0)?TRUE:FALSE);
}

void RefreshAllBoolsVars( )
{
	int NumVar;
	for (NumVar=0; NumVar<NBR_BOOLS_VAR_SPY; NumVar++)
	{
		gtk_toggle_button_set_active( (GtkToggleButton *)chkvar[0][NumVar], ReadVar(VAR_MEM_BIT,NumVar+ValOffsetBoolVar[ 0 ])?TRUE:FALSE );
		gtk_toggle_button_set_active( (GtkToggleButton *)chkvar[1][NumVar], ReadVar(VAR_PHYS_INPUT,NumVar+ValOffsetBoolVar[ 1 ])?TRUE:FALSE );
		gtk_toggle_button_set_active( (GtkToggleButton *)chkvar[2][NumVar], ReadVar(VAR_PHYS_OUTPUT,NumVar+ValOffsetBoolVar[ 2 ])?TRUE:FALSE );
	}
}

void UpdateAllLabelsBoolsVars( )
{
	int ColumnVar, OffVar;
	for(ColumnVar=0; ColumnVar<NBR_TYPE_BOOLS_SPY; ColumnVar++)
	{
		for(OffVar=0; OffVar<NBR_BOOLS_VAR_SPY; OffVar++)
		{
			char BufNumVar[256];
			switch( ColumnVar )
			{
                        case 0:
                            snprintf(BufNumVar, 256, "<span foreground=\"black\" weight=\"bold\">%cB%d</span>",'%', 
                                    OffVar+ValOffsetBoolVar[ ColumnVar ]); 
                            break;
                        case 1:
                            snprintf(BufNumVar, 256, "<span foreground=\"red\" weight=\"bold\">%cI%d</span>",'%',  
                                    OffVar+ValOffsetBoolVar[ ColumnVar ]); 
                            break;
                        case 2:
                            snprintf(BufNumVar, 256, "<span foreground=\"blue\" weight=\"bold\">%cQ%d</span>",'%', 
                                    OffVar+ValOffsetBoolVar[ ColumnVar ]); 
                            break;
			}
			gtk_label_set_markup (GTK_LABEL ( gtk_bin_get_child(GTK_BIN(chkvar[ ColumnVar ][ OffVar ] ))),BufNumVar);
		}
	}
}

static gint OffsetBoolVar_activate_event(GtkWidget *widget, void * NumVarSpy)
{
	int Maxi = 0;
	long NumType = (long)NumVarSpy;
	int ValOffset = atoi( gtk_entry_get_text((GtkEntry *)widget) );
	switch( NumType )
	{
		case 0: Maxi = NBR_BITS; break;
		case 1: Maxi = NBR_PHYS_INPUTS; break;
		case 2: Maxi = NBR_PHYS_OUTPUTS; break;
	}
	if ( ValOffset+NBR_BOOLS_VAR_SPY>Maxi || ValOffset<0 )
		ValOffset = 0;
	ValOffsetBoolVar[ NumType ] = ValOffset;
	UpdateAllLabelsBoolsVars( );
	RefreshAllBoolsVars( );
	return TRUE;
}

gint BoolVarsWindowDeleteEvent( GtkWidget * widget, GdkEvent * event, gpointer data )
{
	(void)widget;
	(void)event;
	(void)data;
	gtk_widget_hide( SpyBoolVarsWindow );
	SetToggleMenuForBoolVarsWindow( FALSE/*OpenedWin*/ );
	// we do not want that the window be destroyed.
	return TRUE;
}

// called per toggle action menu, or at startup (if window saved open or not)...
void OpenSpyBoolVarsWindow( GtkAction * ActionOpen, gboolean OpenIt )
{
	if ( ActionOpen!=NULL )
		OpenIt = gtk_toggle_action_get_active( GTK_TOGGLE_ACTION(ActionOpen) );
	if ( OpenIt )
	{
		gtk_widget_show( SpyBoolVarsWindow );
		gtk_window_present( GTK_WINDOW(SpyBoolVarsWindow) );
	}
	else
	{
		gtk_widget_hide( SpyBoolVarsWindow );
	}
}

void BoolVarsWindowInitGtk()
{
	GtkWidget *vboxboolvars[ NBR_TYPE_BOOLS_SPY ],*vboxmain,*hboxvars;
	long NumCheckWidget,ColumnVar;
//ForGTK3, deprecated...	GtkTooltips * WidgetTooltips[ NBR_TYPE_BOOLS_SPY ];
	
	SpyBoolVarsWindow = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title ((GtkWindow *)SpyBoolVarsWindow, _("Bit Status Window"));
	vboxmain = gtk_vbox_new (FALSE, 0);
	gtk_container_add (GTK_CONTAINER (SpyBoolVarsWindow), vboxmain);
	gtk_widget_show (vboxmain);
	hboxvars = gtk_hbox_new (FALSE, 0);
	gtk_container_add (GTK_CONTAINER (vboxmain), hboxvars);
	gtk_widget_show (hboxvars);

	for( ColumnVar=0; ColumnVar<NBR_TYPE_BOOLS_SPY; ColumnVar++ )
	{
		vboxboolvars[ ColumnVar ] = gtk_vbox_new (FALSE, 0);
		gtk_container_add (GTK_CONTAINER (hboxvars), vboxboolvars[ ColumnVar ]);
		gtk_widget_show (vboxboolvars[ ColumnVar ]);
	}

	NumCheckWidget = 0;
	for(ColumnVar=0; ColumnVar<NBR_TYPE_BOOLS_SPY; ColumnVar++)
	{
		int OffVar;
		offsetboolvar[ ColumnVar ]  = gtk_entry_new();
//ForGTK3, deprecated...		WidgetTooltips[ ColumnVar ] = gtk_tooltips_new();
//ForGTK3, deprecated...		gtk_tooltips_set_tip ( WidgetTooltips[ ColumnVar ], offsetboolvar[ ColumnVar ], "Offset for vars displayed below (press return to apply)", NULL );
		gtk_widget_set_tooltip_text( offsetboolvar[ ColumnVar ], _("Offset for vars displayed below (press return to apply)") );
//GTK3		gtk_widget_set_usize((GtkWidget *)offsetboolvar[ ColumnVar ],40,0);
//////TEST!!!		gtk_widget_set_size_request( offsetboolvar[ ColumnVar ], 40, -1 );
//////gtk_widget_set_size_request( offsetboolvar[ ColumnVar ], 20, -1);
// Changes the size request of the entry to be about the right size for n_chars characters !
gtk_entry_set_width_chars( GTK_ENTRY(offsetboolvar[ ColumnVar ]), 4 );
		gtk_box_pack_start (GTK_BOX(vboxboolvars[ ColumnVar ]),  offsetboolvar[ ColumnVar ] , FALSE, FALSE, 0);
		gtk_widget_show( offsetboolvar[ ColumnVar ] );
		gtk_entry_set_text((GtkEntry *)offsetboolvar[ ColumnVar ],"0");
		gtk_signal_connect(GTK_OBJECT (offsetboolvar[ ColumnVar ]), "activate",
					GTK_SIGNAL_FUNC(OffsetBoolVar_activate_event), (void *)ColumnVar);
		
		for(OffVar=0; OffVar<NBR_BOOLS_VAR_SPY; OffVar++)
		{
			chkvar[ ColumnVar ][ OffVar ] = gtk_check_button_new_with_label("xxxx");
			gtk_box_pack_start (GTK_BOX(vboxboolvars[ ColumnVar ]), chkvar[ ColumnVar ][ OffVar ], FALSE, FALSE, 0);
			gtk_widget_show(chkvar[ ColumnVar ][ OffVar ]);
			gtk_signal_connect(GTK_OBJECT (chkvar[ ColumnVar ][ OffVar ]), "toggled",
					GTK_SIGNAL_FUNC(chkvar_press_event), (void*)NumCheckWidget);
			NumCheckWidget++;
		}
	}
	UpdateAllLabelsBoolsVars( );
	
	gtk_signal_connect( GTK_OBJECT(SpyBoolVarsWindow), "delete_event",
		GTK_SIGNAL_FUNC(BoolVarsWindowDeleteEvent), 0 );

//	gtk_window_set_policy( GTK_WINDOW(SpyBoolVarsWindow), FALSE/*allow_shrink*/, FALSE/*allow_grow*/, TRUE/*auto_shrink*/ );
}


static gint OpenModifyVarWindow_clicked_event(GtkWidget *widget, int NumSpy)
{
	(void)widget;
	char BuffValue[ 30 ];
	CurrentModifyVarType = VarSpy[NumSpy][0];
	CurrentModifyVarOffset = VarSpy[NumSpy][1];

	gtk_entry_set_text( GTK_ENTRY(ModifyVariableNameEdit), CreateVarName(CurrentModifyVarType,CurrentModifyVarOffset,InfosGene->DisplaySymbols) );
	snprintf( BuffValue, sizeof(BuffValue), "%d", ReadVar(CurrentModifyVarType, CurrentModifyVarOffset) );
	gtk_entry_set_text( GTK_ENTRY(ModifyVariableValueEdit), BuffValue );
	gtk_widget_grab_focus( ModifyVariableValueEdit );

	gtk_widget_show( ModifyVarValueWindow );
	if ( SaveModifyVarPosX!=-1 && SaveModifyVarPosY!=-1 )
		gtk_window_move( GTK_WINDOW(ModifyVarValueWindow), SaveModifyVarPosX, SaveModifyVarPosY );

	return TRUE;
}
gint ModifyVarWindowDeleteEvent( GtkWidget * widget, GdkEvent * event, gpointer data )
{
	(void)widget;
	(void)event;
	(void)data;
	gtk_window_get_position( GTK_WINDOW(ModifyVarValueWindow), &SaveModifyVarPosX, &SaveModifyVarPosY );
	gtk_widget_hide( ModifyVarValueWindow );
	// we do not want that the window be destroyed.
	return TRUE;
}
gint ApplyModifiedVar( GtkWidget * widget, GdkEvent * event, gpointer data )
{
	(void)widget;
	(void)event;
	(void)data;
	int NewValue = atoi( gtk_entry_get_text( GTK_ENTRY(ModifyVariableValueEdit) ) );
	WriteVar( CurrentModifyVarType, CurrentModifyVarOffset, NewValue );
	gtk_window_get_position( GTK_WINDOW(ModifyVarValueWindow), &SaveModifyVarPosX, &SaveModifyVarPosY );
	gtk_widget_hide( ModifyVarValueWindow );
	// we do not want that the window be destroyed.
	return TRUE;
}
void ModifyVarWindowInitGtk( )
{
	GtkWidget *vboxMain,*hboxOkCancel;
	GtkWidget *ButtonOk,*ButtonCancel;
	ModifyVarValueWindow = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title ((GtkWindow *)ModifyVarValueWindow, "Modify variable value");
	vboxMain = gtk_vbox_new (FALSE, 0);
	gtk_container_add (GTK_CONTAINER (ModifyVarValueWindow), vboxMain);
	gtk_widget_show (vboxMain);

	ModifyVariableNameEdit = gtk_entry_new();
//	gtk_widget_set_usize( GTK_WIDGET(ModifyVariableValueEdit),110,0);
	gtk_box_pack_start (GTK_BOX( vboxMain ), ModifyVariableNameEdit, TRUE, TRUE, 0);
	gtk_editable_set_editable( GTK_EDITABLE(ModifyVariableNameEdit), FALSE );
	gtk_widget_show( ModifyVariableNameEdit );

	ModifyVariableValueEdit = gtk_entry_new();
//	gtk_widget_set_usize( GTK_WIDGET(ModifyVariableValueEdit),110,0);
	gtk_box_pack_start (GTK_BOX( vboxMain ), ModifyVariableValueEdit, TRUE, TRUE, 0);
	gtk_widget_show( ModifyVariableValueEdit );
	gtk_signal_connect( GTK_OBJECT(ModifyVariableValueEdit), "activate",
                                        GTK_SIGNAL_FUNC(ApplyModifiedVar), (void *)NULL );

	hboxOkCancel = gtk_hbox_new (FALSE, 0);
	gtk_container_add (GTK_CONTAINER (vboxMain), hboxOkCancel);
	gtk_widget_show (hboxOkCancel);

	ButtonOk = gtk_button_new_from_stock( GTK_STOCK_OK );
	gtk_box_pack_start( GTK_BOX(hboxOkCancel), ButtonOk, FALSE, FALSE, 0 );
	gtk_widget_show( ButtonOk );
	gtk_signal_connect( GTK_OBJECT(ButtonOk), "clicked",
                                        GTK_SIGNAL_FUNC(ApplyModifiedVar), (void *)NULL );
	ButtonCancel = gtk_button_new_from_stock( GTK_STOCK_CANCEL );
	gtk_box_pack_start( GTK_BOX(hboxOkCancel), ButtonCancel, FALSE, FALSE, 0 );
	gtk_widget_show( ButtonCancel );
	gtk_signal_connect( GTK_OBJECT(ButtonCancel), "clicked",
                                        GTK_SIGNAL_FUNC(ModifyVarWindowDeleteEvent), NULL );

	gtk_signal_connect( GTK_OBJECT(ModifyVarValueWindow), "delete_event",
		GTK_SIGNAL_FUNC(ModifyVarWindowDeleteEvent), 0 );
}



char * ConvToBin( unsigned int Val )
{
	static char TabBin[ 33 ];
	int Pos;
	unsigned int Mask = 0x80000000;
	char First1 = FALSE;
	rtapi_strxcpy( TabBin, "" );
	for ( Pos = 0; Pos<32; Pos++ )
	{
		if ( Val & Mask )
			First1 = TRUE;
		if ( First1 )
		{
			if ( Val & Mask )
				rtapi_strxcat( TabBin, "1" );
			else
				rtapi_strxcat( TabBin, "0" );
		}
		Mask = Mask>>1;
	}
	if ( Val==0 )
		rtapi_strxcpy( TabBin,"0" );
	return TabBin;
}
// This function updates the signed Integer window
// It checks the display format and displays the number in hex,binary or decimal
// It checks to see what type of word variable it represents (Hal s32 pin in or out or an internal memory variable) and displays that label with colour
// If Displays variable names or symbol names depending on the check box in the section display window
void DisplayFreeVarSpy()
{
	static int LastTime;
	int NumVarSpy,i=NBR_FREE_VAR_SPY;
	int Value;
	char BufferValue[50];
	char DisplayFormat[10];
	char * VarName;
	if ( NBR_WORDS < NBR_FREE_VAR_SPY) { i=NBR_WORDS ;}
	for (NumVarSpy=0; NumVarSpy<i; NumVarSpy++)
	{
		Value = ReadVar(VarSpy[NumVarSpy][0],VarSpy[NumVarSpy][1]);
		rtapi_strxcpy( DisplayFormat, gtk_combo_box_get_active_text(GTK_COMBO_BOX_TEXT(DisplayFormatVarSpy[NumVarSpy]) ));
		rtapi_strxcpy( BufferValue, "" );
		if (strcmp( DisplayFormat,"Dec" )==0 )
			snprintf(BufferValue, sizeof(BufferValue),"%d",Value);
		if (strcmp( DisplayFormat,"Hex" )==0 )
			snprintf(BufferValue, sizeof(BufferValue),"%X",Value);
		if (strcmp( DisplayFormat,"Bin" )==0 )
			rtapi_strxcpy( BufferValue, ConvToBin( Value ) );
		gtk_entry_set_text((GtkEntry *)EntryVarSpy[2*NBR_FREE_VAR_SPY+NumVarSpy],BufferValue);
                VarName= "<span foreground=\"gray\" weight=\"bold\" >Other       </span>";

                switch (VarSpy[NumVarSpy][0])

                      {
                       case VAR_TIMER_VALUE :
                       case VAR_TIMER_PRESET :
                       case VAR_TIMER_RUNNING :
                       case VAR_TIMER_DONE :
                            VarName= "<span foreground=\"brown\" weight=\"bold\" >Timer       </span>";
                            break;
                       case VAR_TIMER_IEC_VALUE :
                       case VAR_TIMER_IEC_PRESET :
                       case VAR_TIMER_IEC_DONE :
                            VarName= "<span foreground=\"brown\" weight=\"bold\" >IEC Timer   </span>";
                            break;
                       case VAR_COUNTER_VALUE :
                       case VAR_COUNTER_PRESET :
                       case VAR_COUNTER_FULL :
                       case VAR_COUNTER_EMPTY :
                       case VAR_COUNTER_DONE :
                            VarName= "<span foreground=\"brown\" weight=\"bold\" >Counter     </span>";
                            break;
                       case VAR_MONOSTABLE_RUNNING :
                       case VAR_MONOSTABLE_PRESET :
                       case VAR_MONOSTABLE_VALUE :
                            VarName= "<span foreground=\"brown\" weight=\"bold\" >Monostable  </span>";
                            break;
                       case VAR_MEM_WORD :
                            VarName= "<span foreground=\"black\" weight=\"bold\" >Memory      </span>";
                            break;
                       case VAR_PHYS_INPUT :
                            VarName= "<span foreground=\"red\" weight=\"bold\" >Bit In Pin  </span>";
                            break;
                       case VAR_PHYS_OUTPUT :
                            VarName= "<span foreground=\"blue\" weight=\"bold\" >Bit Out Pin </span>";
                            break;
                       case VAR_PHYS_FLOAT_INPUT :
                            VarName= "<span foreground=\"red\" weight=\"bold\" >Floatin Pin </span>";
                            break;
                       case VAR_PHYS_FLOAT_OUTPUT :
                            VarName= "<span foreground=\"blue\" weight=\"bold\" >Floatout Pin</span>";
                            break;
                       case VAR_PHYS_WORD_INPUT :
                            VarName= "<span foreground=\"red\" weight=\"bold\" >S32in Pin   </span>";
                            break;
                       case VAR_PHYS_WORD_OUTPUT :
                            VarName= "<span foreground=\"blue\" weight=\"bold\" >S32out Pin  </span>";
                            break;
                       case VAR_MEM_BIT :
                             VarName= "<span foreground=\"black\" weight=\"bold\" >Bit Memory  </span>";
                             break;
                        case VAR_ERROR_BIT :
                             VarName= "<span foreground=\"gold\" weight=\"bold\" >Error Bit   </span>";
                             break;
                        case VAR_STEP_ACTIVITY :
                             VarName= "<span foreground=\"brown\" weight=\"bold\" >Step Active  </span>";
                             break;
                        case VAR_STEP_TIME :
                             VarName= "<span foreground=\"brown\" weight=\"bold\" >Step Run Time</span>";
                             break;
                       }
                 gtk_label_set_markup (GTK_LABEL (LabelFreeVars[NumVarSpy]),VarName);

                 if (InfosGene->DisplaySymbols!=LastTime)
                 {
                 gtk_entry_set_text((GtkEntry *)EntryVarSpy[ NumVarSpy+(1 *NBR_FREE_VAR_SPY)],CreateVarName(VarSpy[NumVarSpy][0],VarSpy[NumVarSpy][1],InfosGene->DisplaySymbols));
                 }
         }
         // we do this check after the FOR loop
         // so it does not toggle each time thru loop
         // Toggle LastTime to match InfoGene
         if (InfosGene->DisplaySymbols!=LastTime)
                 { LastTime=((LastTime-1)*-1); }
 }

static gint EntryVarSpy_activate_event(GtkWidget *widget, int NumSpy)
{
	int NewVarType,NewVarOffset;
	int * NumVarSpy = &VarSpy[NumSpy][0];
	char BufferVar[30];
	rtapi_strxcpy(BufferVar, gtk_entry_get_text((GtkEntry *)widget) );
	if (TextParserForAVar(BufferVar , &NewVarType, &NewVarOffset, NULL, FALSE/*PartialNames*/))
	{
		char * OtherVarName = NULL;
		*NumVarSpy++ = NewVarType;
		*NumVarSpy = NewVarOffset;
		if ( BufferVar[ 0 ]=='%' )
			OtherVarName = ConvVarNameToSymbol( BufferVar );
		else
			OtherVarName = ConvSymbolToVarName( BufferVar );
		if ( OtherVarName )
				//gtk_tooltips_set_tip ( TooltipsEntryVarSpy[ NumSpy ], widget, OtherVarName, NULL );
		gtk_widget_set_sensitive( GTK_WIDGET(ModifyVarSpy[ NumSpy ]), TestVarIsReadWrite( NewVarType, NewVarOffset ) );
	}
	else
	{
		int OldType,OldOffset;
		/* Error Message */
		if (ErrorMessageVarParser)
			ShowMessageBoxError( ErrorMessageVarParser );
		else
			ShowMessageBoxError( _("Unknown variable...") );
		OldType = *NumVarSpy++;
		OldOffset = *NumVarSpy;
		/* put back old correct var */
		gtk_entry_set_text((GtkEntry *)widget,CreateVarName(OldType,OldOffset,InfosGene->DisplaySymbols));
	}
	return TRUE;
}

gint FreeVarsWindowDeleteEvent( GtkWidget * widget, GdkEvent * event, gpointer data )
{
	(void)widget;
	(void)event;
	(void)data;
	gtk_widget_hide( SpyFreeVarsWindow );
	SetToggleMenuForFreeVarsWindow( FALSE/*OpenedWin*/ );
	// we do not want that the window be destroyed.
	return TRUE;
}

// called per toggle action menu, or at startup (if window saved open or not)...
void OpenSpyFreeVarsWindow( GtkAction * ActionOpen, gboolean OpenIt )
{
	if ( ActionOpen!=NULL )
		OpenIt = gtk_toggle_action_get_active( GTK_TOGGLE_ACTION(ActionOpen) );
	if ( OpenIt )
	{
		gtk_widget_show( SpyFreeVarsWindow );
		gtk_window_present( GTK_WINDOW(SpyFreeVarsWindow) );
	}
	else
	{
		gtk_widget_hide( SpyFreeVarsWindow );
	}
}

// modified this to have 3 columns so we can display variable type
void FreeVarsWindowInitGtk( )
{
	GtkWidget * hboxfreevars[ NBR_FREE_VAR_SPY ], *vboxMain;
	long ColumnVar;
	int NumVarSpy,NumEntry,i=NBR_FREE_VAR_SPY;

	SpyFreeVarsWindow = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title ((GtkWindow *)SpyFreeVarsWindow, _("Watch Window"));
	vboxMain = gtk_vbox_new (FALSE, 0);
	gtk_container_add (GTK_CONTAINER (SpyFreeVarsWindow), vboxMain);
	gtk_widget_show (vboxMain);
	
	if ( NBR_WORDS < NBR_FREE_VAR_SPY) { i=NBR_WORDS ;}
	for(NumVarSpy=0; NumVarSpy<i; NumVarSpy++)
	{
		hboxfreevars[ NumVarSpy ] = gtk_hbox_new (FALSE, 0);
		gtk_container_add (GTK_CONTAINER (vboxMain), hboxfreevars[ NumVarSpy ]);
		gtk_widget_show (hboxfreevars[ NumVarSpy ]);

		for(ColumnVar=0; ColumnVar<3; ColumnVar++)
		{
			NumEntry = NumVarSpy+ColumnVar*NBR_FREE_VAR_SPY;			
			
			if ( ColumnVar==0)
			{
				LabelFreeVars[NumEntry] = gtk_label_new(NULL);				
				gtk_widget_set_size_request((GtkWidget *)LabelFreeVars[NumEntry],100,-1);
				gtk_box_pack_start (GTK_BOX (hboxfreevars[ NumVarSpy ]), LabelFreeVars[NumEntry], FALSE, FALSE, 0);
				gtk_widget_show (LabelFreeVars[NumEntry]);
			}
			if ( ColumnVar==1)
			{
				EntryVarSpy[ NumEntry ] = gtk_entry_new();
			        gtk_widget_show(EntryVarSpy[NumEntry]);
				gtk_box_pack_start (GTK_BOX( hboxfreevars[ NumVarSpy ] ), EntryVarSpy[ NumEntry ], TRUE, TRUE, 0);
				gtk_widget_set_size_request((GtkWidget *)EntryVarSpy[ NumEntry ],(ColumnVar==1)?80:110,-1);	
				char * VarName = CreateVarName(VarSpy[NumVarSpy][0],VarSpy[NumVarSpy][1],InfosGene->DisplaySymbols);
				//TooltipsEntryVarSpy[ NumVarSpy ] = gtk_tooltips_new();
				gtk_entry_set_text((GtkEntry *)EntryVarSpy[ NumEntry ],VarName);
				gtk_signal_connect(GTK_OBJECT (EntryVarSpy[ NumEntry ]), "activate",
                                GTK_SIGNAL_FUNC(EntryVarSpy_activate_event), (void *)(intptr_t)NumVarSpy);
			}
			if ( ColumnVar==2)
			{
				EntryVarSpy[ NumEntry ] = gtk_entry_new();
			        gtk_widget_show(EntryVarSpy[NumEntry]);
				gtk_box_pack_start (GTK_BOX( hboxfreevars[ NumVarSpy ] ), EntryVarSpy[ NumEntry ], TRUE, TRUE, 0);
				gtk_widget_set_size_request((GtkWidget *)EntryVarSpy[ NumEntry ],(ColumnVar==2)?80:110,-1);

			}

		}

		DisplayFormatVarSpy[NumVarSpy] = gtk_combo_box_text_new();
        gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(DisplayFormatVarSpy[NumVarSpy]), "Dec");
        gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(DisplayFormatVarSpy[NumVarSpy]), "Hex");
        gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(DisplayFormatVarSpy[NumVarSpy]), "Bin");
        gtk_combo_box_set_active(GTK_COMBO_BOX(DisplayFormatVarSpy[NumVarSpy]), 0);
		gtk_widget_set_size_request((GtkWidget *)DisplayFormatVarSpy[NumVarSpy],65,-1);
		gtk_box_pack_start (GTK_BOX(hboxfreevars[ NumVarSpy ]), DisplayFormatVarSpy[NumVarSpy], FALSE, FALSE, 0);
		gtk_widget_show(DisplayFormatVarSpy[NumVarSpy]);

		ModifyVarSpy[NumVarSpy] = gtk_button_new();
		gtk_button_set_image( GTK_BUTTON ( ModifyVarSpy[NumVarSpy] ),
                        gtk_image_new_from_stock (GTK_STOCK_EDIT, GTK_ICON_SIZE_SMALL_TOOLBAR) );
		gtk_box_pack_start (GTK_BOX(hboxfreevars[ NumVarSpy ]), ModifyVarSpy[NumVarSpy], FALSE, FALSE, 0);
		gtk_widget_show(ModifyVarSpy[NumVarSpy]);
		gtk_signal_connect( GTK_OBJECT(ModifyVarSpy[ NumVarSpy ]), "clicked",
                                        GTK_SIGNAL_FUNC(OpenModifyVarWindow_clicked_event), (void *)(intptr_t)NumVarSpy );
	}
	gtk_signal_connect( GTK_OBJECT(SpyFreeVarsWindow), "delete_event",
		GTK_SIGNAL_FUNC(FreeVarsWindowDeleteEvent), 0 );
}


void VarsWindowInitGtk()
{
	FreeVarsWindowInitGtk( );
	BoolVarsWindowInitGtk( );
	ModifyVarWindowInitGtk( );
}

