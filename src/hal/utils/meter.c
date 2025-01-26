/** This file, 'meter.c', is a GUI program that serves as a simple
    meter to look at HAL signals.  It is a user space component and
    uses GTK 3 for the GUI code.  It allows you to view one pin,
    signal, or parameter, and updates its display about 10 times
    per second.  (It is not a realtime program, and heavy loading
    can temporarily slow or stop the update.)  Clicking on the 'Select'
    button pops up a dialog that allows you to select what pin/signal/
    parameter you want to monitor.  Multiple instances of the program
    can be started if you want to monitor more than one item.  If you
    add "pin|sig|par[am] <name>" to the command line, the meter will
    initially display the pin/signal/parameter <name>, otherwise it
    will initially display nothing.
*/
/* Added ability to specify initial window position on command line 
    -g Xposition Yposiion    Chris Morley June 2009 
*/

/** Copyright (C) 2003 John Kasunich
                       <jmkasunich AT users DOT sourceforge DOT net>
*/

/** This program is free software; you can redistribute it and/or
    modify it under the terms of version 2 of the GNU General
    Public License as published by the Free Software Foundation.
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

    THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
    ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
    TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
    harming persons must have provisions for completely removing power
    from all motors, etc, before persons enter any danger area.  All
    machinery must be designed to comply with local and national safety
    codes, and the authors of this software can not, and do not, take
    any responsibility for such compliance.

    This code was written as part of the EMC HAL project.  For more
    information, go to https://linuxcnc.org.
*/

#include "config.h"
#include <locale.h>
#include <libintl.h>
#define _(x) gettext(x)

#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <ctype.h>
#include <string.h>

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "hal.h"		/* HAL public API decls */
#include "../hal_priv.h"	/* private HAL decls */
#include <rtapi_mutex.h>

#include <gtk/gtk.h>
#include "miscgtk.h"		/* generic GTK stuff */
#include <rtapi_string.h>

/***********************************************************************
*                            TYPEDEFS                                  *
************************************************************************/

/** a 'probe' is an object that references a HAL pin, signal, or
    parameter.  The user may select the item that is to be probed.
*/

#define PROBE_NAME_LEN 63

typedef struct {
    int listnum;		/* 0 = pin, 1 = signal, 2 = parameter */
    char *pickname;		/* name from list, not validated */
    hal_pin_t *pin;		/* metadata (if it's a pin) */
    hal_sig_t *sig;		/* metadata (if it's a signal) */
    hal_param_t *param;		/* metadata (if it's a parameter) */
    GtkWidget *window;		/* selection dialog window */
    GtkWidget *notebook;	/* pointer to the notebook */
    GtkWidget *lists[3];	/* lists for pins, sigs, and params */
    char probe_name[PROBE_NAME_LEN + 1];	/* name of this probe */
} probe_t;

typedef struct {
    probe_t *probe;		/* probe that locates the data */
    GtkWidget *value_label;	/* label object to display value */
    GtkWidget *name_label;	/* label object to display name */
    GtkWidget *button_select;	/* invokes the select dialog */
} meter_t;

/***********************************************************************
*                  GLOBAL VARIABLES DECLARATIONS                       *
************************************************************************/

/* Columns in the TreeView */
enum TREEVIEW_COLUMN {
    LIST_ITEM,
    NUM_COLS
};

int comp_id;			/* HAL component ID */

GtkWidget *main_window;
int small;

/***********************************************************************
*                  LOCAL FUNCTION PROTOTYPES                           *
************************************************************************/

static meter_t *meter_new(void);

/** 'probe_new()' creates a new probe structure.  It also creates
    a dialog window for the probe that allows the user to pick the
    pin, signal, or parameter that the probe will attach to.  It
    should be called during the init phase of the program, before
    the main event loop is started.
*/
static probe_t *probe_new(char *probe_name);

/** 'popup_probe_window()' is an event handler function that opens
    the selection dialog for a probe.  'data' must be a pointer to
    a probe_t structure that was allocated by 'probe_new'.
*/
static void popup_probe_window(GtkWidget * widget, gpointer data);

static void quit(int sig);
static void exit_from_hal(void);
static int refresh_value(gpointer data);
static char *data_value(int type, void *valptr);

static void create_probe_window(probe_t * probe);
static void apply_selection(GtkWidget * widget, gpointer data);
static void page_switched(GtkNotebook *notebook, GtkWidget *page,
                          guint page_num, gpointer user_data);
static void on_changed(GtkWidget *widget, gpointer data);

/***********************************************************************
*                        MAIN() FUNCTION                               *
************************************************************************/

int main(int argc, gchar * argv[])
{
    GtkWidget *vbox, *hbox;
    GtkWidget *button_select, *button_exit;
    char buf[30];
    int initial_type = 0, n, i, height, geometryflag = 0, xposition = 0, yposition = 0, width = 270;
    char *initial_name = NULL , *win_name;
    meter_t *meter;

    bindtextdomain("linuxcnc", EMC2_PO_DIR);
    setlocale(LC_MESSAGES,"");
    setlocale(LC_CTYPE,"");
    textdomain("linuxcnc");

    /* process and remove any GTK specific command line args */
    gtk_init(&argc, &argv);

    /* process my own command line args (if any) here */
    small = 0;
    n = 1;
    while ( argc > n ) {  
        if ( strcmp (argv[n], "-g") == 0 ) {
            /* This sets up the variables for initial position of window*/
            /* The last check is for the optional width request*/
	        geometryflag = 1;
	        n++;
            xposition =  atoi(argv[n]);
            n++;
            yposition =  atoi(argv[n]);
            n++;
            if ( argc > n ){
                rtapi_strxcpy(buf,argv[n]);
                for (i=0; i< (int)strlen(argv[n]); i++) {
                    if (isdigit(buf[i]) == 0) { break; } 
                }
                if ((int)strlen(argv[n]) == i){
                    width =  atoi(argv[n]);
                    n++;
                }
            } 
	    }
        if ((argc > n) && ( strcmp (argv[n], "-s") == 0 )) {
	        small = 1;
	        n++;
        }
        if (argc > n) {
	        /* check for user specified initial probe point */
	            if (strncmp(argv[n], "pin", 3) == 0) {
	                /* initial probe is a pin */
	                initial_type = 0;
	            } else if (strncmp(argv[n], "sig", 3) == 0) {
	                /* initial probe is a signal */
	                initial_type = 1;
	            } else if (strncmp(argv[n], "par", 3) == 0) {
	                /* initial probe is a parameter */
	                initial_type = 2;
	            } else {
	                printf(_("ERROR: '%s' is not a valid probe type\n"), argv[n]);
	                return -1;
	            }
	            n++;
	            if ( argc > n ) {
	                initial_name = argv[n];
                    n++;
	            } else {
	                printf(_("ERROR: no pin/signal/parameter name\n"));
	                return -1;
	            }	
        }     
    }
    if ((initial_name == NULL) && (small == 1)) {
        printf(_("ERROR: -s option requires a probe type and a pin/signal/parameter name\n"));
        return -1;
    }

    /* create a unique module name */
    snprintf(buf, 29, "halmeter-%d", getpid());
    /* connect to the HAL */
    comp_id = hal_init(buf);
    if (comp_id < 0) {
	return -1;
    }
    hal_ready(comp_id);
    /* register an exit function to disconnect from the HAL */
    atexit(exit_from_hal);
    /* capture INT (ctrl-C) and TERM signals */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);

    /* ideally this wouldn't be fixed size in pixels */
    if ( small ) {
	height = 22;
	win_name = initial_name;
    } else {
	height = 80;
	win_name = _("Hal Meter");
    }

    /* create main window, set it's size, title, and lock the size */
    main_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_widget_set_size_request(GTK_WIDGET(main_window), width, height);
    gtk_window_set_resizable(GTK_WINDOW(main_window), FALSE);
    gtk_window_set_keep_above(GTK_WINDOW(main_window),TRUE);
    gtk_window_set_title(GTK_WINDOW(main_window), win_name);

    /* this makes the application exit when the window is closed */
    g_signal_connect(main_window, "destroy",
            G_CALLBACK(gtk_main_quit), NULL);

    /* a vbox to hold the displayed value and the pin/sig/param name */
    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 3);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 2);
    gtk_container_add(GTK_CONTAINER(main_window), vbox);

    /* create a meter object */
    meter = meter_new();
    if (meter == NULL) {
    printf("null meter\n");
	exit(-1);
    }

    /* set up for initial probe, if any */
    if (initial_name != NULL) {
	meter->probe->pickname = initial_name;
	meter->probe->listnum = initial_type;
	apply_selection(NULL, meter->probe);
    }

    /* add the meter's value label to the vbox */
    gtk_box_pack_start(GTK_BOX(vbox), meter->value_label, TRUE, TRUE, 0);

    /* add the meter's name label to the vbox */
    if ( !small ) {
	gtk_box_pack_start(GTK_BOX(vbox), meter->name_label, TRUE, TRUE, 0);
    }

    /* arrange for periodic refresh of the value */
    g_timeout_add(100, refresh_value, meter);

    /* an hbox to hold the select and exit buttons */
    if ( !small ) {
	hbox = gtk_hbox_new_in_box(FALSE, 0, 0, vbox, FALSE, TRUE, 0);

	/* create the buttons and add them to the hbox */
	button_select = gtk_button_new_with_mnemonic(_("_Select"));
	button_exit = gtk_button_new_with_mnemonic(_("E_xit"));

	gtk_box_pack_start(GTK_BOX(hbox), button_select, TRUE, TRUE, 4);
	gtk_box_pack_start(GTK_BOX(hbox), button_exit, TRUE, TRUE, 4);

        /* make the application exit when the 'exit' button is clicked */
        g_signal_connect(button_exit, "clicked",
                G_CALLBACK(gtk_main_quit), NULL);

        /* activate selection window when the 'select' button is clicked */
        g_signal_connect(button_select, "clicked",
                G_CALLBACK(popup_probe_window), meter->probe);

	/* save reference to select button */
	meter->button_select = button_select;
    }

    /* The interface is now set up so we show the window and
       enter the gtk_main loop. */
    gtk_widget_show_all(main_window);
    /* If the -g option was invoked: set position */
    if (geometryflag == 1) {
        gtk_window_move(GTK_WINDOW(main_window),xposition,yposition);
    }
    gtk_main();

    return (0);
}

/***********************************************************************
*                         LOCAL FUNCTION CODE                          *
************************************************************************/

static meter_t *meter_new(void)
{
    meter_t *new;

    /* allocate a meter object for the display */
    new = malloc(sizeof(meter_t));
    if (new == NULL) {
	return NULL;
    }
    /* define a probe for the display item */
    new->probe = probe_new(_("Select Item to Probe"));
    if (new->probe == NULL) {
	free(new);
	return NULL;
    }
    /* create a label widget to hold the value */
    new->value_label = gtk_label_new("----");
    /* center justify text, no wordwrap */
    gtk_label_set_justify(GTK_LABEL(new->value_label), GTK_JUSTIFY_CENTER);
    gtk_label_set_line_wrap(GTK_LABEL(new->value_label), FALSE);

    /* create a label widget to hold the name */
    if ( !small ) {
	new->name_label = gtk_label_new("------");
	/* center justify text, no wordwrap */
	gtk_label_set_justify(GTK_LABEL(new->name_label),
	    GTK_JUSTIFY_CENTER);
	gtk_label_set_line_wrap(GTK_LABEL(new->name_label), FALSE);
    }
    return new;
}

probe_t *probe_new(char *probe_name)
{
    probe_t *new;

    if (probe_name == NULL) {
	/* no name specified, fake it */
	probe_name = _("Select Item to Probe");
    }
    /* allocate a new probe structure */
    new = malloc(sizeof(probe_t));
    if (new == NULL) {
	return NULL;
    }
    /* init the fields */
    new->pickname = NULL;
    new->listnum = -1;
    new->pin = NULL;
    new->sig = NULL;
    new->param = NULL;
    strncpy(new->probe_name, probe_name, HAL_NAME_LEN);
    new->probe_name[HAL_NAME_LEN] = '\0';
    /* window will be created just before it is displayed */
    new->window = NULL;
    /* done */
    return new;
}

void popup_probe_window(GtkWidget * widget, gpointer data)
{
    (void)widget;
    probe_t *probe;
    hal_pin_t *pin;
    hal_sig_t *sig;
    hal_param_t *param;

    int next, row, match_row, tab, match_tab;
    char *name[HAL_NAME_LEN + 1];


    /* get a pointer to the probe data structure */
    probe = (probe_t *) data;

    /* create window if needed */
    if (probe->window == NULL) {
        create_probe_window(probe);
    } else {
        gtk_window_present(GTK_WINDOW(probe->window));
    }

    /*
     * This part clears the list, then add all items back into the list.
     * If a pin, signal, or parameter is showing in the main window, that item
     * should be selected in the "Select item to probe" window, when the window
     * is displayed again.
     *
     * Changing the tab (page) and selecting the item (row) needs to be done after
     * the window is displayed.
     */
    clear_list(probe->lists[0]);
    clear_list(probe->lists[1]);
    clear_list(probe->lists[2]);

    rtapi_mutex_get(&(hal_data->mutex));
    next = hal_data->pin_list_ptr;
    match_tab = 0;
    match_row = 0;
    row = 0;
    tab = 0;
    while (next != 0) {
        pin = SHMPTR(next);
        *name = pin->name;

        add_to_list(probe->lists[tab], name, NUM_COLS);
        if (probe->pin == pin) {
            match_tab = tab;
            match_row = row;
        }
        next = pin->next_ptr;
        row++;
    }

    next = hal_data->sig_list_ptr;
    row = 0;
    tab = 1;
    while (next != 0) {
        sig = SHMPTR(next);
        *name = sig->name;

        add_to_list(probe->lists[tab], name, NUM_COLS);
        if (probe->sig == sig) {
            match_tab = tab;
            match_row = row;
        }
        next = sig->next_ptr;
        row++;
    }

    next = hal_data->param_list_ptr;
    row = 0;
    tab = 2;
    while (next != 0) {
        param = SHMPTR(next);
        *name = param->name;

        add_to_list(probe->lists[tab], name, NUM_COLS);
        if (probe->param == param) {
            match_tab = tab;
            match_row = row;
        }
        next = param->next_ptr;
        row++;
    }

    rtapi_mutex_give(&(hal_data->mutex));
    gtk_widget_show_all(probe->window);

    if (probe->pickname != NULL) {
        gtk_notebook_set_current_page(GTK_NOTEBOOK(probe->notebook), match_tab);
        mark_selected_row(probe->lists[match_tab], match_row);
    }
}

static void quit(int sig)
{
    (void)sig;
    gtk_main_quit();
}

static void exit_from_hal(void)
{
    hal_exit(comp_id);
}

/* this function refreshes the value display */
static int refresh_value(gpointer data)
{
    meter_t *meter;
    probe_t *probe;
    char *value_str, *name_str;
    hal_sig_t *sig;
    static int first = 1;

    meter = (meter_t *) data;
    probe = meter->probe;

    if ( first ) {
	first = 0;
	if ( probe->pickname == NULL ) {
	    g_signal_emit_by_name(meter->button_select, "clicked");
	}
    }

    rtapi_mutex_get(&(hal_data->mutex));
    if (probe->pin != NULL) {
	if (probe->pin->name[0] == '\0') {
	    /* pin has been deleted, can't display it any more */
	    probe->pin = NULL;
	    rtapi_mutex_give(&(hal_data->mutex));
	    return 1;
	}
	name_str = probe->pin->name;
	if (probe->pin->signal == 0) {
	    /* pin is unlinked, get data from dummysig */
	    value_str = data_value(probe->pin->type, &(probe->pin->dummysig));
	} else {
	    /* pin is linked to a signal */
	    sig = SHMPTR(probe->pin->signal);
	    value_str = data_value(probe->pin->type, SHMPTR(sig->data_ptr));
	}
    } else if (probe->sig != NULL) {
	if (probe->sig->name[0] == '\0') {
	    /* signal has been deleted, can't display it any more */
	    probe->sig = NULL;
	    rtapi_mutex_give(&(hal_data->mutex));
	    return 1;
	}
	name_str = probe->sig->name;
	value_str =
	    data_value(probe->sig->type, SHMPTR(probe->sig->data_ptr));
    } else if (probe->param != NULL) {
	if (probe->param->name[0] == '\0') {
	    /* parameter has been deleted, can't display it any more */
	    probe->param = NULL;
	    rtapi_mutex_give(&(hal_data->mutex));
	    return 1;
	}
	name_str = probe->param->name;
	value_str =
	    data_value(probe->param->type, SHMPTR(probe->param->data_ptr));
    } else {
	name_str = "-----";
	value_str = "---";
    }
    rtapi_mutex_give(&(hal_data->mutex));
    gtk_label_set_text(GTK_LABEL(meter->value_label), value_str);
    if (!small) {
	gtk_label_set_text(GTK_LABEL(meter->name_label), name_str);
    }
    return 1;
}

/* Switch function to return var value for the print_*_list functions  */
static char *data_value(int type, void *valptr)
{
    char *value_str;
    static char buf[25];

    switch (type) {
    case HAL_BIT:
	if (*((char *) valptr) == 0)
	    value_str = "FALSE";
	else
	    value_str = "TRUE";
	break;
    case HAL_FLOAT:
	snprintf(buf, 14, "%.7g", (double)*((hal_float_t *) valptr));
	value_str = buf;
	break;
    case HAL_S32:
	snprintf(buf, 24, "%10ld", (long)*((hal_s32_t *) valptr));
	value_str = buf;
	break;
    case HAL_U32:
	snprintf(buf, 24, "%10lu (0x%08lX)", (unsigned long)*((hal_u32_t *) valptr),
	    *((unsigned long *) valptr));
	value_str = buf;
	break;
    default:
	/* Shouldn't get here, but just in case... */
	value_str = "";
    }
    return value_str;
}

static void create_probe_window(probe_t * probe)
{
    GtkWidget *vbox, *hbox;
    GtkWidget *label;
    GtkWidget *button_close;
    GtkWidget *scrolled_window;
    GtkTreeSelection *selection;

    char *tab_label_text[3];
    int n;

    /* create window, set size and title */
    probe->window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_widget_set_size_request(GTK_WIDGET(probe->window), -1, 400);
    gtk_window_set_title(GTK_WINDOW(probe->window), probe->probe_name);

    /* a box to hold everything, add it to window */
    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 3);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 2);
    gtk_container_add(GTK_CONTAINER(probe->window), vbox);

    /* create a notebook to hold pin, signal, and parameter list */
    probe->notebook = gtk_notebook_new();
    gtk_box_pack_start(GTK_BOX(vbox), probe->notebook, TRUE, TRUE, 0);

    /* text for tab labels */
    tab_label_text[0] = _(" _Pins ");
    tab_label_text[1] = _(" _Signals ");
    tab_label_text[2] = _(" Para_meters ");

    /* loop to create three identical tabs */
    for (n = 0; n < 3; n++) {
        /* Create a scrolled window to display the list */
        scrolled_window = gtk_scrolled_window_new(NULL, NULL);
        gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled_window),
            GTK_POLICY_AUTOMATIC, GTK_POLICY_ALWAYS);

        /* create and set tabs in notebook */
        label = gtk_label_new_with_mnemonic(tab_label_text[n]);
        gtk_widget_set_size_request(label, 70, -1);
        gtk_notebook_append_page(GTK_NOTEBOOK(probe->notebook), scrolled_window, label);

        /* create and initialize the list to hold the data */
        probe->lists[n] = gtk_tree_view_new();
        gtk_tree_view_set_headers_visible(
                GTK_TREE_VIEW(probe->lists[n]), FALSE);
        init_list(probe->lists[n], &tab_label_text[n], NUM_COLS);
        selection = gtk_tree_view_get_selection(GTK_TREE_VIEW(probe->lists[n]));
        gtk_container_add(GTK_CONTAINER(scrolled_window), probe->lists[n]);

        /* signals related to the lists */
        g_signal_connect_swapped(probe->lists[n], "row-activated",
                G_CALLBACK(gtk_widget_destroy), probe->window);
        g_signal_connect(selection, "changed",
                G_CALLBACK(on_changed), probe);
    }

    /* create a box and a close button */
    hbox = gtk_hbox_new_in_box(TRUE, 0, 0, vbox, FALSE, TRUE, 0);
    button_close = gtk_button_new_with_mnemonic(_("_Close"));
    gtk_box_pack_start(GTK_BOX(hbox), button_close, TRUE, TRUE, 4);

    /* signals */
    g_signal_connect(probe->notebook, "switch-page",
            G_CALLBACK(page_switched), probe);
    g_signal_connect_swapped(button_close, "clicked",
            G_CALLBACK(gtk_widget_destroy), probe->window);
    g_signal_connect(probe->window, "destroy",
            G_CALLBACK(gtk_widget_destroyed), &(probe->window));
}

static void apply_selection(GtkWidget * widget, gpointer data)
{
    (void)widget;
    probe_t *probe;

    /* get a pointer to the probe data structure */
    probe = (probe_t *) data;
    /* discard info about previous item */
    probe->pin = NULL;
    probe->sig = NULL;
    probe->param = NULL;
    if (probe->pickname == NULL) {
	/* not a valid selection */
	/* should pop up a message or something here, instead we ignore it */
	return;
    }
    if (probe->listnum == 0) {
	/* search the pin list */
	probe->pin = halpr_find_pin_by_name(probe->pickname);
    } else if (probe->listnum == 1) {
	/* search the signal list */
	probe->sig = halpr_find_sig_by_name(probe->pickname);
    } else if (probe->listnum == 2) {
	/* search the parameter list */
	probe->param = halpr_find_param_by_name(probe->pickname);
    }
    /* at this point, the probe structure contain a pointer to the item we
       wish to display, or all three are NULL if the item doesn't exist */
}

/* Keeps track of which tab was last used. */
static void page_switched(GtkNotebook *notebook, GtkWidget *page,
                          guint page_num, gpointer user_data)
{
    (void)notebook;
    (void)page;
    probe_t *probe;
    probe = (probe_t *) user_data;
    probe->listnum=page_num;
}

/*
 * This gets triggered with the "changed" event. Get the name of the selected
 * row and updates probe->pickname.
 *
 * Calls the function apply_selection to update the label in the main window.
 */
static void on_changed(GtkWidget *widget,  gpointer data)
{
    probe_t *probe;
    probe = (probe_t *) data;

    GtkTreeIter iter;
    GtkTreeModel *model;

    if (gtk_tree_selection_get_selected(GTK_TREE_SELECTION(widget), &model, &iter)) {
        gtk_tree_model_get(model, &iter, LIST_ITEM, &(probe->pickname), -1);
        apply_selection(GTK_WIDGET(probe->window), probe);
    }
}
