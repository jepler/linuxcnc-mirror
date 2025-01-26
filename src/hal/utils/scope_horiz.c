/** This file, 'scope_horiz.c', contains the portion of halscope
    that deals with horizontal stuff - sample rate, scaling,
    position and such.
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
#include <ctype.h>
#include <string.h>

#include "rtapi.h"		/* RTAPI realtime OS API */
#include <rtapi_mutex.h>
#include "hal.h"		/* HAL public API decls */
#include "../hal_priv.h"	/* private HAL decls */

#include <gtk/gtk.h>
#include "miscgtk.h"		/* generic GTK stuff */
#include "scope_usr.h"		/* scope related declarations */

#define BUFLEN 80		/* length for sprintf buffers */

/***********************************************************************
*                  GLOBAL VARIABLES DECLARATIONS                       *
************************************************************************/

/* Columns in the TreeView */
enum TREEVIEW_COLUMN {
    COL_THREAD,
    COL_PERIOD,
    NUM_COLS
};

/***********************************************************************
*                  LOCAL FUNCTION PROTOTYPES                           *
************************************************************************/

static void init_horiz_window(void);
static void init_acquire_function(void);
static void acquire_popup(GtkWidget * widget, gpointer gdata);

static void dialog_realtime_not_loaded(void);
static void dialog_realtime_not_linked(void);
static void dialog_realtime_not_running(void);
static void log_popup(GtkWindow *parent);
static void acquire_selection_made(GtkWidget *widget, gpointer data);
static int set_sample_thread_name(char *name);
static int activate_sample_thread(void);
static void deactivate_sample_thread(void);

static void mult_changed(GtkAdjustment * adj, gpointer gdata);
static void zoom_changed(GtkAdjustment * adj, gpointer gdata);
static void pos_changed(GtkAdjustment * adj, gpointer gdata);
static void rec_len_button(GtkWidget * widget, gpointer gdata);

static void calc_horiz_scaling(void);

static void refresh_horiz_info(void);
static gboolean refresh_pos_disp(void);

/* helper functions */
static void format_time_value(char *buf, int buflen, double timeval);
static void format_freq_value(char *buf, int buflen, double freqval);

/* manipulation functions */
static gint horiz_press(GtkWidget *widget, GdkEventButton *event);
static gint horiz_release(GtkWidget *widget, GdkEventButton *event);
static gint horiz_motion(GtkWidget *widget, GdkEventMotion *event);

static gboolean configure_window(GtkWidget *widget, GdkEventConfigure *event,
                                 gpointer data);

/***********************************************************************
*                       PUBLIC FUNCTIONS                               *
************************************************************************/

void init_horiz(void)
{
    /* stop sampling */
    ctrl_shm->state = IDLE;
    /* init non-zero members of the horizontal structure */
    /* set up the window */
    init_horiz_window();
    /* set up the realtime function */
    init_acquire_function();
    /* make sure displays are up to date */
    calc_horiz_scaling();
    refresh_horiz_info();
}

static void init_horiz_window(void)
{
    scope_horiz_t *horiz;
    GtkWidget *hbox, *vbox;

    horiz = &(ctrl_usr->horiz);

    /* upper region - main display */
    hbox =
	gtk_hbox_new_in_box(FALSE, 0, 0, ctrl_usr->horiz_info_win, FALSE,
	TRUE, 0);
    /* first column - slider labels */
    vbox = gtk_vbox_new_in_box(TRUE, 0, 0, hbox, FALSE, TRUE, 3);
    gtk_label_new_in_box(_("Zoom"), vbox, FALSE, TRUE, 0);
    gtk_label_new_in_box(_(" Pos "), vbox, FALSE, TRUE, 0);
    /* second column - sliders */
    vbox = gtk_vbox_new_in_box(TRUE, 0, 0, hbox, TRUE, TRUE, 3);
    /* add a slider for zoom level */
    horiz->zoom_adj = gtk_adjustment_new(1, 1, 9, 1, 1, 0);
    horiz->zoom_slider = gtk_scale_new(
            GTK_ORIENTATION_HORIZONTAL, GTK_ADJUSTMENT(horiz->zoom_adj));
    gtk_scale_set_digits(GTK_SCALE(horiz->zoom_slider), 0);
    gtk_scale_set_draw_value(GTK_SCALE(horiz->zoom_slider), FALSE);
    gtk_box_pack_start(GTK_BOX(vbox), horiz->zoom_slider, FALSE, FALSE, 0);
    /* store the current value of the slider */
    horiz->zoom_setting = gtk_adjustment_get_value(GTK_ADJUSTMENT(horiz->zoom_adj));
    /* connect the slider to a function that re-calcs horizontal scaling */
    g_signal_connect(horiz->zoom_adj, "value_changed",
	G_CALLBACK(zoom_changed), NULL);
    gtk_widget_show(horiz->zoom_slider);
    /* add a slider for position control */
    horiz->pos_adj = gtk_adjustment_new(500, 0, 1000, 1, 1, 0);
    horiz->pos_slider = gtk_scale_new(
            GTK_ORIENTATION_HORIZONTAL, GTK_ADJUSTMENT(horiz->pos_adj));
    gtk_scale_set_digits(GTK_SCALE(horiz->pos_slider), 0);
    gtk_scale_set_draw_value(GTK_SCALE(horiz->pos_slider), FALSE);
    gtk_box_pack_start(GTK_BOX(vbox), horiz->pos_slider, FALSE, FALSE, 0);
    /* store the current value of the slider */
    horiz->pos_setting = gtk_adjustment_get_value(GTK_ADJUSTMENT(horiz->pos_adj)) / 1000.0;
    /* connect the slider to a function that re-calcs horizontal position */
    g_signal_connect(horiz->pos_adj, "value_changed",
	G_CALLBACK(pos_changed), NULL);
    gtk_widget_show(horiz->pos_slider);
    /* third column - scale display */
    horiz->scale_label = gtk_label_new_in_box("----", hbox, FALSE, FALSE, 5);
    gtk_label_size_to_fit(GTK_LABEL(horiz->scale_label),
	"99.9 ms\nper div");
    /* fourth column - record length and sample rate button */
    horiz->record_button =
	gtk_button_new_with_label(_("----- Samples\nat ---- kHz"));
    horiz->record_label = gtk_bin_get_child(GTK_BIN(horiz->record_button));
    gtk_label_size_to_fit(GTK_LABEL(horiz->record_label),
	"99999 Samples\nat 99.9 MHz");
    gtk_box_pack_start(GTK_BOX(hbox), horiz->record_button, FALSE, FALSE, 0);
    /* activate the acquire menu if button is clicked */
    g_signal_connect(horiz->record_button, "clicked",
	G_CALLBACK(acquire_popup), NULL);
    gtk_widget_show(horiz->record_button);
    /* lower region, graphical status display */
    gtk_hseparator_new_in_box(ctrl_usr->horiz_info_win, 0);
    hbox =
	gtk_hbox_new_in_box(FALSE, 0, 0, ctrl_usr->horiz_info_win, FALSE,
	TRUE, 0);
    /* graphic horizontal display */
    horiz->disp_area = gtk_drawing_area_new();

    g_signal_connect(horiz->disp_area, "configure_event",
            G_CALLBACK(configure_window), NULL);
    g_signal_connect(horiz->disp_area, "draw",
            G_CALLBACK(refresh_pos_disp), NULL);
    g_signal_connect(horiz->disp_area, "button_press_event",
        G_CALLBACK(horiz_press), 0);
    g_signal_connect(horiz->disp_area, "button_release_event",
        G_CALLBACK(horiz_release), 0);
    g_signal_connect(horiz->disp_area, "motion_notify_event",
        G_CALLBACK(horiz_motion), 0);
    gtk_widget_set_events(GTK_WIDGET(horiz->disp_area),
        GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK
        | GDK_POINTER_MOTION_MASK | GDK_POINTER_MOTION_HINT_MASK);
    gtk_box_pack_start(GTK_BOX(hbox), horiz->disp_area, TRUE, TRUE, 0);
    gtk_widget_show(horiz->disp_area);
    /* label for state */
    gtk_vseparator_new_in_box(hbox, 3);
    vbox = gtk_vbox_new_in_box(FALSE, 0, 0, hbox, FALSE, TRUE, 3);
    horiz->state_label =
	gtk_label_new_in_box(" ---- ", vbox, FALSE, FALSE, 3);
    gtk_label_size_to_fit(GTK_LABEL(horiz->state_label), " TRIGGERED ");
}

static void init_acquire_function(void)
{
    hal_funct_t *funct;
    int next_thread;
    hal_thread_t *thread;
    hal_list_t *list_root, *list_entry;
    hal_funct_entry_t *fentry;
    scope_horiz_t *horiz;

    horiz = &(ctrl_usr->horiz);
    /* set watchdog to trip immediately once heartbeat funct is called */
    ctrl_shm->watchdog = 10;
    /* is the realtime function present? */
    funct = halpr_find_funct_by_name("scope.sample");
    if (funct == NULL) {
	/* realtime function not present - watchdog timeout will open a
	   dialog asking the user to insmod the module */
	return;
    }
    if (funct->users == 0) {
	/* function not in use - watchdog timeout will open a dialog asking
	   the user to select a thread */
	return;
    }
    /* function is in use, find out which thread it is linked to */
    rtapi_mutex_get(&(hal_data->mutex));
    next_thread = hal_data->thread_list_ptr;
    while (next_thread != 0) {
	thread = SHMPTR(next_thread);
	list_root = &(thread->funct_list);
	list_entry = list_next(list_root);
	while (list_entry != list_root) {
	    fentry = (hal_funct_entry_t *) list_entry;
	    if (funct == SHMPTR(fentry->funct_ptr)) {
		/* found a match, update structure members */
		horiz->thread_name = thread->name;
		horiz->thread_period_ns = thread->period;
		/* done with hal data */
		rtapi_mutex_give(&(hal_data->mutex));
		/* reset watchdog to give RT code some time */
		ctrl_shm->watchdog = 1;
		return;
	    }
	    list_entry = list_next(list_entry);
	}
	next_thread = thread->next_ptr;
    }
    /* didn't find a linked thread - should never get here, but... */
    rtapi_mutex_give(&(hal_data->mutex));
    return;
}

void handle_watchdog_timeout(void)
{
    hal_funct_t *funct;

    /* stop sampling */
    ctrl_shm->state = IDLE;
    ctrl_shm->samples = 0;
    /* does realtime function exist? */
    funct = halpr_find_funct_by_name("scope.sample");
    if (funct == NULL) {
	/* function is not loaded */
	dialog_realtime_not_loaded();
    } else if (funct->users == 0) {
	/* function is loaded, but not in a thread */
	dialog_realtime_not_linked();
    } else if (ctrl_shm->watchdog != 0) {
	/* function is in a thread, but thread is not running */
	dialog_realtime_not_running();
    } else {
	/* everything should be fine... */
	return;
    }
}

void refresh_state_info(void)
{
    scope_horiz_t *horiz;
    static const gchar *state_names[] = { "IDLE",
	"INIT",
	"PRE-TRIG",
	"TRIGGER?",
	"TRIGGERED",
	"DONE",
	"RESET"
    };

    horiz = &(ctrl_usr->horiz);
    if (ctrl_shm->state > RESET) {
	ctrl_shm->state = IDLE;
    }
    gtk_label_set_text_if(horiz->state_label, state_names[ctrl_shm->state]);
}

void write_horiz_config(FILE *fp)
{
    scope_horiz_t *horiz;

    horiz = &(ctrl_usr->horiz);
    fprintf(fp, "THREAD %s\n", horiz->thread_name);
    fprintf(fp, "MAXCHAN %d\n", ctrl_shm->sample_len);
    fprintf(fp, "HMULT %d\n", ctrl_shm->mult);
    fprintf(fp, "HZOOM %d\n", horiz->zoom_setting);
    fprintf(fp, "HPOS %e\n", horiz->pos_setting);
}

int set_sample_thread(char *name)
{
    int rv;

    /* This is broken into two parts.  When called directly
       while reading config file commands, both execute in
       order.  however, when the dialog is running, it calls
       the two separately, setting the sample_thread_name
       during the dialog, and sctivating it when the dialog
       is closed.  This may not be necessary, but that is
       how it works right now. */
    rv = set_sample_thread_name(name);
    if ( rv < 0 ) {
	return rv;
    }
    rv = activate_sample_thread();
    return rv;
}

int set_rec_len(int setting)
{
    int count, n;

    switch ( setting ) {
    case 1:
    case 2:
    case 4:
    case 8:
    case 16:
	/* acceptable value */
	break;
    default:
	/* bad value */
	return -1;
    }
    /* count enabled channels */
    count = 0;
    for (n = 0; n < 16; n++) {
	if (ctrl_usr->vert.chan_enabled[n]) {
	    count++;
	}
    }
    if (count > setting) {
	/* too many channels already enabled */
	return -1;
    }
    ctrl_shm->sample_len = setting;
    ctrl_shm->rec_len = ctrl_shm->buf_len / ctrl_shm->sample_len;
    calc_horiz_scaling();
    refresh_horiz_info();
    return 0;
}

int set_horiz_mult(int setting)
{
    scope_horiz_t *horiz;
    long period_ns, max_mult;

    /* validate setting */
    if ( setting < 1 ) {
	return -1;
    }
    /* point to data */
    horiz = &(ctrl_usr->horiz);
    /* get period, make sure it is valid */
    period_ns = horiz->thread_period_ns;
    if ( period_ns < 10 ) {
	return -1;
    }
    /* calc max possible mult (to keep sample period <= 1 sec */
    max_mult = 1000000000 / period_ns;
    if (max_mult > 1000) {
	max_mult = 1000;
    }
    /* make sure we aren't too high */
    if ( setting > max_mult ) {
	setting = max_mult;
    }
    /* save new value */
    ctrl_shm->mult = setting;
    /* refresh other stuff */
    calc_horiz_scaling();
    refresh_horiz_info();
    return 0;
}

int set_horiz_zoom(int setting)
{
    scope_horiz_t *horiz;
    GtkAdjustment *adj;

    /* range check setting */
    if (( setting < 1 ) || ( setting > 9 )) {
	return -1;
    }
    /* point to data */
    horiz = &(ctrl_usr->horiz);
    /* save new value */
    horiz->zoom_setting = setting;
    /* set zoom slider based on new setting */
    adj = GTK_ADJUSTMENT(horiz->zoom_adj);
    gtk_adjustment_set_value(adj, setting);
    /* refresh other stuff */
    calc_horiz_scaling();
    refresh_horiz_info();
    request_display_refresh(1);
    return 0;
}

int set_horiz_pos(double setting)
{
    scope_horiz_t *horiz;
    GtkAdjustment *adj;

    /* range check setting */
    if (( setting < 0.0 ) || ( setting > 1.0 )) {
	return -1;
    }
    /* point to data */
    horiz = &(ctrl_usr->horiz);
    /* save new value */
    horiz->pos_setting = setting;
    /* set position slider based on new setting */
    adj = GTK_ADJUSTMENT(horiz->pos_adj);
    gtk_adjustment_set_value(adj, setting * 1000);
    /* refresh other stuff */
    refresh_horiz_info();
    request_display_refresh(1);
    return 0;
}

/***********************************************************************
*                       LOCAL FUNCTIONS                                *
************************************************************************/

static void dialog_realtime_not_loaded(void)
{
    int retval;
    static int first_time=1;
    GtkWidget *dialog;

    if(first_time) {
        first_time = 0;
        if(system(EMC2_BIN_DIR "/halcmd loadrt scope_rt") == 0) {
            sleep(1);
            return;
        }
    }
    dialog = gtk_message_dialog_new(GTK_WINDOW(ctrl_usr->main_win),
                                    GTK_DIALOG_MODAL,
                                    GTK_MESSAGE_INFO,
                                    GTK_BUTTONS_NONE,
                                    _("Realtime component not loaded"));
    gtk_message_dialog_format_secondary_text(
            GTK_MESSAGE_DIALOG(dialog),
            _("HALSCOPE uses a realtime component called scope_rt'\n"
            "to sample signals for display.  It is not currently loaded\n"
            "and attempting to load it automatically failed.  More information\n"
            "may be available in the terminal where halscope was started.\n\n"
            "Please do one of the following:\n\n"
            "Load the component (using 'halcmd loadrt scope_rt'), then click 'OK'\n"
            "or\n" "Click 'Quit' to exit HALSCOPE"));
    gtk_dialog_add_buttons(GTK_DIALOG(dialog),
                           _("OK"), GTK_RESPONSE_OK,
                           _("Quit"), GTK_RESPONSE_CLOSE,
                           NULL);
    retval = gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);

    if (retval == GTK_RESPONSE_CLOSE) {
        /* user pressed quit - end the program */
        gtk_main_quit();
    }
}

static void dialog_realtime_not_linked(void)
{
    scope_horiz_t *horiz;

    int next, sel_row, n;
    int retval;
    double period;
    hal_thread_t *thread;
    char *strs[2];
    char buf[BUFLEN + 1];
    GtkWidget *hbox, *label;
    GtkWidget *content_area;
    GtkWidget *dialog;
    GtkWidget *buttons[5];
    GtkWidget *scrolled_window;
    GtkTreeSelection *selection;

    char *titles[NUM_COLS];
    const char *title, *msg;

    horiz = &(ctrl_usr->horiz);
    if (horiz->thread_name == NULL) {
	title = _("Realtime function not linked");
	msg = _("The HALSCOPE realtime sampling function\n"
	    "must be called from a HAL thread in to\n"
	    "determine the sampling rate.\n\n"
	    "Please do one of the following:\n\n"
	    "Select a thread name and multiplier then click 'OK'\n"
	    "or\n" "Click 'Quit' to exit HALSCOPE");
    } else {
	title = _("Select Sample Rate");
	msg = _("Select a thread name and multiplier then click 'OK'\n"
	    "or\n" "Click 'Quit' to exit HALSCOPE");
    }
    /* create dialog window, disable resizing and set title */
    dialog = gtk_dialog_new_with_buttons(title,
                                         GTK_WINDOW(ctrl_usr->main_win),
                                         GTK_DIALOG_MODAL,
                                         _("_OK"), GTK_RESPONSE_OK,
                                         _("Quit"), GTK_RESPONSE_CLOSE,
                                         NULL);
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));

    /* display message */
    label = gtk_label_new(msg);
    gtk_widget_set_margin_start(label, 15);
    gtk_widget_set_margin_end(label, 15);
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
            label, FALSE, TRUE, 5);

    /* a separator */
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
            gtk_separator_new(GTK_ORIENTATION_HORIZONTAL), FALSE, FALSE , 0);

    /* thread name display */
    hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_set_homogeneous(GTK_BOX(hbox), TRUE);
    gtk_label_new_in_box(_("Thread:"), hbox, TRUE, TRUE, 0);
    horiz->thread_name_label =
	gtk_label_new_in_box("------", hbox, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
            hbox, FALSE, TRUE, 0);

    /* sample period display */
    hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_set_homogeneous(GTK_BOX(hbox), TRUE);
    gtk_label_new_in_box(_("Sample Period:"), hbox, TRUE, TRUE, 0);
    horiz->sample_period_label =
	gtk_label_new_in_box("------", hbox, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
            hbox, FALSE, TRUE, 0);

    /* sample rate display */
    hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_set_homogeneous(GTK_BOX(hbox), TRUE);
    gtk_label_new_in_box(_("Sample Rate:"), hbox, TRUE, TRUE, 0);
    horiz->sample_rate_label =
	gtk_label_new_in_box("------", hbox, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
            hbox, FALSE, TRUE, 0);

    /* a separator */
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
            gtk_separator_new(GTK_ORIENTATION_HORIZONTAL), FALSE, FALSE , 0);

    /* Create a scrolled window to display the thread list */
    scrolled_window = gtk_scrolled_window_new(NULL, NULL);
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
                    scrolled_window, TRUE, TRUE, 5);

    /* create a list to hold the threads */
    titles[0] = _("Thread");
    titles[1] = _("Period");
    horiz->thread_list = gtk_tree_view_new();
    init_list(horiz->thread_list, titles, NUM_COLS);
    selection = gtk_tree_view_get_selection(GTK_TREE_VIEW(horiz->thread_list));
    gtk_tree_selection_set_mode(GTK_TREE_SELECTION(selection),
            GTK_SELECTION_BROWSE);
    gtk_container_add(GTK_CONTAINER(scrolled_window), horiz->thread_list);

    /* set up a callback for when the user selects a line */
    g_signal_connect(selection, "changed",
            G_CALLBACK(acquire_selection_made), horiz);

    /* get mutex before traversing list */
    rtapi_mutex_get(&(hal_data->mutex));
    n = 0;
    sel_row = -1;
    next = hal_data->thread_list_ptr;
    while (next != 0) {
	thread = SHMPTR(next);
	/* check thread period */
	if (thread->period <= 1000000000) {
	    /* period is less than 1 sec, add to list */
	    period = thread->period / 1000000000.0;
	    /* create a string for display */
	    format_time_value(buf, BUFLEN, period);
	    strs[1] = buf;
	    /* get thread name */
	    strs[0] = thread->name;
	    /* add to list */
	    add_to_list(horiz->thread_list, strs, NUM_COLS);
	    if ((horiz->thread_name != NULL)
		&& (strcmp(horiz->thread_name, thread->name) == 0)) {
		/* found the current thread, remember it's row number */
		sel_row = n;
		/* and make sure thread_period is correct */
		horiz->thread_period_ns = thread->period;
	    }
	}
	n++;
	next = thread->next_ptr;
    }

    rtapi_mutex_give(&(hal_data->mutex));

    /* set up the the layout for the multiplier spinbutton */
    hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_set_homogeneous(GTK_BOX(hbox), TRUE);
    gtk_label_new_in_box(_("Multiplier:"), hbox, FALSE, FALSE, 0);
    /* set up the multiplier spinbutton - ranges from every run of the
       thread, to every 1000th run */
    horiz->mult_adj =
	gtk_adjustment_new(ctrl_shm->mult, 1, 1000, 1, 1, 0);
    horiz->mult_spinbutton =
	gtk_spin_button_new(GTK_ADJUSTMENT(horiz->mult_adj), 1, 0);
    gtk_box_pack_start(GTK_BOX(hbox), horiz->mult_spinbutton, FALSE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
                    hbox, FALSE, TRUE, 0);

    /* connect the multiplier spinbutton to a function */
    g_signal_connect(horiz->mult_adj, "value_changed",
	G_CALLBACK(mult_changed), NULL);

    /* a separator */
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
            gtk_separator_new(GTK_ORIENTATION_HORIZONTAL), FALSE, FALSE , 0);

    /* box for record length buttons */
    label = gtk_label_new(_("Record Length"));
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
            label, TRUE, TRUE, 0);

    /* now define the radio buttons */
    snprintf(buf, BUFLEN, _("%5d samples (1 channel)"), ctrl_shm->buf_len);
    buttons[0] = gtk_radio_button_new_with_label(NULL, buf);
    snprintf(buf, BUFLEN, _("%5d samples (2 channels)"), ctrl_shm->buf_len / 2);
    buttons[1] =
	gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(buttons
	    [0]), buf);
    snprintf(buf, BUFLEN, _("%5d samples (4 channels)"), ctrl_shm->buf_len / 4);
    buttons[2] =
	gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(buttons
	    [0]), buf);
    snprintf(buf, BUFLEN, _("%5d samples (8 channels)"), ctrl_shm->buf_len / 8);
    buttons[3] =
	gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(buttons
	    [0]), buf);
    snprintf(buf, BUFLEN, _("%5d samples (16 channels)"),
	ctrl_shm->buf_len / 16);
    buttons[4] =
	gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(buttons
	    [0]), buf);
    /* now put them into the box and make visible */
    for (n = 0; n < 5; n++) {
        gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
                buttons[n], FALSE, FALSE, 0);
    }
    /* determine which button should be pressed by default */
    if (ctrl_shm->sample_len == 1) {
	n = 0;
    } else if (ctrl_shm->sample_len == 2) {
	n = 1;
    } else if (ctrl_shm->sample_len == 4) {
	n = 2;
    } else if (ctrl_shm->sample_len == 8) {
	n = 3;
    } else if (ctrl_shm->sample_len == 16) {
	n = 4;
    } else {
	n = 2;
	ctrl_shm->sample_len = 4;
	ctrl_shm->rec_len = ctrl_shm->buf_len / ctrl_shm->sample_len;
    }
    /* set the default button */
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(buttons[n]), TRUE);
    /* set up callbacks for the buttons */
    g_signal_connect(buttons[0], "clicked",
	G_CALLBACK(rec_len_button), (gpointer) 1);
    g_signal_connect(buttons[1], "clicked",
	G_CALLBACK(rec_len_button), (gpointer) 2);
    g_signal_connect(buttons[2], "clicked",
	G_CALLBACK(rec_len_button), (gpointer) 4);
    g_signal_connect(buttons[3], "clicked",
	G_CALLBACK(rec_len_button), (gpointer) 8);
    g_signal_connect(buttons[4], "clicked",
	G_CALLBACK(rec_len_button), (gpointer) 16);

    /* was a thread previously used? */
    if (sel_row > -1) {
	/* yes, preselect appropriate line */
        mark_selected_row(horiz->thread_list, sel_row);
    }
    gtk_widget_show_all(dialog);

    retval = gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);

    /* these items no longer exist - NULL them */
    horiz->thread_list = NULL;
    horiz->thread_name_label = NULL;
    horiz->sample_rate_label = NULL;
    horiz->sample_period_label = NULL;
    horiz->mult_adj = NULL;
    horiz->mult_spinbutton = NULL;

    /* we get here when the user hits OK or Quit */
    if (retval == GTK_RESPONSE_CLOSE) {
	/* user pressed quit - end the program */
	gtk_main_quit();
    }
    activate_sample_thread();
}

static void dialog_realtime_not_running(void)
{
    int retval;
    GtkWidget *dialog;

    dialog = gtk_message_dialog_new(GTK_WINDOW(ctrl_usr->main_win),
                                    GTK_DIALOG_MODAL,
                                    GTK_MESSAGE_INFO,
                                    GTK_BUTTONS_NONE,
                                    _("Realtime thread(s) not running"));
    gtk_message_dialog_format_secondary_text(
            GTK_MESSAGE_DIALOG(dialog),
            _("HALSCOPE uses code in a realtime HAL thread to sample\n"
            "signals for display.  The HAL thread(s) are not running.\n"
            "Threads are usually started by the application you are\n"
            "attempting to run, or you can use the 'halcmd start' command.\n\n"
            "Please do one of the following:\n\n"
            "Start the threads, then click 'OK'\n"
            "or\n" "Click 'Quit' to exit HALSCOPE"));
    gtk_dialog_add_buttons(GTK_DIALOG(dialog),
                           _("OK"), GTK_RESPONSE_OK,
                           _("Quit"), GTK_RESPONSE_CLOSE,
                           NULL);
    retval = gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);

    if (retval == GTK_RESPONSE_CLOSE) {
	/* user pressed quit - end the program */
	gtk_main_quit();
    }
}

void save_log_cb(GtkWindow *parent)
{
    scope_vert_t *vert;
    GtkWidget *dialog;
    int i;

    /* Only create log if one or more channels is selected. */
    vert = &(ctrl_usr->vert);
    for (i = 0; i < 16; i++) {
        if (vert->chan_enabled[i] == 1) {
            log_popup(parent);
            return;
        }
    }

    dialog = gtk_message_dialog_new(NULL,
                                    GTK_DIALOG_MODAL,
                                    GTK_MESSAGE_INFO,
                                    GTK_BUTTONS_OK,
                                    _("No channels selected"));
    gtk_message_dialog_format_secondary_text(
            GTK_MESSAGE_DIALOG(dialog),
            _("You need to choose at least one channel."));
    gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);
}

static void log_popup(GtkWindow *parent)
{
    GtkWidget *filew;
    GtkFileChooser *chooser;

    filew = gtk_file_chooser_dialog_new(_("Pick log file to write to:"),
                                        parent, GTK_FILE_CHOOSER_ACTION_SAVE,
                                        _("_Cancel"), GTK_RESPONSE_CANCEL,
                                        _("_Save"), GTK_RESPONSE_ACCEPT, NULL);

    gtk_file_chooser_set_current_name(GTK_FILE_CHOOSER(filew), "halscope.csv");
    chooser = GTK_FILE_CHOOSER(filew);
    set_file_filter(chooser, "Text CSV (.csv)", "*.csv");
    gtk_file_chooser_set_do_overwrite_confirmation(chooser, TRUE);

    if (gtk_dialog_run(GTK_DIALOG(filew)) == GTK_RESPONSE_ACCEPT) {
        char *filename;

        filename = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(filew));
        write_log_file(filename);
        g_free(filename);
    }
    gtk_widget_destroy(filew);
}

static void acquire_popup(GtkWidget * widget, gpointer gdata)
{
    (void)widget;
    (void)gdata;
    prepare_scope_restart();

    /** This function doesn't directly cause the acquire menu to
        pop up.  Instead is disconnects the acquire function from
        whatever thread is calling it.  This will result in a
        watchdog timeout, and that in turn will pop up the dialog
        requesting you to reconnect it.
    */
    deactivate_sample_thread();
    /* presetting the watchdog to 10 avoids the delay that would otherwise
       take place while the watchdog times out. */
    ctrl_shm->watchdog = 10;
    return;
}

static void acquire_selection_made(GtkWidget *widget, gpointer data)
{
    scope_horiz_t *horiz;
    horiz = (scope_horiz_t *) data;
    char *picked;

    GtkTreeIter iter;
    GtkTreeModel *model;

    if (gtk_tree_selection_get_selected(GTK_TREE_SELECTION(widget), &model, &iter)) {
        gtk_tree_model_get(model, &iter, COL_THREAD, &picked, -1);

        /* set thread */
        set_sample_thread_name(picked);
        /* get a pointer to the horiz data structure */
        horiz = &(ctrl_usr->horiz);
        /* set mult spinbutton to (possibly) new value */
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(horiz->mult_spinbutton), ctrl_shm->mult);
    }
}

static int set_sample_thread_name(char *name)
{
    scope_horiz_t *horiz;
    hal_thread_t *thread;
    long max_mult;

    /* get a pointer to the horiz data structure */
    horiz = &(ctrl_usr->horiz);
    /* look for a new thread that matches name*/
    thread = halpr_find_thread_by_name(name);
    if (thread == NULL) {
	return -1;
    }
    /* shut down any prior thread */
    deactivate_sample_thread();
    /* save info about the thread */
    horiz->thread_name = thread->name;
    horiz->thread_period_ns = thread->period;
    /* calc max possible mult (to keep sample period <= 1 sec */
    max_mult = (1000000000 / horiz->thread_period_ns);
    if (max_mult > 1000) {
	max_mult = 1000;
    }
    if (ctrl_shm->mult > max_mult) {
	ctrl_shm->mult = max_mult;
    }
    calc_horiz_scaling();
    refresh_horiz_info();
    return 0;
}

static void deactivate_sample_thread(void)
{
    /* check for old sample thread */
    if (ctrl_shm->thread_name[0] != '\0') {
	/* disconnect sample funct from old thread */
	hal_del_funct_from_thread("scope.sample", ctrl_shm->thread_name);
	/* clear thread name from shared memory */
	ctrl_shm->thread_name[0] = '\0';
    }
}

static int activate_sample_thread(void)
{
    scope_horiz_t *horiz;
    int rv;

    /* get a pointer to the horiz data structure */
    horiz = &(ctrl_usr->horiz);
    /* has a thread name been specified? */
    if (horiz->thread_name == NULL) {
	return -1;
    }
    /* shut down any prior thread */
    /* (probably already sone, but just making sure */
    deactivate_sample_thread();
    /* hook sampling function to thread */
    rv = hal_add_funct_to_thread("scope.sample", horiz->thread_name, -1);
    if ( rv < 0 ) {
	return rv;
    }
    /* store name of thread in shared memory */
    strncpy(ctrl_shm->thread_name, horiz->thread_name, HAL_NAME_LEN);
    ctrl_shm->thread_name[HAL_NAME_LEN] = '\0';
    /* give the code some time to get started */
    ctrl_shm->watchdog = 0;
    invalidate_all_channels();
    request_display_refresh(1);
    return 0;
}

static void mult_changed(GtkAdjustment * adj, gpointer gdata)
{
    (void)adj;
    (void)gdata;
    scope_horiz_t *horiz;
    int value;

    /* point to GUI widgets */
    horiz = &(ctrl_usr->horiz);
    /* get value from spinbutton */
    value = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(horiz->mult_spinbutton));
    /* set it */
    set_horiz_mult(value);
    /* set spinbutton to new value */
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(horiz->mult_spinbutton), ctrl_shm->mult);
}

static void zoom_changed(GtkAdjustment * adj, gpointer gdata)
{
    (void)gdata;
    set_horiz_zoom(gtk_adjustment_get_value(adj));
}

static void pos_changed(GtkAdjustment * adj, gpointer gdata)
{
    (void)gdata;
    set_horiz_pos(gtk_adjustment_get_value(adj) / 1000.0);
}

static void rec_len_button(GtkWidget * widget, gpointer gdata)
{
    int retval;
    GtkWidget *dialog;

    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget)) != TRUE) {
	/* not pressed, ignore it */
	return;
    }
    retval = set_rec_len((long)gdata);
    if (retval < 0) {
	/* too many channels already enabled */
        dialog = gtk_message_dialog_new(GTK_WINDOW(ctrl_usr->main_win),
                                        GTK_DIALOG_MODAL,
                                        GTK_MESSAGE_INFO,
                                        GTK_BUTTONS_OK,
                                        _("Not enough channels"));
        gtk_message_dialog_format_secondary_text(
                GTK_MESSAGE_DIALOG(dialog),
                _("This record length cannot handle the channels\n"
                "that are currently enabled.  Pick a shorter\n"
                "record length that supports more channels."));
        gtk_dialog_run(GTK_DIALOG(dialog));
        gtk_widget_destroy(dialog);
    }
}

static void calc_horiz_scaling(void)
{
    scope_horiz_t *horiz;
    double total_rec_time;
    long int desired_usec_per_div, actual_usec_per_div;
    int n, decade, sub_decade;

    horiz = &(ctrl_usr->horiz);
    if (horiz->thread_name == NULL) {
	horiz->thread_period_ns = 0;
	ctrl_shm->mult = 1;
	horiz->sample_period_ns = 0;
	horiz->sample_period = 0.0;
	horiz->disp_scale = 0.0;
	return;
    }
    horiz->sample_period_ns = horiz->thread_period_ns * ctrl_shm->mult;
    horiz->sample_period = horiz->sample_period_ns / 1000000000.0;
    total_rec_time = ctrl_shm->rec_len * horiz->sample_period;
    if (total_rec_time < 0.000010) {
	/* out of range, set to 1 µs per div */
	horiz->disp_scale = 0.000001;
	return;
    }
    desired_usec_per_div = (total_rec_time / 10.0) * 1000000.0;
    /* find scaling to display entire record */
    decade = 1;
    sub_decade = 1;
    actual_usec_per_div = decade * sub_decade;
    while (actual_usec_per_div < desired_usec_per_div) {
	if (sub_decade == 1) {
	    sub_decade = 2;
	} else if (sub_decade == 2) {
	    sub_decade = 5;
	} else {
	    sub_decade = 1;
	    decade *= 10;
	}
	actual_usec_per_div = decade * sub_decade;
    }
    /* now correct for zoom factor */
    for (n = 1; n < horiz->zoom_setting; n++) {
	if (sub_decade == 1) {
	    sub_decade = 5;
	    decade /= 10;
	} else if (sub_decade == 2) {
	    sub_decade = 1;
	} else {		/* sub_decade == 5 */

	    sub_decade = 2;
	}
    }
    if (decade == 0) {
	/* underflow from zoom, set to minimum */
	decade = 1;
	sub_decade = 1;
    }
    actual_usec_per_div = decade * sub_decade;
    /* convert to floating point */
    horiz->disp_scale = actual_usec_per_div / 1000000.0;
}

static void refresh_horiz_info(void)
{
    scope_horiz_t *horiz;
    gchar *name;
    static gchar tmp[BUFLEN + 1], rate[BUFLEN + 1], period[BUFLEN + 1];
    static gchar scale[BUFLEN + 1], rec_len[BUFLEN + 1], msg[BUFLEN + 1];
    double freqval;

    horiz = &(ctrl_usr->horiz);
    if (horiz->thread_name == NULL) {
	name = "----";
    } else {
	name = horiz->thread_name;
    }
    if (horiz->disp_scale == 0.0) {
	snprintf(scale, BUFLEN, "----");
    } else {
	format_time_value(tmp, BUFLEN, horiz->disp_scale);
	snprintf(scale, BUFLEN, _("%s\nper div"), tmp);
    }
    if (horiz->sample_period == 0.0) {
	snprintf(period, BUFLEN, "----");
	snprintf(rate, BUFLEN, "----");
    } else {
	format_time_value(period, BUFLEN, horiz->sample_period);
	freqval = 1.0 / horiz->sample_period;
	format_freq_value(rate, BUFLEN, freqval);
    }
    if (ctrl_shm->rec_len == 0) {
	snprintf(rec_len, BUFLEN, "----");
    } else {
	snprintf(rec_len, BUFLEN, "%d", ctrl_shm->rec_len);
    }
    snprintf(msg, BUFLEN, _("%s samples\nat %s"), rec_len, rate);
    gtk_label_set_text_if(horiz->thread_name_label, name);
    gtk_label_set_text_if(horiz->sample_rate_label, rate);
    gtk_label_set_text_if(horiz->scale_label, scale);
    gtk_label_set_text_if(horiz->sample_period_label, period);
    gtk_label_set_text_if(horiz->record_label, msg);
    refresh_state_info();
}

/*
 * Draws the boxes and trigger line in the horiz_info_win
 */
static gboolean refresh_pos_disp(void)
{
    scope_horiz_t *horiz;
    horiz = &(ctrl_usr->horiz);

    double disp_center, disp_start, disp_end;
    double rec_start, rec_curr, rec_end;
    double min, max, span, scale;
    int pre_trig;
    int rec_line_y, rec_line_left, rec_line_right;
    int box_y_off, box_top, box_bot, box_right, box_left;
    int trig_y_off, trig_line_top, trig_line_bot, trig_line_x;
    int rec_curr_x;

    /* these are based on widget dimensions */
    rec_line_y = (horiz->height - 1)/ 2;
    box_y_off = rec_line_y / 2;
    trig_y_off = rec_line_y;
    trig_line_top = rec_line_y - trig_y_off;
    trig_line_bot = rec_line_y + trig_y_off;
    box_top = rec_line_y - box_y_off;
    box_bot = rec_line_y + box_y_off;

    /* these need to be calculated */
    pre_trig = ctrl_shm->rec_len * ctrl_usr->trig.position;
    /* times relative to trigger */
    rec_start = -pre_trig * horiz->sample_period;
    rec_end = (ctrl_shm->rec_len - pre_trig) * horiz->sample_period;
    rec_curr = rec_start + (ctrl_shm->samples * horiz->sample_period);
    disp_center = rec_start + horiz->pos_setting * (rec_end - rec_start);
    disp_start = disp_center - 5.0 * horiz->disp_scale;
    disp_end = disp_center + 5.0 * horiz->disp_scale;
    if (rec_start < disp_start) {
	min = rec_start;
    } else {
	min = disp_start;
    }
    if (rec_end > disp_end) {
	max = rec_end;
    } else {
	max = disp_end;
    }
    span = max - min;
    scale = (horiz->width - 1) / span;

    trig_line_x = scale * (0 - min);
    rec_line_left = scale * (rec_start - min);
    rec_line_right = scale * (rec_end - min);
    rec_curr_x = scale * (rec_curr - min);
    box_left = scale * (disp_start - min);
    box_right = scale * (disp_end - min);

    /* set color to black */
    cairo_set_source_rgb(horiz->disp_context, 0.0, 0.0, 0.0);

    /* small box */
    cairo_move_to(horiz->disp_context, rec_line_left, rec_line_y + 2);
    cairo_line_to(horiz->disp_context, rec_line_right, rec_line_y + 2);
    cairo_line_to(horiz->disp_context, rec_line_right, rec_line_y - 2);
    cairo_line_to(horiz->disp_context, rec_line_left, rec_line_y - 2);
    cairo_line_to(horiz->disp_context, rec_line_left, rec_line_y + 2);
    cairo_stroke(horiz->disp_context);

    /* fill small box */
    cairo_move_to(horiz->disp_context, rec_line_left, rec_line_y);
    cairo_line_to(horiz->disp_context, rec_curr_x, rec_line_y);
    cairo_stroke(horiz->disp_context);

    /* vertical trigger line */
    cairo_move_to(horiz->disp_context, trig_line_x, trig_line_top);
    cairo_line_to(horiz->disp_context, trig_line_x, trig_line_bot);
    cairo_stroke(horiz->disp_context);

    /* large positional box */
    cairo_move_to(horiz->disp_context, box_left, box_top);
    cairo_line_to(horiz->disp_context, box_right, box_top);
    cairo_line_to(horiz->disp_context, box_right, box_bot);
    cairo_line_to(horiz->disp_context, box_left, box_bot);
    cairo_line_to(horiz->disp_context, box_left, box_top);
    cairo_stroke(horiz->disp_context);

    gtk_widget_queue_draw(horiz->disp_area);
    return FALSE;
}

static void format_time_value(char *buf, int buflen, double timeval)
{
    const char *units;
    int decimals;

    /* convert to nanoseconds */
    timeval *= 1000000000.0;
    units = _("ns");
    if (timeval >= 1000.0) {
	timeval /= 1000.0;
	units = _("µs");
    }
    if (timeval >= 1000.0) {
	timeval /= 1000.0;
	units = _("ms");
    }
    if (timeval >= 1000.0) {
	timeval /= 1000.0;
	units = _("s");
    }
    decimals = 2;
    if (timeval >= 10.0) {
	decimals = 1;
    }
    if (timeval >= 100.0) {
	decimals = 0;
    }
    snprintf(buf, buflen, "%0.*f %s", decimals, timeval, units);
}

static void format_freq_value(char *buf, int buflen, double freqval)
{
    const char *units;
    int decimals;

    units = _("Hz");
    if (freqval >= 1000.0) {
	freqval /= 1000.0;
	units = _("kHz");
    }
    if (freqval >= 1000.0) {
	freqval /= 1000.0;
	units = _("MHz");
    }
    decimals = 2;
    if (freqval >= 10.0) {
	decimals = 1;
    }
    if (freqval >= 100.0) {
	decimals = 0;
    }
    snprintf(buf, buflen, "%0.*f %s", decimals, freqval, units);
}

static gint horiz_press(GtkWidget *widget, GdkEventButton *event) {
    (void)widget;
    ctrl_usr->horiz.x0 = event->x;
    return TRUE;
}

static gint horiz_motion(GtkWidget *widget, GdkEventMotion *event) {
    (void)widget;
    scope_horiz_t *horiz = &(ctrl_usr->horiz);

    int motion;

    int pre_trig;
    double disp_center, disp_start, disp_end;
    double rec_start, rec_end;
    double min, max, span, scale;
    double newpos;

    int x, y;
    GdkModifierType state;

    if (event->is_hint) {
        gdk_window_get_device_position(event->window, event->device, &x, &y, &state);
    } else {
        x = event->x;
        y = event->y;
        state = event->state;
    }

    if(!(state & GDK_BUTTON1_MASK)) return TRUE;

    motion = x - horiz->x0;

    pre_trig = ctrl_shm->rec_len * ctrl_usr->trig.position;
    rec_start = -pre_trig * horiz->sample_period;
    rec_end = (ctrl_shm->rec_len - pre_trig) * horiz->sample_period;
    disp_center = rec_start + horiz->pos_setting * (rec_end - rec_start);
    disp_start = disp_center - 5.0 * horiz->disp_scale;
    disp_end = disp_center + 5.0 * horiz->disp_scale;

    if (rec_start < disp_start) {
	min = rec_start;
    } else {
	min = disp_start;
    }
    if (rec_end > disp_end) {
	max = rec_end;
    } else {
	max = disp_end;
    }
    span = max - min;
    scale = (horiz->width - 1) / span;

    newpos = gtk_adjustment_get_value(
            GTK_ADJUSTMENT(horiz->pos_adj)) + motion * 100 / scale;
    gtk_adjustment_set_value(GTK_ADJUSTMENT(horiz->pos_adj), newpos);

    horiz->x0 = event->x;
    return TRUE;
}

static gint horiz_release(GtkWidget *widget, GdkEventButton *event) {
    (void)widget;
    (void)event;
    return TRUE;
}

static gboolean configure_window(GtkWidget *widget, GdkEventConfigure *event,
                                 gpointer data)
{
    (void)event;
    (void)data;
    scope_horiz_t *horiz;
    horiz = &(ctrl_usr->horiz);

    if (horiz->surface) {
        cairo_surface_destroy(horiz->surface);
    }

    horiz->width = gtk_widget_get_allocated_width(widget);
    horiz->height = gtk_widget_get_allocated_height(widget);
    horiz->surface = gdk_window_create_similar_image_surface(gtk_widget_get_window(widget),
            CAIRO_FORMAT_A1, horiz->width, horiz->height, 0);

    horiz->disp_context = cairo_create(horiz->surface);
    cairo_destroy(horiz->disp_context);

    return TRUE;
}
