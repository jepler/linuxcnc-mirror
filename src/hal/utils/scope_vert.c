/** This file, 'scope_vert.c', contains the portion of halscope
    that deals with vertical stuff - signal sources, scaling,
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

#include "rtapi.h"		// RTAPI realtime OS API
#include <rtapi_mutex.h>
#include <rtapi_string.h>	// rtapi_strlcpy()
#include "hal.h"		// HAL public API decls
#include "../hal_priv.h"	// private HAL decls

#include <gtk/gtk.h>

#include "miscgtk.h"		// generic GTK stuff
#include "scope_usr.h"		// scope related declarations
#include <rtapi_string.h>

#define BUFLEN 80		// length for sprintf buffers

/***********************************************************************
*                  GLOBAL VARIABLES DECLARATIONS                       *
************************************************************************/

#define VERT_POS_RESOLUTION 100.0

/* Columns in the TreeView */
enum TREEVIEW_COLUMN {
    LIST_ITEM,
    NUM_COLS
};

/* The channel select buttons sometimes need to be toggled by
   the code rather than the user, without causing any action.
   This global is used for that */
static int ignore_click = 0;

/***********************************************************************
*                  LOCAL FUNCTION PROTOTYPES                           *
************************************************************************/

struct offset_data {
    char buf[BUFLEN];
    int ac_coupled;
};

GtkWidget *chan_buttons[16] = {NULL};

static void init_chan_sel_window(void);
static void init_chan_info_window(void);
static void init_vert_info_window(void);

static gboolean dialog_select_source(int chan_num);
static void selection_changed(GtkTreeSelection *selection, char *name);
static void selection_made(GtkTreeView *treeview, GtkTreePath *path,
        GtkTreeViewColumn *col, GtkWidget *dialog);
static void change_source_button(GtkWidget * widget, gpointer gdata);
static void offset_button(GtkWidget * widget, gpointer gdata);
static gboolean dialog_set_offset(int chan_num);
static void scale_changed(GtkAdjustment * adj, gpointer gdata);
static void offset_changed(GtkEditable * editable, struct offset_data *);
static void offset_activated(GtkEntry *entry, GtkWidget *dialog);
static void pos_changed(GtkAdjustment * adj, gpointer gdata);
static void chan_sel_button(GtkWidget * widget, gpointer gdata);

/* helper functions */
static void write_chan_config(FILE *fp, scope_chan_t *chan);
static void style_with_css(GtkWidget *widget, int color_index);

/***********************************************************************
*                       PUBLIC FUNCTIONS                               *
************************************************************************/

void init_vert(void)
{
    scope_chan_t *chan;
    int n;

    /* stop sampling */
    ctrl_shm->state = IDLE;
    /* init non-zero members of the vertical structure */
    invalidate_all_channels();
    /* init non-zero members of the channel structures */
    for (n = 1; n <= 16; n++) {
	chan = &(ctrl_usr->chan[n - 1]);
	chan->position = 0.5;
    }
    /* set up the windows */
    init_chan_sel_window();
    init_chan_info_window();
    init_vert_info_window();
}

int set_active_channel(int chan_num)
{
    int n, count;
    scope_vert_t *vert;
    scope_chan_t *chan;
    if (( chan_num < 1 ) || ( chan_num > 16 )) {
	return -1;
    }
    vert = &(ctrl_usr->vert);
    chan = &(ctrl_usr->chan[chan_num - 1]);

    if (vert->chan_enabled[chan_num - 1] == 0 ) {
	/* channel is disabled, want to enable it */
	if (ctrl_shm->state != IDLE) {
	    /* acquisition in progress, must restart it */
            prepare_scope_restart();
	}
	count = 0;
	for (n = 0; n < 16; n++) {
	    if (vert->chan_enabled[n]) {
		count++;
	    }
	}
	if (count >= ctrl_shm->sample_len) {
	    /* max number of channels already enabled */
	    return -2;
	}
	if (chan->name == NULL) {
	    /* no signal source */
	    return -3;
	}
	/* "push" the button in to indicate enabled channel */
	ignore_click = 1;
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(vert->chan_sel_buttons[chan_num-1]), TRUE);
	vert->chan_enabled[chan_num - 1] = 1;
    }
    if (vert->selected != chan_num) {
	/* make chan_num the selected channel */
	vert->selected = chan_num;
	channel_changed();
    }
    return 0;
}

int set_channel_off(int chan_num)
{
    scope_vert_t *vert;
    int n;

    if ((chan_num < 1) || (chan_num > 16)) {
	return -1;
    }
    vert = &(ctrl_usr->vert);
    if ( vert->chan_enabled[chan_num - 1] == 0 ) {
	/* channel is already off, nothing to do */
	return -1;
    }
    vert->chan_enabled[chan_num - 1] = 0;
    /* force the button to pop out */
    ignore_click = 1;
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(vert->
	    chan_sel_buttons[chan_num - 1]), FALSE);
    if ( chan_num == vert->selected ) {
	/* channel was selected, pick new selected channel */
	n = 0;
	vert->selected = 0;
	do {
	    chan_num++;
	    if (chan_num > 16) {
		chan_num = 1;
	    }
	    if (vert->chan_enabled[chan_num - 1] != 0) {
		vert->selected = chan_num;
	    }
	} while ((++n < 16) && (vert->selected == 0));
    }
    channel_changed();
    return 0;
}

int set_channel_source(int chan_num, int type, char *name)
{
    scope_vert_t *vert;
    scope_chan_t *chan;
    hal_pin_t *pin;
    hal_sig_t *sig;
    hal_param_t *param;

    vert = &(ctrl_usr->vert);
    chan = &(ctrl_usr->chan[chan_num - 1]);
    /* locate the selected item in the HAL */
    if (type == 0) {
	/* search the pin list */
	pin = halpr_find_pin_by_name(name);
	if (pin == NULL) {
	    /* pin not found */
	    return -1;
	}
	chan->data_source_type = 0;
	chan->data_source = SHMOFF(pin);
	chan->data_type = pin->type;
	chan->name = pin->name;
    } else if (type == 1) {
	/* search the signal list */
	sig = halpr_find_sig_by_name(name);
	if (sig == NULL) {
	    /* signal not found */
	    return -1;
	}
	chan->data_source_type = 1;
	chan->data_source = SHMOFF(sig);
	chan->data_type = sig->type;
	chan->name = sig->name;
    } else if (type == 2) {
	/* search the parameter list */
	param = halpr_find_param_by_name(name);
	if (param == NULL) {
	    /* parameter not found */
	    return -1;
	}
	chan->data_source_type = 2;
	chan->data_source = SHMOFF(param);
	chan->data_type = param->type;
	chan->name = param->name;
    }
    switch (chan->data_type) {
    case HAL_BIT:
	chan->data_len = sizeof(hal_bit_t);
	chan->min_index = -2;
	chan->max_index = 2;
	break;
    case HAL_FLOAT:
	chan->data_len = sizeof(hal_float_t);
	chan->min_index = -36;
	chan->max_index = 36;
	break;
    case HAL_S32:
	chan->data_len = sizeof(hal_s32_t);
	chan->min_index = -2;
	chan->max_index = 30;
	break;
    case HAL_U32:
	chan->data_len = sizeof(hal_u32_t);
	chan->min_index = -2;
	chan->max_index = 30;
	break;
    default:
	/* Shouldn't get here, but just in case... */
	chan->data_len = 0;
	chan->min_index = -1;
	chan->max_index = 1;
    }
    /* invalidate any data in the buffer for this channel */
    vert->data_offset[chan_num - 1] = -1;
    /* set scale and offset to nominal values */
    chan->vert_offset = 0.0;
    chan->scale_index = 0;
    /* return success */
    return 0;
}

int set_vert_scale(int setting)
{
    scope_vert_t *vert;
    scope_chan_t *chan;
    GtkAdjustment *adj;
    int chan_num, index;
    double scale;
    gchar buf[BUFLEN];

    vert = &(ctrl_usr->vert);
    chan_num = vert->selected;
    if ((chan_num < 1) || (chan_num > 16)) {
	return -1;
    }
    chan = &(ctrl_usr->chan[chan_num - 1]);
    if ((setting > chan->max_index) || (setting < chan->min_index)) {
	/* value out of range for this data type */
	return -1;
    }
    /* save new index */
    chan->scale_index = setting;
    /* set scale slider based on new setting */
    adj = GTK_ADJUSTMENT(vert->scale_adj);
    gtk_adjustment_set_value(adj, setting);

    /* compute scale factor */
    scale = 1.0;
    index = chan->scale_index;
    while (index >= 3) {
	scale *= 10.0;
	index -= 3;
    }
    while (index <= -3) {
	scale *= 0.1;
	index += 3;
    }
    switch (index) {
    case 2:
	scale *= 5.0;
	break;
    case 1:
	scale *= 2.0;
	break;
    case -1:
	scale *= 0.5;
	break;
    case -2:
	scale *= 0.2;
	break;
    default:
	break;
    }
    chan->scale = scale;
    format_scale_value(buf, BUFLEN - 1, scale);
    gtk_label_set_text_if(vert->scale_label, buf);
    if (chan_num == ctrl_shm->trig_chan) {
	refresh_trigger();
    }
    request_display_refresh(1);
    return 0;
}

int set_vert_pos(double setting)
{
    scope_vert_t *vert;
    scope_chan_t *chan;
    int chan_num;
    GtkAdjustment *adj;

    /* range check setting */
    if (( setting < 0.0 ) || ( setting > 1.0 )) {
	return -1;
    }
    /* point to data */
    vert = &(ctrl_usr->vert);
    chan_num = vert->selected;
    if ((chan_num < 1) || (chan_num > 16)) {
	return -1;
    }
    chan = &(ctrl_usr->chan[chan_num - 1]);
    chan->position = setting;
    /* set position slider based on new setting */
    adj = GTK_ADJUSTMENT(vert->pos_adj);
    gtk_adjustment_set_value(adj, chan->position * VERT_POS_RESOLUTION);
    /* refresh other stuff */
    if (chan_num == ctrl_shm->trig_chan) {
	refresh_trigger();
    }
    request_display_refresh(1);
    return 0;
}

int set_vert_offset(double setting, int ac_coupled)
{
    scope_vert_t *vert;
    scope_chan_t *chan;
    int chan_num;
    gchar buf1[BUFLEN + 1], buf2[BUFLEN + 1];

    /* point to data */
    vert = &(ctrl_usr->vert);
    chan_num = vert->selected;
    if ((chan_num < 1) || (chan_num > 16)) {
	return -1;
    }
    chan = &(ctrl_usr->chan[chan_num - 1]);
    /* set the new offset */
    chan->vert_offset = setting;
    chan->ac_offset = ac_coupled;
    /* update the offset display */
    if (chan->data_type == HAL_BIT) {
	snprintf(buf1, BUFLEN, "----");
    } else {
        if(chan->ac_offset) {
            snprintf(buf1, BUFLEN, "(AC)");
        } else {
            format_signal_value(buf1, BUFLEN, chan->vert_offset);
        }
    }
    snprintf(buf2, BUFLEN, _("Offset\n%s"), buf1);
    gtk_label_set_text_if(vert->offset_label, buf2);
    /* refresh other stuff */
    if (chan_num == ctrl_shm->trig_chan) {
	refresh_trigger();
    }
    request_display_refresh(1);
    return 0;
}

void format_signal_value(char *buf, int buflen, double value)
{
    char *units;
    int decimals;
    char sign, symbols[] = "pnum KMGT";

    if (value < 0) {
	value = -value;
	sign = '-';
    } else {
	sign = '+';
    }
    if (value <= 1.0e-24) {
	/* pretty damn small, call it zero */
	snprintf(buf, buflen, "0.000");
	return;
    }
    if (value <= 1.0e-12) {
	/* less than pico units, use scientific notation */
	snprintf(buf, buflen, "%c%10.3e", sign, value);
	return;
    }
    if (value >= 1.0e+12) {
	/* greater than tera-units, use scientific notation */
	snprintf(buf, buflen, "%c%10.3e", sign, value);
	return;
    }
    units = &(symbols[4]);
    while (value < 1.0) {
	value *= 1000.0;
	units--;
    }
    while (value >= 1000.0) {
	value /= 1000.0;
	units++;
    }
    decimals = 3;
    if (value >= 9.999) {
	decimals--;
    }
    if (value >= 99.99) {
	decimals--;
    }
    snprintf(buf, buflen, "%c%0.*f%c", sign, decimals, value, *units);
}

void write_vert_config(FILE *fp)
{
    int n;
    scope_vert_t *vert;
    scope_chan_t *chan;

    vert = &(ctrl_usr->vert);
    /* first write disabled channels */
    for ( n = 1 ; n <= 16 ; n++ ) {
	if ( vert->chan_enabled[n-1] != 0 ) {
	    // channel enabled, do it later
	    continue;
	}
	chan = &(ctrl_usr->chan[n-1]);
	if ( chan->name == NULL ) {
	    // no source for this channel, skip it
	    continue;
	}
	fprintf(fp, "CHAN %d\n", n);
	write_chan_config(fp, chan);
	fprintf(fp, "CHOFF\n");
    }
    /* next write enabled channels */
    for ( n = 1 ; n <= 16 ; n++ ) {
	if ( vert->chan_enabled[n-1] == 0 ) {
	    // channel disabled, skip it
	    continue;
	}
	if ( vert->selected == n ) {
	    // channel selected, do it last
	    continue;
	}
	chan = &(ctrl_usr->chan[n-1]);
	if ( chan->name == NULL ) {
	    // no source for this channel, skip it
	    continue;
	}
	fprintf(fp, "CHAN %d\n", n);
	write_chan_config(fp, chan);
    }
    /* write selected channel last */
    if ((vert->selected < 1) || (vert->selected > 16)) {
	return;
    }
    chan = &(ctrl_usr->chan[vert->selected-1]);
    fprintf(fp, "CHAN %d\n", vert->selected);
    write_chan_config(fp, chan);
}


/***********************************************************************
*                       LOCAL FUNCTIONS                                *
************************************************************************/

extern int normal_colors[16][3];
static void init_chan_sel_window(void)
{
    scope_vert_t *vert;
    GtkWidget *button;
    long n;
    gchar buf[5];

    vert = &(ctrl_usr->vert);
    for (n = 0; n < 16; n++) {
        snprintf(buf, 4, "%ld", n + 1);
        /* define the button */
        button = gtk_toggle_button_new_with_label(buf);
        chan_buttons[n] = button;

        style_with_css(button, n);
        /* put it in the window */
        gtk_box_pack_start(GTK_BOX(ctrl_usr->chan_sel_win), button, TRUE,
            TRUE, 0);
        gtk_widget_show(button);
        /* hook a callback function to it */
        g_signal_connect(button, "clicked",
            G_CALLBACK(chan_sel_button), (gpointer) n + 1);
        /* save the button pointer */
        vert->chan_sel_buttons[n] = button;
    }
}

static void init_chan_info_window(void)
{
    scope_vert_t *vert;
    char dummyname[HAL_NAME_LEN+1];
    int n;

    vert = &(ctrl_usr->vert);

    vert->chan_num_label =
	gtk_label_new_in_box("--", ctrl_usr->chan_info_win, FALSE, FALSE, 5);
    gtk_label_size_to_fit(GTK_LABEL(vert->chan_num_label), "99");
    gtk_vseparator_new_in_box(ctrl_usr->chan_info_win, 3);

    /* a button to change the source */
    vert->source_name_button = gtk_button_new_with_label("------");
    gtk_box_pack_start(GTK_BOX(ctrl_usr->chan_info_win),
	vert->source_name_button, FALSE, FALSE, 3);

    vert->source_name_label = gtk_bin_get_child(GTK_BIN(vert->source_name_button));
    gtk_label_set_justify(GTK_LABEL(vert->source_name_label),
	GTK_JUSTIFY_LEFT);
    /* longest source name we ever need to display */
    for ( n = 0 ; n < HAL_NAME_LEN ; n++) dummyname[n] = 'x';
    dummyname[n] = '\0';
    gtk_label_size_to_fit(GTK_LABEL(vert->source_name_label), dummyname);
    /* activate the source selection dialog if button is clicked */
    g_signal_connect(vert->source_name_button, "clicked",
	G_CALLBACK(change_source_button), NULL);
    gtk_widget_show(vert->source_name_button);


    vert->readout_label = gtk_label_new_in_box("",
		    ctrl_usr->chan_info_win, FALSE, FALSE, 0);
    gtk_label_set_justify(GTK_LABEL(vert->readout_label), GTK_JUSTIFY_LEFT);
    gtk_label_size_to_fit(GTK_LABEL(vert->readout_label),
		    "f(99999.9999) = 99999.9999 (ddt 99999.9999)");
}

static void init_vert_info_window(void)
{
    scope_vert_t *vert;
    GtkWidget *hbox, *vbox;

    vert = &(ctrl_usr->vert);

    /* box for the two sliders */
    hbox =
	gtk_hbox_new_in_box(TRUE, 0, 0, ctrl_usr->vert_info_win, TRUE, TRUE,
	0);
    /* box for the scale slider */
    vbox = gtk_vbox_new_in_box(FALSE, 0, 0, hbox, TRUE, TRUE, 0);
    gtk_label_new_in_box(_("Gain"), vbox, FALSE, FALSE, 0);
    vert->scale_adj = gtk_adjustment_new(0, -5, 5, 1, 1, 0);
    vert->scale_slider = gtk_scale_new(
            GTK_ORIENTATION_VERTICAL, GTK_ADJUSTMENT(vert->scale_adj));
    gtk_scale_set_digits(GTK_SCALE(vert->scale_slider), 0);
    gtk_scale_set_draw_value(GTK_SCALE(vert->scale_slider), FALSE);
    gtk_box_pack_start(GTK_BOX(vbox), vert->scale_slider, TRUE, TRUE, 0);
    /* connect the slider to a function that re-calcs vertical scale */
    g_signal_connect(vert->scale_adj, "value_changed",
	G_CALLBACK(scale_changed), NULL);
    gtk_widget_show(vert->scale_slider);
    /* box for the position slider */
    vbox = gtk_vbox_new_in_box(FALSE, 0, 0, hbox, TRUE, TRUE, 0);
    gtk_label_new_in_box(_("Pos"), vbox, FALSE, FALSE, 0);
    vert->pos_adj =
	gtk_adjustment_new(VERT_POS_RESOLUTION / 2, 0, VERT_POS_RESOLUTION, 1,
	1, 0);
    vert->pos_slider = gtk_scale_new(
            GTK_ORIENTATION_VERTICAL, GTK_ADJUSTMENT(vert->pos_adj));
    gtk_scale_set_digits(GTK_SCALE(vert->pos_slider), 0);
    gtk_scale_set_draw_value(GTK_SCALE(vert->pos_slider), FALSE);
    gtk_box_pack_start(GTK_BOX(vbox), vert->pos_slider, TRUE, TRUE, 0);
    /* connect the slider to a function that re-calcs vertical pos */
    g_signal_connect(vert->pos_adj, "value_changed",
	G_CALLBACK(pos_changed), NULL);
    gtk_widget_show(vert->pos_slider);
    /* Scale display */
    gtk_hseparator_new_in_box(ctrl_usr->vert_info_win, 3);
    gtk_label_new_in_box(_("Scale"), ctrl_usr->vert_info_win, FALSE, FALSE, 0);
    vert->scale_label =
	gtk_label_new_in_box(" ---- ", ctrl_usr->vert_info_win, FALSE, FALSE,
	0);
    /* Offset control */
    vert->offset_button = gtk_button_new_with_label(_("Offset\n----"));
    vert->offset_label = gtk_bin_get_child(GTK_BIN(vert->offset_button));
    gtk_box_pack_start(GTK_BOX(ctrl_usr->vert_info_win),
	vert->offset_button, FALSE, FALSE, 0);
    g_signal_connect(vert->offset_button, "clicked",
	G_CALLBACK(offset_button), NULL);
    gtk_widget_show(vert->offset_button);
}

static void scale_changed(GtkAdjustment * adj, gpointer gdata)
{
    (void)gdata;
    set_vert_scale(gtk_adjustment_get_value(adj));
}

static void pos_changed(GtkAdjustment * adj, gpointer gdata)
{
    (void)gdata;
    set_vert_pos(gtk_adjustment_get_value(adj) / VERT_POS_RESOLUTION);
}

static void offset_button(GtkWidget * widget, gpointer gdata)
{
    (void)widget;
    (void)gdata;
    scope_vert_t *vert;
    scope_chan_t *chan;
    int chan_num;

    vert = &(ctrl_usr->vert);
    chan_num = vert->selected;
    if ((chan_num < 1) || (chan_num > 16)) {
	return;
    }
    chan = &(ctrl_usr->chan[chan_num - 1]);
    if (chan->data_type == HAL_BIT) {
	/* no offset for bits */
	return;
    }
    if (dialog_set_offset(chan_num)) {
	if (chan_num == ctrl_shm->trig_chan) {
	    refresh_trigger();
	}
	channel_changed();
	request_display_refresh(1);
    }
}

static gboolean dialog_set_offset(int chan_num)
{
    scope_vert_t *vert;
    scope_chan_t *chan;
    char msg[BUFLEN], *cptr;
    struct offset_data data;
    GtkWidget *content_area;
    GtkWidget *dialog;
    GtkWidget *label;
    double tmp;
    int retval;

    vert = &(ctrl_usr->vert);
    chan = &(ctrl_usr->chan[chan_num - 1]);
    snprintf(msg, BUFLEN - 1, _("Set the vertical offset\n"
	"for channel %d."), chan_num);

    /* create dialog window, disable resizing and place it in center of screen */
    dialog = gtk_dialog_new_with_buttons(_("Set Offset"),
                                         NULL, GTK_DIALOG_MODAL,
                                         _("_OK"), GTK_RESPONSE_OK,
                                         _("_Cancel"), GTK_RESPONSE_CANCEL,
                                         NULL);
    gtk_window_set_position(GTK_WINDOW(dialog), GTK_WIN_POS_CENTER);
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));

    /* add elements to dialog */
    label = gtk_label_new(msg);
    gtk_widget_set_margin_start(label, 15);
    gtk_widget_set_margin_end(label, 15);
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
            label, FALSE, FALSE, 5);

    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
            gtk_separator_new(GTK_ORIENTATION_HORIZONTAL), FALSE, FALSE, 0);

    vert->offset_ac = gtk_check_button_new_with_label(_("AC Coupled"));
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
            vert->offset_ac, FALSE, TRUE, 0);

    vert->offset_entry = gtk_entry_new();
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
            vert->offset_entry, FALSE, TRUE, 0);

    /* update elements */
    snprintf(data.buf, BUFLEN, "%f", chan->vert_offset);
    gtk_entry_set_text(GTK_ENTRY(vert->offset_entry), data.buf);
    gtk_entry_set_max_length(GTK_ENTRY(vert->offset_entry), BUFLEN-1);
    /* point at first char */
    gtk_editable_set_position(GTK_EDITABLE(vert->offset_entry), 0);
    /* select all chars, so if the user types the original value goes away */
    gtk_editable_select_region(GTK_EDITABLE(vert->offset_entry), 0, strlen(data.buf));
    /* make it active so user doesn't have to click on it */
    gtk_widget_grab_focus(GTK_WIDGET(vert->offset_entry));

    /* signals */
    g_signal_connect(vert->offset_ac, "toggled",
	G_CALLBACK(offset_changed), &data);
    g_signal_connect(vert->offset_entry, "changed",
	G_CALLBACK(offset_changed), &data);
    g_signal_connect(vert->offset_entry, "activate",
	G_CALLBACK(offset_activated), dialog);
    gtk_widget_show_all(dialog);

    retval = gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);
    if (retval == GTK_RESPONSE_OK) {
        tmp = strtod(data.buf, &cptr);
        if (cptr == data.buf) {
            return FALSE;
        }
        set_vert_offset(tmp, data.ac_coupled);
        return TRUE;
    }
    return FALSE;
}

static void offset_changed(GtkEditable * editable, struct offset_data *data)
{
    (void)editable;
    const char *text;

    /* maybe user hit "ac coupled" button" */
    data->ac_coupled =
      gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ctrl_usr->vert.offset_ac));
    gtk_widget_set_sensitive(ctrl_usr->vert.offset_entry, !data->ac_coupled);

    /* maybe user typed something, save it in the buffer */
    text = gtk_entry_get_text(GTK_ENTRY(ctrl_usr->vert.offset_entry));
    snprintf(data->buf, BUFLEN, "%s", text);
}

/*
 * emit GTK_REPSONSE_OK signal when 'enter' is pressed and
 * the text entry widget is active
 */
static void offset_activated(GtkEntry *entry, GtkWidget *dialog)
{
    (void)entry;
    gtk_dialog_response(GTK_DIALOG(dialog), GTK_RESPONSE_OK);
}


static void chan_sel_button(GtkWidget * widget, gpointer gdata)
{
    long chan_num;
    int n, count;
    scope_vert_t *vert;
    scope_chan_t *chan;
    GtkWidget *dialog;

    vert = &(ctrl_usr->vert);
    chan_num = (long) gdata;
    chan = &(ctrl_usr->chan[chan_num - 1]);

    if (ignore_click) {
        ignore_click = 0;
        return;
    }
    if (vert->chan_enabled[chan_num - 1] == 0 ) {
        /* channel is disabled, want to enable it */

        if (ctrl_shm->state != IDLE) {
            /* acquisition in progress, must restart it */
            prepare_scope_restart();
        }
        count = 0;
        for (n = 0; n < 16; n++) {
            if (vert->chan_enabled[n]) {
            count++;
            }
        }
        if (count >= ctrl_shm->sample_len) {
            /* max number of channels already enabled */
            /* force the button to pop back out */
            ignore_click = 1;
            gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(widget), FALSE);
            dialog = gtk_message_dialog_new(GTK_WINDOW(ctrl_usr->main_win),
                                            GTK_DIALOG_MODAL,
                                            GTK_MESSAGE_INFO,
                                            GTK_BUTTONS_CLOSE,
                                            _("Too many channels"));
            gtk_message_dialog_format_secondary_text(
                    GTK_MESSAGE_DIALOG(dialog),
                    _("You cannot add another channel.\n\n"
                    "Either turn off one or more channels, or shorten\n"
                    "the record length to allow for more channels"));
            gtk_dialog_run(GTK_DIALOG(dialog));
            gtk_widget_destroy(dialog);
            return;
        }
        if (chan->name == NULL) {
            /* need to assign a source */

            if (dialog_select_source(chan_num) != TRUE) {
                /* user failed to assign a source */
                /* force the button to pop back out */
                ignore_click = 1;
                gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(widget), FALSE);
                return;
            }
        }
        vert->chan_enabled[chan_num - 1] = 1;
        /* make chan_num the selected channel */
        vert->selected = chan_num;
        channel_changed();
    } else if (vert->selected == chan_num) {
        /* a click on an already active channel turns it off */
        set_channel_off(chan_num);
        ignore_click = 0;
    } else {
        /* channel was already enabled, user wants to select it */
        /* button should stay down, so we force it */
        ignore_click = 1;
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(widget), TRUE);
        /* make chan_num the selected channel */
        vert->selected = chan_num;
        channel_changed();
    }
}

static void change_source_button(GtkWidget * widget, gpointer gdata)
{
    (void)widget;
    (void)gdata;
    int chan_num;

    chan_num = ctrl_usr->vert.selected;
    if ((chan_num < 1) || (chan_num > 16)) {
	return;
    }
    if (ctrl_shm->state != IDLE) {
        /* acquisition in progress, must restart it */
        prepare_scope_restart();
    }
    invalidate_channel(chan_num);
    dialog_select_source(chan_num);
    channel_changed();
}

static void change_page(GtkNotebook *notebook, GtkWidget *page,
                        guint page_num, gpointer user_data)
{
    (void)notebook;
    (void)page;
    (void)user_data;
    scope_vert_t *vert;

    vert = &(ctrl_usr->vert);
    vert->listnum = page_num;
    gtk_widget_grab_focus(GTK_WIDGET(vert->lists[page_num]));
}

static gboolean dialog_select_source(int chan_num)
{
    scope_vert_t *vert;
    scope_chan_t *chan;

    GtkWidget *content_area;
    GtkWidget *dialog;
    GtkWidget *label;
    GtkWidget *scrolled_window;

    hal_pin_t *pin;
    hal_sig_t *sig;
    hal_param_t *param;

    char *tab_label_text[3];
    char *name[HAL_NAME_LEN + 1];
    char signal_name[HAL_NAME_LEN + 1];
    char title[BUFLEN];
    int next, n, tab, retval;
    int row, match_tab, match_row;

    vert = &(ctrl_usr->vert);
    chan = &(ctrl_usr->chan[chan_num - 1]);

    vert->chan_num = chan_num;

    snprintf(title, BUFLEN - 1, _("Select Channel %d Source"), chan_num);

    /* create dialog window, disable resizing, set title, size and position */
    dialog = gtk_dialog_new_with_buttons(title,
                                         NULL, GTK_DIALOG_MODAL,
                                         _("_OK"), GTK_RESPONSE_ACCEPT,
                                         _("_Cancel"), GTK_RESPONSE_CANCEL,
                                         NULL);
    gtk_widget_set_size_request(GTK_WIDGET(dialog), -1, 400);
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    gtk_window_set_position(GTK_WINDOW(dialog), GTK_WIN_POS_CENTER);
    content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));

    /*
    * create a notebook to hold pin, signal, and parameter list,
    * remember the notebook so we can change the pages later and
    * add the notebook to the window
    */
    vert->notebook = gtk_notebook_new();
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)), vert->notebook,
            TRUE, TRUE, 0);

    /* text for tab labels */
    tab_label_text[0] = _("Pins");
    tab_label_text[1] = _("Signals");
    tab_label_text[2] = _("Parameters");

    /* loop to create three identical tabs */
    for (n = 0; n < 3; n++) {
        /* Create a scrolled window to display the list */
        scrolled_window = gtk_scrolled_window_new(NULL, NULL);
        gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled_window),
                GTK_POLICY_AUTOMATIC, GTK_POLICY_ALWAYS);

        /* create and set tabs in notebook */
        label = gtk_label_new_with_mnemonic(tab_label_text[n]);
        gtk_widget_set_size_request(label, 70, -1);
        gtk_notebook_append_page(GTK_NOTEBOOK(vert->notebook), scrolled_window, label);

        /* create a list to hold the data */
        vert->lists[n] = gtk_tree_view_new();
        gtk_tree_view_set_headers_visible(
                GTK_TREE_VIEW(vert->lists[n]), FALSE);
        init_list(vert->lists[n], &tab_label_text[n], NUM_COLS);
        gtk_container_add(GTK_CONTAINER(scrolled_window), vert->lists[n]);

        g_signal_connect(vert->lists[n], "row-activated",
            G_CALLBACK(selection_made), dialog);
        g_signal_connect(gtk_tree_view_get_selection(GTK_TREE_VIEW(vert->lists[n])),
                         "changed", G_CALLBACK(selection_changed), signal_name);
    }

    /* signals */
    g_signal_connect(vert->notebook, "switch-page",
            G_CALLBACK(change_page), vert);

    /* populate the pin, signal, and parameter lists */
    rtapi_mutex_get(&(hal_data->mutex));
    next = hal_data->pin_list_ptr;
    match_tab = -1;
    match_row = 0;
    row = 0;
    tab = 0;
    while (next != 0) {
        pin = SHMPTR(next);
        *name = pin->name;

        add_to_list(vert->lists[tab], name, NUM_COLS);
        if (chan->name == *name) {
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

        add_to_list(vert->lists[tab], name, NUM_COLS);
        if (chan->name == *name) {
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

        add_to_list(vert->lists[tab], name, NUM_COLS);
        if (chan->name == *name) {
            match_tab = tab;
            match_row = row;
        }
        next = param->next_ptr;
        row++;
    }

    rtapi_mutex_give(&(hal_data->mutex));

    gtk_widget_show_all(dialog);

    /* highlight the currently selected name */
    /* set scrolling window to show the highlighted name */
    if (match_tab != -1) {
        gtk_notebook_set_current_page(GTK_NOTEBOOK(vert->notebook), match_tab);
        mark_selected_row(vert->lists[match_tab], match_row);
    }

    retval = gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);

    if (retval == GTK_RESPONSE_ACCEPT) {
        if (set_channel_source(vert->chan_num, vert->listnum, signal_name))
            channel_changed();
        return TRUE;
    }
    return FALSE;
}

static void selection_changed(GtkTreeSelection *selection, char *name)
{
    GtkTreeIter iter;
    GtkTreeModel *model;
    char *tmp;

    if (gtk_tree_selection_get_selected(selection, &model, &iter)) {
        gtk_tree_model_get(model, &iter, LIST_ITEM, &tmp, -1);
        rtapi_strlcpy(name, tmp, HAL_NAME_LEN);
        g_free(tmp);
    }
}

/* User has double-clicked or hit 'enter' on a row in the list. */
static void selection_made(GtkTreeView *treeview, GtkTreePath *path,
                           GtkTreeViewColumn *col, GtkWidget *dialog)
{
    (void)treeview;
    (void)path;
    (void)col;
    gtk_dialog_response(GTK_DIALOG(dialog), GTK_RESPONSE_ACCEPT);
}

void channel_changed(void)
{
    scope_vert_t *vert;
    scope_chan_t *chan;
    GtkAdjustment *adj;
    gchar *name;
    gchar buf1[BUFLEN + 1], buf2[BUFLEN + 1];
    static int last_channel = 0;
    vert = &(ctrl_usr->vert);
    /* add a name to apply CSS for highlighted channel */
    if (last_channel != vert->selected) {
        if (last_channel) {
            gtk_widget_set_name(chan_buttons[last_channel-1],"");
        }
        if (vert->selected) {
            gtk_widget_set_name(chan_buttons[vert->selected-1],"selected");
        }
        last_channel = vert->selected;
    }
    if ((vert->selected < 1) || (vert->selected > 16)) {
        gtk_label_set_text_if(vert->scale_label, "----");
        gtk_label_set_text_if(vert->chan_num_label, "--");
        gtk_label_set_text_if(vert->source_name_label, "------");
        request_display_refresh(1);
        return;
    }
    chan = &(ctrl_usr->chan[vert->selected - 1]);
    /* set position slider based on new channel */
    gtk_adjustment_set_value(GTK_ADJUSTMENT(vert->pos_adj),
                             chan->position * VERT_POS_RESOLUTION);
    /* set scale slider based on new channel */
    adj = GTK_ADJUSTMENT(vert->scale_adj);
    gtk_adjustment_set_lower(adj, chan->min_index);
    gtk_adjustment_set_upper(adj, chan->max_index);
    gtk_adjustment_set_value(adj, chan->scale_index);

    // Call the `scale_changed()` callback once by hand to initialize
    // the new channel's scale/gain.
    scale_changed(adj, NULL);

    /* update the channel number and name display */
    snprintf(buf1, BUFLEN, "%2d", vert->selected);
    name = chan->name;
    gtk_label_set_text_if(vert->chan_num_label, buf1);
    gtk_label_set_text_if(vert->source_name_label, name);
    /* update the offset display */
    if (chan->data_type == HAL_BIT) {
	    snprintf(buf1, BUFLEN, "----");
    } else {
        if(chan->ac_offset) {
            snprintf(buf1, BUFLEN, "(AC)");
        } else {
            format_signal_value(buf1, BUFLEN, chan->vert_offset);
        }
    }
    snprintf(buf2, BUFLEN, _("Offset\n%s"), buf1);
    gtk_label_set_text_if(vert->offset_label, buf2);
    request_display_refresh(1);
}

void format_scale_value(char *buf, int buflen, double value)
{
    char *units;
    char symbols[] = "pnum KMGT";

    if (value < 0.9e-12) {
	/* less than pico units, shouldn't happen */
	snprintf(buf, buflen, "tiny");
	return;
    }
    if (value > 1.1e+12) {
	/* greater than tera-units, shouldn't happen */
	snprintf(buf, buflen, "huge");
	return;
    }
    units = &(symbols[4]);
    while (value < 1.0) {
	value *= 1000.0;
	units--;
    }
    while (value >= 999.99) {
	value *= 0.001;
	units++;
    }
    snprintf(buf, buflen, "%0.0f%c/div", value, *units);
}

static void write_chan_config(FILE *fp, scope_chan_t *chan)
{
    if ( chan->data_source_type == 0 ) {
	// pin
	fprintf(fp, "PIN %s\n", chan->name);
    } else if ( chan->data_source_type == 1 ) {
	// signal
	fprintf(fp, "SIG %s\n", chan->name);
    } else if ( chan->data_source_type == 2 ) {
	// pin
	fprintf(fp, "PARAM %s\n", chan->name);
    } else {
	// not configured
	return;
    }
    fprintf(fp, "VSCALE %d\n", chan->scale_index);
    fprintf(fp, "VPOS %f\n", chan->position);
    if(chan->ac_offset) {
        fprintf(fp, "VAC %e\n", chan->vert_offset);
    } else {
        fprintf(fp, "VOFF %e\n", chan->vert_offset);
    }
}

/*
 * Inline css, set color to  channel select buttons.
 */
static void style_with_css(GtkWidget *widget, int color_index)
{
    GtkStyleContext *context;
    GtkCssProvider *provider;

    char buf[270];
    snprintf(buf, sizeof(buf), "* {margin: 1px; border-style:solid; border-width: 2px;}\n"
                               "#selected {border-color: black; font-weight: bold;}\n"
                               "*:checked, *:active {background: rgb(%d,%d,%d);}\n"
                               "*:hover {background: rgba(%d,%d,%d,0.3);}\n"
                               "*:hover#selected {background: rgba(%d,%d,%d,0.6);}\n",
                               normal_colors[color_index][0],normal_colors[color_index][1],
                               normal_colors[color_index][2],
                               normal_colors[color_index][0],normal_colors[color_index][1],
                               normal_colors[color_index][2],
                               normal_colors[color_index][0],normal_colors[color_index][1],
                               normal_colors[color_index][2]);

    provider = gtk_css_provider_new ();
    context = gtk_widget_get_style_context(widget);
    gtk_style_context_add_provider(context, GTK_STYLE_PROVIDER(provider),
            GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    gtk_css_provider_load_from_data(provider, buf, -1, NULL);

    g_object_unref(provider);
}

