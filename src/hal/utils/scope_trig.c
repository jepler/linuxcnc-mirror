/** This file, 'scope_trig.c', contains the portion of halscope
    that deals with triggering
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
#include "hal.h"		/* HAL public API decls */
#include "../hal_priv.h"	/* private HAL decls */

#include <gtk/gtk.h>
#include "miscgtk.h"		/* generic GTK stuff */
#include "scope_usr.h"		/* scope related declarations */

#define BUFLEN 80		/* length for sprintf buffers */

/***********************************************************************
*                  GLOBAL VARIABLES DECLARATIONS                       *
************************************************************************/

#define TRIG_LEVEL_RESOLUTION 100.0
#define TRIG_POS_RESOLUTION 100.0

/* Columns in the TreeView */
enum TREEVIEW_COLUMN {
    COL_CHAN,
    COL_SOURCE,
    NUM_COLS
};

/***********************************************************************
*                  LOCAL FUNCTION PROTOTYPES                           *
************************************************************************/

static void init_trigger_mode_window(void);
static void init_trigger_info_window(void);

static void trigger_selection_made(GtkWidget *treeview, GdkEventButton *event,
                                   GtkWidget *dialog);
static void dialog_select_trigger_source(void);

/* callback functions */
static void auto_button_clicked(GtkWidget * widget, gpointer * gdata);
static void normal_button_clicked(GtkWidget * widget, gpointer * gdata);
static void force_button_clicked(GtkWidget * widget, gpointer * gdata);
static void source_button_clicked(GtkWidget * widget, gpointer * gdata);
static void edge_button_clicked(GtkWidget * widget, gpointer * gdata);
static void level_changed(GtkAdjustment * adj, gpointer gdata);
static void pos_changed(GtkAdjustment * adj, gpointer gdata);

/***********************************************************************
*                       PUBLIC FUNCTIONS                               *
************************************************************************/

void init_trig(void)
{
    /* stop sampling */
    ctrl_shm->state = IDLE;
    /* init non-zero members of the trigger structure */

    /* set up the windows */
    init_trigger_mode_window();
    init_trigger_info_window();
}

void refresh_trigger(void)
{
    scope_trig_t *trig;
    scope_chan_t *chan;
    gchar buf[BUFLEN + 1];
    double fp_level;

    trig = &(ctrl_usr->trig);
    /* display edge */
    if (ctrl_shm->trig_edge == 0) {
	snprintf(buf, BUFLEN, _("Falling"));
    } else {
	snprintf(buf, BUFLEN, _("Rising"));
    }
    gtk_label_set_text_if(trig->edge_label, buf);
    /* display source */
    if ((ctrl_shm->trig_chan < 1) || (ctrl_shm->trig_chan > 16)) {
	/* no source */
	ctrl_shm->trig_chan = 0;
	gtk_label_set_text_if(trig->source_label, _("Source\nNone"));
	gtk_label_set_text_if(trig->level_label, "  ----  ");
	/* nothing left to do */
	return;
    }
    snprintf(buf, BUFLEN, _("Source\nChan %2d"), ctrl_shm->trig_chan);
    gtk_label_set_text_if(trig->source_label, buf);
    /* point to source channel data */
    chan = &(ctrl_usr->chan[ctrl_shm->trig_chan - 1]);
    /* calculate a preliminary value for trigger level */
    fp_level =
	chan->scale * ((chan->position - trig->level) * 10) +
	chan->vert_offset;
    /* apply type specific tweaks to trigger level */
    switch (chan->data_type) {
    case HAL_FLOAT:
	ctrl_shm->trig_level.d_real = fp_level;
	break;
    case HAL_S32:
	if (fp_level > 2147483647.0) {
	    fp_level = 2147483647.0;
	}
	if (fp_level < -2147483648.0) {
	    fp_level = -2147483648.0;
	}
	ctrl_shm->trig_level.d_s32 = fp_level;
	break;
    case HAL_U32:
	if (fp_level > 4294967295.0) {
	    fp_level = 4294967295.0;
	}
	if (fp_level < 0.0) {
	    fp_level = 0.0;
	}
	ctrl_shm->trig_level.d_u32 = fp_level;
	break;
    default:
	break;
    }
    if (chan->data_type == HAL_BIT) {
	snprintf(buf, BUFLEN, "  ----  ");
        gtk_widget_set_sensitive(GTK_WIDGET(trig->level_slider), 0);
    } else {
	format_signal_value(buf, BUFLEN, fp_level);
        gtk_widget_set_sensitive(GTK_WIDGET(trig->level_slider), 1);
    }
    gtk_label_set_text_if(trig->level_label, buf);
    redraw_window();
}

void write_trig_config(FILE *fp)
{
    scope_trig_t *trig;

    trig = &(ctrl_usr->trig);
    if (ctrl_shm->trig_chan > 0) {
	fprintf(fp, "TSOURCE %d\n", ctrl_shm->trig_chan);
	fprintf(fp, "TLEVEL %f\n", trig->level);
	fprintf(fp, "TPOS %f\n", trig->position);
	fprintf(fp, "TPOLAR %d\n", ctrl_shm->trig_edge);
    }
    fprintf(fp, "TMODE %d\n", ctrl_shm->auto_trig);
}

/***********************************************************************
*                       LOCAL FUNCTIONS                                *
************************************************************************/

static void init_trigger_mode_window(void)
{
    scope_trig_t *trig;

    /* make a pointer to the trig structure */
    trig = &(ctrl_usr->trig);
    /* set initial state to normal */
    ctrl_shm->auto_trig = 0;
    /* define the radio buttons */
    trig->normal_button = gtk_radio_button_new_with_label(NULL, _("Normal"));
    trig->auto_button = gtk_radio_button_new_with_label_from_widget(
        GTK_RADIO_BUTTON(trig->normal_button), _("Auto"));
    /* and a regular button */
    trig->force_button = gtk_button_new_with_label(_("Force"));
    /* now put them into the box */
    gtk_box_pack_start(GTK_BOX(ctrl_usr->trig_mode_win),
	trig->normal_button, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(ctrl_usr->trig_mode_win),
	trig->auto_button, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(ctrl_usr->trig_info_win),
	trig->force_button, FALSE, FALSE, 0);
    /* hook callbacks to buttons */
    g_signal_connect(trig->normal_button, "clicked",
	G_CALLBACK(normal_button_clicked), NULL);
    g_signal_connect(trig->auto_button, "clicked",
	G_CALLBACK(auto_button_clicked), NULL);
    g_signal_connect(trig->force_button, "clicked",
	G_CALLBACK(force_button_clicked), NULL);
    /* and make them visible */
    gtk_widget_show(trig->normal_button);
    gtk_widget_show(trig->auto_button);
    gtk_widget_show(trig->force_button);
}

static void init_trigger_info_window(void)
{
    scope_trig_t *trig;
    GtkWidget *hbox, *vbox;

    /* make a pointer to the trigger structure */
    trig = &(ctrl_usr->trig);
    /* box for the two sliders */
    hbox =
	gtk_hbox_new_in_box(TRUE, 0, 0, ctrl_usr->trig_info_win, TRUE, TRUE,
	0);
    /* box for the level slider */
    vbox = gtk_vbox_new_in_box(FALSE, 0, 0, hbox, TRUE, TRUE, 0);
    gtk_label_new_in_box(_("Level"), vbox, FALSE, FALSE, 0);
    trig->level_adj =
	gtk_adjustment_new(TRIG_LEVEL_RESOLUTION / 2, 0,
	TRIG_LEVEL_RESOLUTION, 1, 1, 0);
    trig->level_slider = gtk_scale_new(
            GTK_ORIENTATION_VERTICAL, GTK_ADJUSTMENT(trig->level_adj));
    gtk_scale_set_digits(GTK_SCALE(trig->level_slider), 0);
    gtk_scale_set_draw_value(GTK_SCALE(trig->level_slider), FALSE);
    gtk_box_pack_start(GTK_BOX(vbox), trig->level_slider, TRUE, TRUE, 0);
    /* set initial trigger level */
    trig->level = gtk_adjustment_get_value(
            GTK_ADJUSTMENT(trig->level_adj)) / TRIG_LEVEL_RESOLUTION;
    /* connect the slider to a function that re-sets the trigger level */
    g_signal_connect(trig->level_adj, "value_changed",
	G_CALLBACK(level_changed), NULL);
    gtk_widget_show(trig->level_slider);
    /* box for the position slider */
    vbox = gtk_vbox_new_in_box(FALSE, 0, 0, hbox, TRUE, TRUE, 0);
    gtk_label_new_in_box(_("Pos"), vbox, FALSE, FALSE, 0);
    trig->pos_adj =
	gtk_adjustment_new(TRIG_POS_RESOLUTION / 2, 0, TRIG_POS_RESOLUTION, 1,
	1, 0);
    trig->pos_slider = gtk_scale_new(
            GTK_ORIENTATION_VERTICAL, GTK_ADJUSTMENT(trig->pos_adj));
    gtk_scale_set_digits(GTK_SCALE(trig->pos_slider), 0);
    gtk_scale_set_draw_value(GTK_SCALE(trig->pos_slider), FALSE);
    gtk_box_pack_start(GTK_BOX(vbox), trig->pos_slider, TRUE, TRUE, 0);
    /* set initial trigger position */
    trig->position = gtk_adjustment_get_value(
            GTK_ADJUSTMENT(trig->pos_adj)) / TRIG_POS_RESOLUTION;
    /* connect the slider to a function that re-sets trigger position */
    g_signal_connect(trig->pos_adj, "value_changed",
	G_CALLBACK(pos_changed), NULL);
    gtk_widget_show(trig->pos_slider);
    /* level display */
    gtk_hseparator_new_in_box(ctrl_usr->trig_info_win, 3);
    gtk_label_new_in_box(_("Level"), ctrl_usr->trig_info_win, FALSE, FALSE, 0);
    trig->level_label =
	gtk_label_new_in_box(" ---- ", ctrl_usr->trig_info_win, FALSE, FALSE,
	0);
    /* define a button to set the trigger edge */
    ctrl_shm->trig_edge = 1;
    trig->edge_button = gtk_button_new_with_label(_("Rising"));
    trig->edge_label = gtk_bin_get_child(GTK_BIN(trig->edge_button));
    gtk_box_pack_start(GTK_BOX(ctrl_usr->trig_info_win),
	trig->edge_button, FALSE, FALSE, 0);
    g_signal_connect(trig->edge_button, "clicked",
	G_CALLBACK(edge_button_clicked), NULL);
    gtk_widget_show(trig->edge_button);
    /* define a button to set the trigger source */
    trig->source_button = gtk_button_new_with_label(_("Source\nNone"));
    trig->source_label = gtk_bin_get_child(GTK_BIN(trig->source_button));
    gtk_box_pack_start(GTK_BOX(ctrl_usr->trig_info_win),
	trig->source_button, FALSE, FALSE, 0);
    g_signal_connect(trig->source_button, "clicked",
	G_CALLBACK(source_button_clicked), NULL);
    gtk_widget_show(trig->source_button);
}

static void dialog_select_trigger_source(void)
{
    char *msg;
    int n;
    char *strs[2], *titles[NUM_COLS];
    char buf[BUFLEN + 1];
    GtkWidget *label, *scrolled_window, *trig_list;
    GtkWidget *content_area;
    GtkWidget *dialog;

    /* is acquisition in progress? */
    if (ctrl_shm->state != IDLE) { prepare_scope_restart(); }
    msg = _("Select a channel to use for triggering.");

    /* create dialog window, disable resizing, set size and title */
    dialog= gtk_dialog_new_with_buttons(_("Trigger Source"),
                                        NULL, GTK_DIALOG_MODAL,
                                        _("_Cancel"), GTK_RESPONSE_CANCEL,
                                        NULL);
    gtk_widget_set_size_request(GTK_WIDGET(dialog), -1, 400);
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));

    /* display message */
    label = gtk_label_new(msg);
    gtk_widget_set_margin_start(label, 15);
    gtk_widget_set_margin_end(label, 15);
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
            label, FALSE, TRUE, 5);

    /* Create a scrolled window to display the list */
    scrolled_window = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled_window),
	GTK_POLICY_AUTOMATIC, GTK_POLICY_ALWAYS);
    gtk_box_pack_start(GTK_BOX(GTK_CONTAINER(content_area)),
            scrolled_window, TRUE, TRUE, 5);

    /* create a list to hold the data */
    titles[0] = _("Chan");
    titles[1] = _("Source");
    trig_list = gtk_tree_view_new();
    init_list(trig_list, titles, NUM_COLS);
    gtk_container_add(GTK_CONTAINER(scrolled_window), trig_list);

    g_signal_connect(trig_list, "button-press-event",
            G_CALLBACK(trigger_selection_made), dialog);

    /* populate the trigger source list */
    for (n = 0; n < 16; n++) {
	snprintf(buf, BUFLEN, "%d", n + 1);
	strs[0] = buf;
	if (ctrl_usr->chan[n].name != NULL) {
	    strs[1] = ctrl_usr->chan[n].name;
	} else {
	    strs[1] = "----";
	}
        add_to_list(trig_list, strs, NUM_COLS);
    }

    /* was a channel previously selected? */
    if (ctrl_shm->trig_chan > 0) {
	/* yes, preselect appropriate line */
        mark_selected_row(trig_list, ctrl_shm->trig_chan - 1);
    }
    gtk_widget_show_all(dialog);

    gtk_dialog_run(GTK_DIALOG(dialog));
    /* we get here when the user makes a selection, hits Cancel, or closes
       the window */
    gtk_widget_destroy(dialog);
    trig_list = NULL;
}

int set_trigger_source(int chan_num)
{
    ctrl_shm->trig_chan = chan_num;
    if (ctrl_usr->chan[ctrl_shm->trig_chan - 1].name == NULL) {
	/* selected channel has no source */
	ctrl_shm->trig_chan = 0;
    }
    refresh_trigger();
    return 0;
}

/* If we come here, then the user has selected a row in the list. */
static void trigger_selection_made(GtkWidget *treeview, GdkEventButton *event,
                                   GtkWidget *dialog)
{
    if (event->type == GDK_BUTTON_PRESS && event->button == 1) {
        GtkTreePath *path;

        if (gtk_tree_view_get_path_at_pos(GTK_TREE_VIEW(treeview),
                    (int) event->x, (int) event->y, &path, NULL, NULL, NULL)) {
            int *row;

            row = gtk_tree_path_get_indices(path);
            set_trigger_source(row[0] + 1);

            /* set return value of dialog and close window */
            gtk_dialog_response(GTK_DIALOG(dialog), GTK_RESPONSE_ACCEPT);
        }
    }
}

int set_trigger_level(double setting)
{
    scope_trig_t *trig;
    GtkAdjustment *adj;

    /* range check setting */
    if (( setting < 0.0 ) || ( setting > 1.0 )) {
	return -1;
    }
    /* point to data */
    trig = &(ctrl_usr->trig);
    /* set level */
    trig->level = setting;
    /* set level slider based on new setting */
    adj = GTK_ADJUSTMENT(trig->level_adj);
    gtk_adjustment_set_value(adj, trig->level * TRIG_LEVEL_RESOLUTION);
    refresh_trigger();
    return 0;
}

static void level_changed(GtkAdjustment * adj, gpointer gdata)
{
    (void)gdata;
    set_trigger_level(gtk_adjustment_get_value(adj) / TRIG_LEVEL_RESOLUTION);
}

int set_trigger_pos(double setting)
{
    scope_trig_t *trig;
    GtkAdjustment *adj;

    /* range check setting */
    if (( setting < 0.0 ) || ( setting > 1.0 )) {
	return -1;
    }
    /* point to data */
    trig = &(ctrl_usr->trig);
    /* is acquisition in progress? */
    if (ctrl_shm->state != IDLE) {
	/* yes, prepare to restart the scope */
        prepare_scope_restart();
    }
    /* set position */
    trig->position = setting;
    /* set position slider based on new setting */
    adj = GTK_ADJUSTMENT(trig->pos_adj);
    gtk_adjustment_set_value(adj, trig->position * TRIG_POS_RESOLUTION);
    refresh_trigger();
    return 0;
}

static void pos_changed(GtkAdjustment * adj, gpointer gdata)
{
    (void)gdata;
    set_trigger_pos(gtk_adjustment_get_value(adj) / TRIG_POS_RESOLUTION);
}

int set_trigger_polarity(int setting)
{
    if (setting == 0) {
	ctrl_shm->trig_edge = 0;
    } else if ( setting == 1 ) {
	ctrl_shm->trig_edge = 1;
    } else {
	return -1;
    }
    refresh_trigger();
    return 0;
}

static void edge_button_clicked(GtkWidget * widget, gpointer * gdata)
{
    (void)widget;
    (void)gdata;
    if (ctrl_shm->trig_edge == 0) {
	/* was falling edge, make rising */
	set_trigger_polarity(1);
    } else {
	/* was rising edge, make falling */
	set_trigger_polarity(0);
    }
}

int set_trigger_mode(int mode)
{
    GtkWidget *button;

    if ( mode == 0 ) {
	/* normal mode */
	button = ctrl_usr->trig.normal_button;
    } else if ( mode == 1 ) {
	/* auto mode */
	button = ctrl_usr->trig.auto_button;
    } else {
	/* illegal mode */
	return -1;
    }
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(button), 1);
    return 0;
}

static void normal_button_clicked(GtkWidget * widget, gpointer * gdata)
{
    (void)gdata;
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget)) != TRUE) {
	/* not pressed, ignore it */
	return;
    }
    ctrl_shm->auto_trig = 0;
}

static void auto_button_clicked(GtkWidget * widget, gpointer * gdata)
{
    (void)gdata;
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget)) != TRUE) {
	/* not pressed, ignore it */
	return;
    }
    ctrl_shm->auto_trig = 1;
}

static void force_button_clicked(GtkWidget * widget, gpointer * gdata)
{
    (void)widget;
    (void)gdata;
    ctrl_shm->force_trig = 1;
}

static void source_button_clicked(GtkWidget * widget, gpointer * gdata)
{
    (void)widget;
    (void)gdata;
    dialog_select_trigger_source();
}
