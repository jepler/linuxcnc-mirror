/** This file, 'scope.c', is a GUI program that together with
    'scope_rt.c' serves as an oscilloscope to examine HAL pins,
    signals, and parameters.  It is a user space component and
    uses GTK 3.0 for the GUI code.
*/

static char *license = \
"              Copyright (C) 2003 John Kasunich\n\
                       <jmkasunich AT users DOT sourceforge DOT net>\n\
\n\
    This program is free software; you can redistribute it and/or\n\
    modify it under the terms of version 2 of the GNU General\n\
    Public License as published by the Free Software Foundation.\n\
    This library is distributed in the hope that it will be useful,\n\
    but WITHOUT ANY WARRANTY; without even the implied warranty of\n\
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n\
    GNU General Public License for more details.\n\
\n\
    You should have received a copy of the GNU General Public\n\
    License along with this library; if not, write to the Free Software\n\
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.\n\
\n\
    THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR\n\
    ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE\n\
    TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of\n\
    harming persons must have provisions for completely removing power\n\
    from all motors, etc, before persons enter any danger area.  All\n\
    machinery must be designed to comply with local and national safety\n\
    codes, and the authors of this software can not, and do not, take\n\
    any responsibility for such compliance.\n\
\n\
\n\
    This code was written as part of the EMC HAL project.  For more\n\
    information, go to https://linuxcnc.org\n\
";

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
#include <unistd.h>	/* getopt() */

#include "config.h"
#include "rtapi.h"		/* RTAPI realtime OS API */
#include "hal.h"		/* HAL public API decls */
#include "../hal_priv.h"	/* HAL private API decls */

#include <gtk/gtk.h>
#include "miscgtk.h"		/* generic GTK stuff */
#include "scope_usr.h"		/* scope related declarations */
#include <rtapi_string.h>

/***********************************************************************
*                         GLOBAL VARIABLES                             *
************************************************************************/

scope_usr_control_t *ctrl_usr;	/* ptr to main user control structure */
scope_shm_control_t *ctrl_shm;	/* ptr to shared mem control struct */

/***********************************************************************
*                         LOCAL VARIABLES                              *
************************************************************************/

static int comp_id;		/* component ID */
static int shm_id;		/* shared memory ID */
static scope_usr_control_t ctrl_struct;	/* scope control structure */

/***********************************************************************
*                  LOCAL FUNCTION PROTOTYPES                           *
************************************************************************/

/* init functions */
static void init_usr_control_struct(void *shmem);

static void define_scope_windows(void);
static void init_run_mode_window(void);

/* callback functions */
static void exit_from_hal(void);
static void main_window_closed(GtkWidget * widget, gpointer * gdata);
static void set_focus(GtkWindow * window, GtkWidget *widget, gpointer * gdata);
static void quit(int sig);
static int heartbeat(gpointer data);
static void rm_normal_button_clicked(GtkWidget * widget, gpointer * gdata);
static void rm_single_button_clicked(GtkWidget * widget, gpointer * gdata);
static void rm_roll_button_clicked(GtkWidget * widget, gpointer * gdata);
static void rm_stop_button_clicked(GtkWidget * widget, gpointer * gdata);

static void exit_on_signal(int signum) {
    fprintf(stderr,"%s Caught signum=%d <%s>\n   killing userspace comp_id=%d\n"
           ,__FILE__,signum,strsignal(signum),comp_id);
    exit_from_hal();
    exit(1);
}
/***********************************************************************
*                        MAIN() FUNCTION                               *
************************************************************************/
static void *shm_base;


int main(int argc, gchar * argv[])
{
    int retval;
    int num_samples = SCOPE_NUM_SAMPLES_DEFAULT;
    char *ifilename = "autosave.halscope";
    char *ofilename = "autosave.halscope";

    bindtextdomain("linuxcnc", EMC2_PO_DIR);
    setlocale(LC_MESSAGES,"");
    setlocale(LC_CTYPE,"");
    textdomain("linuxcnc");

    /* process and remove any GTK specific command line args */
    gtk_init(&argc, &argv);
    /* process halscope command line args (if any) here */


    while(1) {
        int c;
        c = getopt(argc, argv, "hi:o:");
        if(c == -1) break;
        switch(c) {
         case 'h':
            rtapi_print_msg(RTAPI_MSG_ERR,
            _("Usage:\n  halscope [-h] [-i infile] [-o outfile]"
            " [num_samples]\n"));
            return -1;
            break;
         case 'i':
            ifilename = optarg;
            break;
         case 'o':
            ofilename = optarg;
            break;
        }
    }
    if(argc > optind) num_samples = atoi(argv[argc-1]);
    if(num_samples <= 0)
	num_samples = SCOPE_NUM_SAMPLES_DEFAULT;

    /* connect to the HAL */
    comp_id = hal_init("halscope");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR, "SCOPE: ERROR: hal_init() failed\n");
	return -1;
    }

    if (!halpr_find_funct_by_name("scope.sample")) {
	char buf[1000];
	snprintf(buf, sizeof(buf), EMC2_BIN_DIR "/halcmd loadrt scope_rt num_samples=%d",
		num_samples);
	if(system(buf) != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR, "loadrt scope_rt failed\n");
	    hal_exit(comp_id);
	    exit(1);
	}
    }
    /* set up a shared memory region for the scope data */
    shm_id = rtapi_shmem_new(SCOPE_SHM_KEY, comp_id, sizeof(scope_shm_control_t));
    if (shm_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SCOPE: ERROR: failed to get shared memory (key=0x%x, size=%lu)\n",
            SCOPE_SHM_KEY,
            (unsigned long)sizeof(scope_shm_control_t)
        );
	hal_exit(comp_id);
	return -1;
    }
    retval = rtapi_shmem_getptr(shm_id, &shm_base);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "SCOPE: ERROR: failed to map shared memory\n");
	rtapi_shmem_delete(shm_id, comp_id);
	hal_exit(comp_id);
	return -1;
    }

    hal_ready(comp_id);

    /* register an exit function to cleanup and disconnect from the HAL */
    atexit(exit_from_hal);

    /* init control structure */
    ctrl_usr = &ctrl_struct;
    init_usr_control_struct(shm_base);

    /* init watchdog */
    ctrl_shm->watchdog = 10;
    /* set main window */
    define_scope_windows();
    /* this makes the application exit when the window is closed */
    g_signal_connect(ctrl_usr->main_win, "destroy",
	G_CALLBACK(main_window_closed), NULL);
    g_signal_connect(ctrl_usr->main_win, "focus-in-event",
	G_CALLBACK(set_focus), NULL);
    /* define menu windows */
    /* do next level of init */
    init_horiz();
    init_vert();
    init_trig();
    init_display();
    init_run_mode_window();
    /* register signal handlers for ctrl-C and SIGTERM */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);
    signal(SIGSEGV, exit_on_signal);
    signal(SIGFPE,  exit_on_signal);

    /* The interface is now completely set up */
    /* show the window */
    gtk_widget_show(ctrl_usr->main_win);
    /* read the saved config file */
    read_config_file(ifilename);
    /* arrange for periodic call of heartbeat() */
    g_timeout_add(100, heartbeat, NULL);
    /* enter the main loop */
    gtk_main();
    write_config_file(ofilename);

    return (0);
}

/* This function is called approximately 10 times per second.
   It is responsible for updating anything that is supposed to be
   "live", such as the state display.  It also watches for asynchronous
   events that need to be dealt with, such as loss of the realtime
   component of the scope.
*/

static int heartbeat(gpointer data)
{
    (void)data;
    refresh_state_info();
    /* check watchdog */
    if (ctrl_shm->watchdog < 10) {
	ctrl_shm->watchdog++;
    } else {
	handle_watchdog_timeout();
    }
    if (ctrl_usr->pending_restart && ctrl_shm->state == IDLE) {
        ctrl_usr->pending_restart = 0;
        ctrl_usr->run_mode = ctrl_usr->old_run_mode;
        if(ctrl_usr->run_mode != STOP) {
            start_capture();
        }
    }
    if (ctrl_usr->display_refresh_timer > 0) {
	/* decrement timer, did it time out? */
	if (--ctrl_usr->display_refresh_timer == 0) {
	    /* yes, refresh the display */
	    redraw_window();
	}
    }
    if (ctrl_shm->state == DONE) {
        if(!gtk_window_is_active(GTK_WINDOW(ctrl_usr->main_win)))
            gtk_window_set_urgency_hint(GTK_WINDOW(ctrl_usr->main_win), TRUE);
	capture_complete();
    } else if (ctrl_usr->run_mode == ROLL) capture_cont();
    return 1;
}

void start_capture(void)
{
    int n;
    scope_chan_t *chan;
    hal_pin_t *pin;
    hal_sig_t *sig;
    hal_param_t *param;

    if (ctrl_shm->state != IDLE) {
	/* already running! */
	return;
    }
    for (n = 0; n < 16; n++) {
	/* point to user space channel data */
	chan = &(ctrl_usr->chan[n]);
	/* find address of data in shared memory */
	if ( chan->data_source_type == 0 ) {
	    /* channel source is a pin, point at it */
	    pin = SHMPTR(chan->data_source);
	    /* make sure it's still valid */
	    if ( pin->name[0] == '\0' ) {
		/* pin has been deleted */
		chan->data_source_type = -1;
		chan->data_len = 0;
		break;
	    }
	    /* point at pin data */
	    if (pin->signal == 0) {
		/* pin is unlinked, get data from dummysig */
		ctrl_shm->data_offset[n] = SHMOFF(&(pin->dummysig));
	    } else {
		/* pin is linked to a signal */
		sig = SHMPTR(pin->signal);
		ctrl_shm->data_offset[n] = sig->data_ptr;
	    }
	} else if ( chan->data_source_type == 1 ) {
	    /* channel source is a signal, point at it */
	    sig = SHMPTR(chan->data_source);
	    /* make sure it's still valid */
	    if ( sig->name[0] == '\0' ) {
		/* signal has been deleted */
		chan->data_source_type = -1;
		chan->data_len = 0;
		break;
	    }
	    ctrl_shm->data_offset[n] = sig->data_ptr;
	} else if ( chan->data_source_type == 2 ) {
	    /* channel source is a parameter, point at it */
	    param = SHMPTR(chan->data_source);
	    /* make sure it's still valid */
	    if ( param->name[0] == '\0' ) {
		/* param has been deleted */
		chan->data_source_type = -1;
		chan->data_len = 0;
		break;
	    }
	    ctrl_shm->data_offset[n] = param->data_ptr;
	} else {
	    /* channel source is invalid */
	    chan->data_len = 0;
	}
	/* set data type */
	ctrl_shm->data_type[n] = chan->data_type;
	/* set data length - zero means don't sample */
	if (ctrl_usr->vert.chan_enabled[n]) {
	    ctrl_shm->data_len[n] = chan->data_len;
	} else {
	    ctrl_shm->data_len[n] = 0;
	}
    }
    ctrl_shm->pre_trig = (ctrl_shm->rec_len-2) * ctrl_usr->trig.position;
    ctrl_shm->state = INIT;
}

void capture_copy_data(void) {
    int n, offs;
    scope_data_t *src, *dst, *src_end;
    int samp_len, samp_size;

    offs = 0;
    for (n = 0; n < 16; n++) {
	if (ctrl_shm->data_len[n] > 0) {
	    /* this channel has valid data */
	    ctrl_usr->vert.data_offset[n] = offs;
	    offs++;
	} else {
	    /* this channel was not acquired */
	    ctrl_usr->vert.data_offset[n] = -1;
	}
    }
    /* copy data from shared buffer to display buffer */
    ctrl_usr->samples = ctrl_shm->samples;
    samp_len = ctrl_shm->sample_len;
    samp_size = samp_len * sizeof(scope_data_t);
    dst = ctrl_usr->disp_buf;
    memset(dst, 0, sizeof(scope_data_t) * ctrl_shm->buf_len);
    src = ctrl_usr->buffer + ctrl_shm->start;
    src_end = ctrl_usr->buffer + (ctrl_shm->rec_len * samp_len);
    n = 0;
    while (n < ctrl_usr->samples) {
	/* copy one sample */
	memcpy(dst, src, samp_size);
	n++;
	dst += samp_len;
	src += samp_len;
	if (src >= src_end) {
	    src = ctrl_usr->buffer;
	}
    }
}

void capture_cont()
{
    capture_copy_data();
    redraw_window();
}

void capture_complete(void)
{
    capture_copy_data();
    ctrl_shm->state = IDLE;
    switch (ctrl_usr->run_mode) {
    case STOP:
	break;
    case NORMAL:
    case ROLL:
	start_capture();
	break;
    case SINGLE:
	/* 'push' the stop button */
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl_usr->
		rm_stop_button), TRUE);
	break;
    default:
	break;
    }

	//uncomment me to write log files
	//write_log_file("scope.log");
    redraw_window();
}

/***********************************************************************
*                      LOCAL INIT FUNCTION CODE                        *
************************************************************************/

static void init_usr_control_struct(void *shmem)
{
    int n, skip;

    /* first clear entire user struct to all zeros */
    memset(ctrl_usr, 0, sizeof(scope_usr_control_t));

    /* save pointer to shared control structure */
    ctrl_shm = shmem;
    /* round size of shared struct up to a multiple of 4 for alignment */
    skip = (sizeof(scope_shm_control_t) + 3) & ~3;
    /* the rest of the shared memory area is the data buffer */
    ctrl_usr->buffer = (scope_data_t *) (((char *) (shmem)) + skip);
    /* is the realtime component loaded already? */
    if (ctrl_shm->shm_size == 0) {
	/* no, this is an error condition */
	rtapi_print_msg(RTAPI_MSG_ERR, "Realtime component not loaded? ctrl_shm->size == 0");
	hal_exit(comp_id);
	exit(1);
    }
    /* init any non-zero fields */
    /* set all 16 channels to "no source assigned" */
    for (n = 0; n < 16; n++) {
	ctrl_usr->chan[n].data_source_type = -1;
    }
    /* done */
}

static void menuitem_response(gchar *string) {
    printf("%s\n", string);
}

static void about(void) {
    gtk_show_about_dialog(GTK_WINDOW(ctrl_usr->main_win),
            "copyright", "Copyright (C) 2003 John Kasunich",
            "license", license,
            "website", "https://linuxcnc.org/",
            NULL);
}

static void do_open_configuration(char *filename)
{
    int n;
    for (n = 0; n < 16; n++) {
	ctrl_usr->chan[n].data_source_type = -1;
        ctrl_usr->chan[n].data_len = 0;
        ctrl_usr->vert.data_offset[n] = -1;
    }
    read_config_file(filename);
    channel_changed();
    redraw_window();
}

static void open_configuration(GtkWindow *parent)
{
    GtkWidget *filew;
    GtkFileChooser *chooser;

    filew = gtk_file_chooser_dialog_new(_("Open Configuration File:"),
                                          parent, GTK_FILE_CHOOSER_ACTION_OPEN,
                                          _("_Cancel"), GTK_RESPONSE_CANCEL,
                                          _("_Open"), GTK_RESPONSE_ACCEPT, NULL);

    chooser = GTK_FILE_CHOOSER(filew);
    set_file_filter(chooser, "Halscope", "*.halscope");

    if (gtk_dialog_run(GTK_DIALOG(filew)) == GTK_RESPONSE_ACCEPT) {
        char *filename;

        filename = gtk_file_chooser_get_filename(chooser);
        do_open_configuration(filename);
        g_free(filename);
    }
    gtk_widget_destroy(filew);
}

static void save_configuration(GtkWindow *parent)
{
    GtkWidget *filew;
    GtkFileChooser *chooser;

    filew = gtk_file_chooser_dialog_new(_("Save Configuration File:"),
                                        parent, GTK_FILE_CHOOSER_ACTION_SAVE,
                                        _("_Cancel"), GTK_RESPONSE_CANCEL,
                                        _("_Save"), GTK_RESPONSE_ACCEPT, NULL);

    chooser = GTK_FILE_CHOOSER(filew);
    set_file_filter(chooser, "Halscope", "*.halscope");
    gtk_file_chooser_set_do_overwrite_confirmation(chooser, TRUE);

    if (gtk_dialog_run(GTK_DIALOG(filew)) == GTK_RESPONSE_ACCEPT) {
        char *filename;

        filename = gtk_file_chooser_get_filename(chooser);
        write_config_file(filename);
        g_free(filename);
    }
    gtk_widget_destroy(filew);
}


static void define_menubar(GtkWidget *vboxtop) {
    GtkWidget *file_rootmenu, *help_rootmenu;
    GtkWidget *menubar, *filemenu,
              *fileopenconfiguration, *filesaveconfiguration,
              *fileopendatafile, *filesavedatafile,
              *filequit, *sep1, *sep2;
    GtkWidget *helpmenu, *helpabout;
    GtkWidget *vbox;

    filemenu = gtk_menu_new();
    helpmenu = gtk_menu_new();
    sep1 = gtk_separator_menu_item_new();
    sep2 = gtk_separator_menu_item_new();

    fileopenconfiguration = gtk_menu_item_new_with_mnemonic(_("_Open Configuration..."));
    gtk_menu_shell_append(GTK_MENU_SHELL(filemenu), fileopenconfiguration);
    g_signal_connect_swapped(fileopenconfiguration, "activate",
            G_CALLBACK(open_configuration), 0);
    gtk_widget_show(fileopenconfiguration);

    filesaveconfiguration = gtk_menu_item_new_with_mnemonic(_("_Save Configuration..."));
    gtk_menu_shell_append(GTK_MENU_SHELL(filemenu), filesaveconfiguration);
    g_signal_connect_swapped(filesaveconfiguration, "activate",
            G_CALLBACK(save_configuration), 0);
    gtk_widget_show(filesaveconfiguration);

    gtk_menu_shell_append(GTK_MENU_SHELL(filemenu), sep1);
    gtk_widget_show(sep1);

    fileopendatafile = gtk_menu_item_new_with_mnemonic(_("O_pen Log File"));
    gtk_menu_shell_append(GTK_MENU_SHELL(filemenu), fileopendatafile);
    g_signal_connect_swapped(fileopendatafile, "activate",
            G_CALLBACK(menuitem_response), "file/open datafile");
    gtk_widget_set_sensitive(GTK_WIDGET(fileopendatafile), FALSE); // XXX
    gtk_widget_show(fileopendatafile);

    filesavedatafile = gtk_menu_item_new_with_mnemonic(_("S_ave Log File"));
    gtk_menu_shell_append(GTK_MENU_SHELL(filemenu), filesavedatafile);
    g_signal_connect_swapped(filesavedatafile, "activate",
            G_CALLBACK(save_log_cb), 0);
    gtk_widget_show(filesavedatafile);

    gtk_menu_shell_append(GTK_MENU_SHELL(filemenu), sep2);
    gtk_widget_show(sep2);

    filequit = gtk_menu_item_new_with_mnemonic(_("_Quit"));
    gtk_menu_shell_append(GTK_MENU_SHELL(filemenu), filequit);
    g_signal_connect_swapped(filequit, "activate",
            G_CALLBACK(quit), 0);
    gtk_widget_show(filequit);

    helpabout = gtk_menu_item_new_with_mnemonic(_("_About Halscope"));
    gtk_menu_shell_append(GTK_MENU_SHELL(helpmenu), helpabout);
    g_signal_connect_swapped(helpabout, "activate",
            G_CALLBACK(about), NULL);
    gtk_widget_show(helpabout);

    file_rootmenu = gtk_menu_item_new_with_mnemonic(_("_File"));
    gtk_widget_show(file_rootmenu);
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(file_rootmenu),filemenu);

    help_rootmenu = gtk_menu_item_new_with_mnemonic(_("_Help"));
    gtk_widget_show(help_rootmenu);
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(help_rootmenu),helpmenu);

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_add(GTK_CONTAINER(vboxtop), vbox);
    gtk_widget_show(vbox);

    menubar = gtk_menu_bar_new();
    gtk_box_pack_start(GTK_BOX(vbox), menubar, FALSE, FALSE, 2);
    gtk_widget_show(menubar);

    gtk_menu_shell_append(GTK_MENU_SHELL(menubar), file_rootmenu);
    gtk_menu_shell_append(GTK_MENU_SHELL(menubar), help_rootmenu);
}

/** 'define_scope_windows()' defines the overall layout of the main
    window.  It does not connect signals or load content into the
    windows - it only creates the windows (actually each "window" is
    either an hbox or vbox).  The layout is as shown below:

    **************************************************************
    * menubar                                                    *
    **************************************************************
    *                horiz_info_win              *  run  * trig  *
    **********************************************  mode * info  *
    *                                        * c *  win  * win   *
    *                                        * h *       *       *
    *                                        * a *       *       *
    *                                        * n *********       *
    *                                        *   *       *       *
    *                waveform_win            * s *       *       *
    *                                        * e *       *       *
    *                                        * l * vert  *       *
    *                                        *   * info  *       *
    *                                        * w * win   *       *
    *                                        * i *       *       *
    *                                        * n *       *       *
    **********************************************       *       *
    *               chan_info_win                *       *       *
    **************************************************************

    Pointers to each of the windows named in the above diagram are
    saved in the control structure.  There are a few other boxes
    used to build the nested windows, but they are not needed later
    so pointers are not saved.
*/

static void define_scope_windows(void)
{
    GtkWidget *vbox, *hbox, *vboxtop, *vboxbottom, *vboxleft, *vboxright, *hboxright;

    /* create main window, set its minimum size and title */
    ctrl_usr->main_win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_widget_set_size_request(GTK_WIDGET(ctrl_usr->main_win), 650, 400);
    gtk_window_set_title(GTK_WINDOW(ctrl_usr->main_win), _("HAL Oscilloscope"));

    /* top level - big vbox, menu above, everything else below */
    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 0);
    gtk_container_add(GTK_CONTAINER(ctrl_usr->main_win), vbox);
    gtk_widget_show(vbox);

    vboxtop = gtk_hbox_new_in_box(FALSE, 0, 0, vbox, FALSE, FALSE, 0);
    vboxbottom = gtk_hbox_new_in_box(FALSE, 0, 0, vbox, TRUE, TRUE, 0);

    /* one big hbox for everything under the menu */
    hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_container_set_border_width(GTK_CONTAINER(hbox), 0);
    /* add the hbox to the main window */
    gtk_container_add(GTK_CONTAINER(vboxbottom), hbox);
    gtk_widget_show(hbox);
    /* end of top level */

    define_menubar(vboxtop);

    /* second level of windows */
    vboxleft = gtk_vbox_new_in_box(FALSE, 0, 0, hbox, TRUE, TRUE, 0);
    hboxright = gtk_hbox_new_in_box(TRUE, 0, 0, hbox, FALSE, FALSE, 0);

    /* third level of windows */
    /* left side */
    ctrl_usr->horiz_info_win =
	gtk_vbox_framed_new_in_box(_("Horizontal"), FALSE, 0, 0, vboxleft, FALSE,
	FALSE, 1);
    /* horizontal row of select buttons */
    ctrl_usr->waveform_win =
	gtk_vbox_new_in_box(FALSE, 0, 0, vboxleft, TRUE, TRUE, 0);
    gtk_widget_set_vexpand(ctrl_usr->waveform_win, TRUE);
    gtk_widget_set_hexpand(ctrl_usr->waveform_win, TRUE);
    ctrl_usr->chan_sel_win =
	gtk_hbox_new_in_box(TRUE, 0, 0, vboxleft, FALSE, FALSE, 0);
    ctrl_usr->chan_info_win =
	gtk_hbox_framed_new_in_box(_("Selected Channel"), FALSE, 0, 0, vboxleft,
	FALSE, FALSE, 0);
    /* right side */
    vboxleft = gtk_vbox_new_in_box(FALSE, 0, 0, hboxright, FALSE, FALSE, 0);
    vboxright = gtk_vbox_new_in_box(FALSE, 0, 0, hboxright, FALSE, FALSE, 0);
    ctrl_usr->run_mode_win =
	gtk_vbox_framed_new_in_box(_("Run Mode"), TRUE, 0, 0, vboxleft, FALSE,
	FALSE, 0);
    ctrl_usr->trig_info_win =
	gtk_vbox_framed_new_in_box(_("Trigger"), FALSE, 0, 0, vboxright, TRUE,
	TRUE, 0);
    ctrl_usr->trig_mode_win =
	gtk_vbox_new_in_box(TRUE, 0, 0, ctrl_usr->trig_info_win, FALSE,
	FALSE, 0);
    ctrl_usr->vert_info_win =
	gtk_vbox_framed_new_in_box(_("Vertical"), FALSE, 0, 0, vboxleft, TRUE,
	TRUE, 0);
    /* all windows are now defined */
}

static void init_run_mode_window(void)
{
    /* define the radio buttons and group them */
    ctrl_usr->rm_stop_button = gtk_radio_button_new_with_label(NULL, _("Stop"));
    ctrl_usr->rm_normal_button = gtk_radio_button_new_with_label_from_widget(
            GTK_RADIO_BUTTON(ctrl_usr->rm_stop_button), _("Normal"));
    ctrl_usr->rm_single_button = gtk_radio_button_new_with_label_from_widget(
            GTK_RADIO_BUTTON(ctrl_usr->rm_stop_button), _("Single"));
    ctrl_usr->rm_roll_button = gtk_radio_button_new_with_label_from_widget(
            GTK_RADIO_BUTTON(ctrl_usr->rm_stop_button), _("Roll"));
    /* now put them into the box */
    gtk_box_pack_start(GTK_BOX(ctrl_usr->run_mode_win),
	ctrl_usr->rm_normal_button, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(ctrl_usr->run_mode_win),
	ctrl_usr->rm_single_button, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(ctrl_usr->run_mode_win),
	ctrl_usr->rm_roll_button, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(ctrl_usr->run_mode_win),
	ctrl_usr->rm_stop_button, FALSE, FALSE, 0);
    /* hook callbacks to buttons */
    g_signal_connect(ctrl_usr->rm_normal_button, "clicked",
            G_CALLBACK(rm_normal_button_clicked), NULL);
    g_signal_connect(ctrl_usr->rm_single_button, "clicked",
            G_CALLBACK(rm_single_button_clicked), NULL);
    g_signal_connect(ctrl_usr->rm_roll_button, "clicked",
            G_CALLBACK(rm_roll_button_clicked), NULL);
    g_signal_connect(ctrl_usr->rm_stop_button, "clicked",
            G_CALLBACK(rm_stop_button_clicked), NULL);
    /* and make them visible */
    gtk_widget_show(ctrl_usr->rm_normal_button);
    gtk_widget_show(ctrl_usr->rm_single_button);
    gtk_widget_show(ctrl_usr->rm_roll_button);
    gtk_widget_show(ctrl_usr->rm_stop_button);
}

/*  FIXME - things not yet finished */

/** roll mode - display updates as frequently as possible, not just
    when acquisition is complete.  Also need to revisit pretrig
    logic - would like to have a full buffer of pretrig samples
    until trigger occurs, then start dropping the oldest ones.
*/

/** cursor area = slider for cursor position, two labels, one for
    timevalue, one for signal value, three buttons [1] [2] [d]
    [1] causes labels to display cursor1 data, slider moves
    cursor1, [2] displays cursor 2 data, slider moves cursor2
    [d] displays delta data, slider moves both cursors
    (can we eliminate sliders and let user click on the waveforms?)
*/

/***********************************************************************
*                    LOCAL CALLBACK FUNCTION CODE                      *
************************************************************************/

static void exit_from_hal(void)
{
    rtapi_shmem_delete(shm_id, comp_id);
    if (comp_id >= 0) hal_exit(comp_id);
    // set comp_id to avoid repeat invocations of hal_exit()
    // when handling signal with atexit() in use:
    comp_id = -1;
}

static void main_window_closed(GtkWidget * widget, gpointer * gdata)
{
    (void)widget;
    (void)gdata;
    quit(0);
}

static void set_focus(GtkWindow *window, GtkWidget *widget, gpointer *data) {
    (void)widget;
    (void)data;
    gtk_window_set_urgency_hint(window, FALSE);
}

static void quit(int sig)
{
    (void)sig;
    gtk_main_quit();
}

int set_run_mode(int mode)
{
    GtkWidget *button;

    if ( mode == 0 ) {
	/* stop mode */
	button = ctrl_usr->rm_stop_button;
    } else if ( mode == 1 ) {
	/* normal mode */
	button = ctrl_usr->rm_normal_button;
    } else if ( mode == 2 ) {
	/* single sweep mode */
	button = ctrl_usr->rm_single_button;
#if 0 /* FIXME - roll mode not implemented yet */
    } else if ( mode == 3 ) {
	/* roll mode */
	button = ctrl_usr->rm_roll_button;
#endif
    } else {
	/* illegal mode */
	return -1;
    }
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(button), 1);
    return 0;
}

static void rm_normal_button_clicked(GtkWidget * widget, gpointer * gdata)
{
    (void)gdata;
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget)) != TRUE) {
	/* not pressed, ignore it */
	return;
    }
    ctrl_usr->run_mode = NORMAL;
    if (ctrl_shm->state == IDLE) {
	start_capture();
    }
}

static void rm_single_button_clicked(GtkWidget * widget, gpointer * gdata)
{
    (void)gdata;
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget)) != TRUE) {
	/* not pressed, ignore it */
	return;
    }
    ctrl_usr->run_mode = SINGLE;
    if (ctrl_shm->state == IDLE) {
	start_capture();
    }
}

static void rm_roll_button_clicked(GtkWidget * widget, gpointer * gdata)
{
    (void)gdata;
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget)) != TRUE) {
	/* not pressed, ignore it */
	return;
    }
    ctrl_usr->run_mode = ROLL;
    if (ctrl_shm->state == IDLE) {
	start_capture();
    }
}

static void rm_stop_button_clicked(GtkWidget * widget, gpointer * gdata)
{
    (void)gdata;
    if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget)) != TRUE) {
	/* not pressed, ignore it */
	return;
    }
    if (ctrl_shm->state != IDLE) {
	/* RT code is sampling, tell it to stop */
	ctrl_shm->state = RESET;
    }
    ctrl_usr->run_mode = STOP;
}

void prepare_scope_restart(void) {
    if(ctrl_usr->pending_restart) return;
    ctrl_shm->state = RESET;
    ctrl_usr->old_run_mode = ctrl_usr->run_mode;
    ctrl_usr->pending_restart = 1;
    ctrl_usr->run_mode = STOP;
}
