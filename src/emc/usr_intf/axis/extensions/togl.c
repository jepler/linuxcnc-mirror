
/*
 * Togl - a Tk OpenGL widget
 * Version 1.6
 * Copyright (C) 1996-2002  Brian Paul and Ben Bederson
 * See the LICENSE-Togl file for copyright details.
 */


/*
 * Currently we support X11, Win32 and Macintosh only
 */
#if !defined(WIN32) && !defined(macintosh)
#define X11
#endif

#include "config.h"

/*** Windows headers ***/
#if defined(WIN32) && !defined(X11) && !defined(macintosh)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#undef WIN32_LEAN_AND_MEAN
#include <winnt.h>

/*** X Window System headers ***/
#elif defined(X11) && !defined(WIN32) && !defined(macintosh)
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>  /* for XA_RGB_DEFAULT_MAP atom */
#if defined(__vms)
#include <X11/StdCmap.h>  /* for XmuLookupStandardColormap */
#else
#include <X11/Xmu/StdCmap.h>  /* for XmuLookupStandardColormap */
#endif
#include <GL/glx.h>

/*** Mac headers ***/
#elif defined(macintosh) && !defined(WIN32) && !defined(X11)
#include <Gestalt.h>
#include <Traps.h>
#include <agl.h>
#include <tclMacCommonPch.h>

#else                           /* make sure only one platform defined */
#error Unsupported platform, or confused platform defines...
#endif

/*** Standard C headers ***/
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <tcl.h>
#ifdef USE_LOCAL_TK_H
  #include "tk8.4a3.h"
#else
  #include <tk.h>
#endif

#ifdef WIN32
#  include <tkPlatDecls.h>
#endif

#if TK_MAJOR_VERSION<8
#  error Sorry Togl requires Tcl/Tk ver 8.0 or higher.
#endif

#if defined(macintosh)
#  if TK_MAJOR_VERSION<8 || TK_MINOR_VERSION<3
#    error Sorry Mac version requires Tcl/Tk ver 8.3.0 or higher.
#  endif
#endif /* Mac */

/* workaround for bug #123153 in tcl ver8.4a2 (tcl.h) */
#if defined(Tcl_InitHashTable) && defined(USE_TCL_STUBS)
#undef Tcl_InitHashTable
#define Tcl_InitHashTable (tclStubsPtr->tcl_InitHashTable)
#endif
#if (TK_MAJOR_VERSION>=8 && TK_MINOR_VERSION>=4)
#  define HAVE_TK_SETCLASSPROCS
#endif

/*
 * Copy of TkClassProcs declarations form tkInt.h
 * (this is needed for Tcl ver =< 8.4a3)
 */

typedef int (TkBindEvalProc) _ANSI_ARGS_((ClientData clientData,
	Tcl_Interp *interp, XEvent *eventPtr, Tk_Window tkwin,
	KeySym keySym));
typedef void (TkBindFreeProc) _ANSI_ARGS_((ClientData clientData));
typedef Window (TkClassCreateProc) _ANSI_ARGS_((Tk_Window tkwin,
	Window parent, ClientData instanceData));
typedef void (TkClassGeometryProc) _ANSI_ARGS_((ClientData instanceData));
typedef void (TkClassModalProc) _ANSI_ARGS_((Tk_Window tkwin,
	XEvent *eventPtr));
typedef struct TkClassProcs {
  TkClassCreateProc *createProc;
  TkClassGeometryProc *geometryProc;
  TkClassModalProc *modalProc;
} TkClassProcs;


#include "togl.h"


/* Defaults */
#define DEFAULT_WIDTH		"400"
#define DEFAULT_HEIGHT		"400"
#define DEFAULT_IDENT		""
#define DEFAULT_FONTNAME	"fixed"
#define DEFAULT_TIME		"1"


#ifdef WIN32
/* Maximum size of a logical palette corresponding to a colormap
 * in color index mode.
 */
#define MAX_CI_COLORMAP_SIZE 4096

/* 
 * copy of TkWinColormap from tkWinInt.h 
 */

typedef struct {
    HPALETTE palette;		/* Palette handle used when drawing. */
    UINT size;			/* Number of entries in the palette. */
    int stale;			/* 1 if palette needs to be realized,
				 * otherwise 0.  If the palette is stale,
				 * then an idle handler is scheduled to
				 * realize the palette. */
    Tcl_HashTable refCounts;	/* Hash table of palette entry reference counts
				 * indexed by pixel value. */
} TkWinColormap;

static LRESULT (CALLBACK *tkWinChildProc)(HWND hwnd, UINT message,
				 WPARAM wParam, LPARAM lParam) = NULL;

#define TK_WIN_CHILD_CLASS_NAME "TkChild"

#endif /* WIN32 */


#define MAX(a,b)	(((a)>(b))?(a):(b))

#define TCL_ERR(interp, string)			\
   do {						\
      Tcl_ResetResult(interp);			\
      Tcl_AppendResult(interp, string, NULL);	\
      return TCL_ERROR;				\
   } while (0)

/* The constant DUMMY_WINDOW is used to signal window creation 
   failure from the Togl_CreateWindow() */
#define DUMMY_WINDOW ((Window)-1)

#define ALL_EVENTS_MASK 	\
   (KeyPressMask |		\
    KeyReleaseMask |		\
    ButtonPressMask |		\
    ButtonReleaseMask |		\
    EnterWindowMask |		\
    LeaveWindowMask |		\
    PointerMotionMask |		\
    ExposureMask |		\
    VisibilityChangeMask |	\
    FocusChangeMask |		\
    PropertyChangeMask |	\
    ColormapChangeMask)


struct Togl
{
   struct Togl *Next;           /* next in linked list */

#if defined(WIN32)
   HDC tglGLHdc;                /* Device context of device that OpenGL calls will be drawn on */
   HGLRC tglGLHglrc;            /* OpenGL rendering context to be made current */
   int CiColormapSize;          /* (Maximum) size of colormap in color index mode */
#elif defined(X11)
   GLXContext GlCtx;		/* Normal planes GLX context */
#elif defined(macintosh)
   AGLContext aglCtx;
#endif /* WIN32 */

   Display *display;		/* X's token for the window's display. */
   Tk_Window  TkWin;		/* Tk window structure */
   Tcl_Interp *Interp;		/* Tcl interpreter */
   Tcl_Command widgetCmd;       /* Token for togl's widget command */
#ifndef NO_TK_CURSOR
   Tk_Cursor Cursor;		/* The widget's cursor */
#endif
   int Width, Height;		/* Dimensions of window */
   int TimerInterval;		/* Time interval for timer in milliseconds */
#if (TCL_MAJOR_VERSION * 100 + TCL_MINOR_VERSION) >= 705
   Tcl_TimerToken timerHandler; /* Token for togl's timer handler */
#else
   Tk_TimerToken timerHandler;  /* Token for togl's timer handler */
#endif
   int RgbaFlag;		/* configuration flags (ala GLX parameters) */
   int RgbaRed;
   int RgbaGreen;
   int RgbaBlue;
   int DoubleFlag;
   int DepthFlag;
   int DepthSize;
   int AccumFlag;
   int AccumRed;
   int AccumGreen;
   int AccumBlue;
   int AccumAlpha;
   int AlphaFlag;
   int AlphaSize;
   int StencilFlag;
   int StencilSize;
   int PrivateCmapFlag;
   int OverlayFlag;
   int StereoFlag;
   int AuxNumber;
   int Indirect;
   char *ShareList;             /* name (ident) of Togl to share dlists with */
   char *ShareContext;          /* name (ident) to share OpenGL context with */

   char *Ident;				/* User's identification string */
   ClientData Client_Data;		/* Pointer to user data */

   GLboolean UpdatePending;		/* Should normal planes be redrawn? */

   Togl_Callback *CreateProc;		/* Callback when widget is created */
   Togl_Callback *DisplayProc;		/* Callback when widget is rendered */
   Togl_Callback *ReshapeProc;		/* Callback when window size changes */
   Togl_Callback *DestroyProc;		/* Callback when widget is destroyed */
   Togl_Callback *TimerProc;		/* Callback when widget is idle */

   /* Overlay stuff */
#if defined(X11)
   GLXContext OverlayCtx;		/* Overlay planes OpenGL context */
#elif defined(WIN32)
   HGLRC tglGLOverlayHglrc;
#endif /* X11 */

   Window OverlayWindow;		/* The overlay window, or 0 */
   Togl_Callback *OverlayDisplayProc;	/* Overlay redraw proc */
   GLboolean OverlayUpdatePending;	/* Should overlay be redrawn? */
   Colormap OverlayCmap;		/* colormap for overlay is created */
   int OverlayTransparentPixel;		/* transparent pixel */
   int OverlayIsMapped;
};


/* NTNTNT need to change to handle Windows Data Types */
/*
 * Prototypes for functions local to this file
 */
static int Togl_Cmd(ClientData clientData, Tcl_Interp *interp,
                    int argc, char **argv);
static void Togl_EventProc(ClientData clientData, XEvent *eventPtr);
static Window Togl_CreateWindow(Tk_Window, Window, ClientData);
#ifdef MESA_COLOR_HACK
static int get_free_color_cells( Display *display, int screen,
                                 Colormap colormap);
static void free_default_color_cells( Display *display, Colormap colormap);
#endif
static void ToglCmdDeletedProc( ClientData );



#if defined(__sgi) && defined(STEREO)
/* SGI-only stereo */
static void stereoMakeCurrent( Display *dpy, Window win, GLXContext ctx );
static void stereoInit( struct Togl *togl,int stereoEnabled );
#endif

#ifdef macintosh
static void SetMacBufRect(struct Togl *togl);
#endif


/*
 * Setup Togl widget configuration options:
 */

static Tk_ConfigSpec configSpecs[] = {
    {TK_CONFIG_PIXELS, "-height", "height", "Height",
     DEFAULT_HEIGHT, Tk_Offset(struct Togl, Height), 0, NULL},

    {TK_CONFIG_PIXELS, "-width", "width", "Width",
     DEFAULT_WIDTH, Tk_Offset(struct Togl, Width), 0, NULL},

    {TK_CONFIG_BOOLEAN, "-rgba", "rgba", "Rgba",
     "true", Tk_Offset(struct Togl, RgbaFlag), 0, NULL},

    {TK_CONFIG_INT, "-redsize", "redsize", "RedSize",
     "1", Tk_Offset(struct Togl, RgbaRed), 0, NULL},

    {TK_CONFIG_INT, "-greensize", "greensize", "GreenSize",
     "1", Tk_Offset(struct Togl, RgbaGreen), 0, NULL},

    {TK_CONFIG_INT, "-bluesize", "bluesize", "BlueSize",
     "1", Tk_Offset(struct Togl, RgbaBlue), 0, NULL},

    {TK_CONFIG_BOOLEAN, "-double", "double", "Double",
     "false", Tk_Offset(struct Togl, DoubleFlag), 0, NULL},

    {TK_CONFIG_BOOLEAN, "-depth", "depth", "Depth",
     "false", Tk_Offset(struct Togl, DepthFlag), 0, NULL},

    {TK_CONFIG_INT, "-depthsize", "depthsize", "DepthSize",
     "1", Tk_Offset(struct Togl, DepthSize), 0, NULL},

    {TK_CONFIG_BOOLEAN, "-accum", "accum", "Accum",
     "false", Tk_Offset(struct Togl, AccumFlag), 0, NULL},

    {TK_CONFIG_INT, "-accumredsize", "accumredsize", "AccumRedSize",
     "1", Tk_Offset(struct Togl, AccumRed), 0, NULL},

    {TK_CONFIG_INT, "-accumgreensize", "accumgreensize", "AccumGreenSize",
     "1", Tk_Offset(struct Togl, AccumGreen), 0, NULL},

    {TK_CONFIG_INT, "-accumbluesize", "accumbluesize", "AccumBlueSize",
     "1", Tk_Offset(struct Togl, AccumBlue), 0, NULL},

    {TK_CONFIG_INT, "-accumalphasize", "accumalphasize", "AccumAlphaSize",
     "1", Tk_Offset(struct Togl, AccumAlpha), 0, NULL},

    {TK_CONFIG_BOOLEAN, "-alpha", "alpha", "Alpha",
     "false", Tk_Offset(struct Togl, AlphaFlag), 0, NULL},

    {TK_CONFIG_INT, "-alphasize", "alphasize", "AlphaSize",
     "1", Tk_Offset(struct Togl, AlphaSize), 0, NULL},

    {TK_CONFIG_BOOLEAN, "-stencil", "stencil", "Stencil",
     "false", Tk_Offset(struct Togl, StencilFlag), 0, NULL},

    {TK_CONFIG_INT, "-stencilsize", "stencilsize", "StencilSize",
     "1", Tk_Offset(struct Togl, StencilSize), 0, NULL},

    {TK_CONFIG_INT, "-auxbuffers", "auxbuffers", "AuxBuffers",
     "0", Tk_Offset(struct Togl, AuxNumber), 0, NULL},

    {TK_CONFIG_BOOLEAN, "-privatecmap", "privateCmap", "PrivateCmap",
     "false", Tk_Offset(struct Togl, PrivateCmapFlag), 0, NULL},

    {TK_CONFIG_BOOLEAN, "-overlay", "overlay", "Overlay",
     "false", Tk_Offset(struct Togl, OverlayFlag), 0, NULL},

    {TK_CONFIG_BOOLEAN, "-stereo", "stereo", "Stereo",
     "false", Tk_Offset(struct Togl, StereoFlag), 0, NULL},

#ifndef NO_TK_CURSOR
    { TK_CONFIG_ACTIVE_CURSOR, "-cursor", "cursor", "Cursor",
     "", Tk_Offset(struct Togl, Cursor), TK_CONFIG_NULL_OK, NULL },
#endif

    {TK_CONFIG_INT, "-time", "time", "Time",
     DEFAULT_TIME, Tk_Offset(struct Togl, TimerInterval), 0, NULL},

    {TK_CONFIG_STRING, "-sharelist", "sharelist", "ShareList",
     NULL, Tk_Offset(struct Togl, ShareList), 0, NULL},

    {TK_CONFIG_STRING, "-sharecontext", "sharecontext", "ShareContext",
     NULL, Tk_Offset(struct Togl, ShareContext), 0, NULL},

    {TK_CONFIG_STRING, "-ident", "ident", "Ident",
     DEFAULT_IDENT, Tk_Offset(struct Togl, Ident), 0, NULL},

    {TK_CONFIG_BOOLEAN, "-indirect", "indirect", "Indirect",
     "false", Tk_Offset(struct Togl, Indirect), 0, NULL},

    {TK_CONFIG_END, (char *) NULL, (char *) NULL, (char *) NULL,
     (char *) NULL, 0, 0, NULL}
};


/*
 * Default callback pointers.  When a new Togl widget is created it
 * will be assigned these initial callbacks.
 */
static Togl_Callback *DefaultCreateProc = NULL;
static Togl_Callback *DefaultDisplayProc = NULL;
static Togl_Callback *DefaultReshapeProc = NULL;
static Togl_Callback *DefaultDestroyProc = NULL;
static Togl_Callback *DefaultOverlayDisplayProc = NULL;
static Togl_Callback *DefaultTimerProc = NULL;
static ClientData DefaultClientData = NULL;
static Tcl_HashTable CommandTable;

/*
 * Head of linked list of all Togl widgets
 */
static struct Togl *ToglHead = NULL;

/*
 * Add given togl widget to linked list.
 */
static void AddToList(struct Togl *t)
{
   t->Next = ToglHead;
   ToglHead = t;
}

/*
 * Remove given togl widget from linked list.
 */
static void RemoveFromList(struct Togl *t)
{
   struct Togl *prev = NULL;
   struct Togl *pos = ToglHead;
   while (pos) {
      if (pos == t) {
         if (prev) {
            prev->Next = pos->Next;
         }
         else {
            ToglHead = pos->Next;
         }
         return;
      }
      prev = pos;
      pos = pos->Next;
   }
}

/*
 * Return pointer to togl widget given a user identifier string.
 */
static struct Togl *FindTogl(const char *ident)
{
   struct Togl *t = ToglHead;
   while (t) {
      if (strcmp(t->Ident, ident) == 0)
         return t;
      t = t->Next;
   }
   return NULL;
}




#if defined(X11)
/*
 * Return an X colormap to use for OpenGL RGB-mode rendering.
 * Input:  dpy - the X display
 *         scrnum - the X screen number
 *         visinfo - the XVisualInfo as returned by glXChooseVisual()
 * Return:  an X Colormap or 0 if there's a _serious_ error.
 */
static Colormap
get_rgb_colormap( Display *dpy,
                  int scrnum,
                  const XVisualInfo *visinfo,
                  Tk_Window tkwin)
{
   Atom hp_cr_maps;
   Status status;
   int numCmaps;
   int i;
   XStandardColormap *standardCmaps;
   Window root = XRootWindow(dpy,scrnum);
   int using_mesa;

   /*
    * First check if visinfo's visual matches the default/root visual.
    */
   if (visinfo->visual==Tk_Visual(tkwin)) {
      /* use the default/root colormap */
      Colormap cmap;
      cmap = Tk_Colormap(tkwin);
#ifdef MESA_COLOR_HACK
      (void) get_free_color_cells( dpy, scrnum, cmap);
#endif
      return cmap;
   }

   /*
    * Check if we're using Mesa.
    */
   if (strstr(glXQueryServerString( dpy, scrnum, GLX_VERSION ), "Mesa")) {
      using_mesa = 1;
   }
   else {
      using_mesa = 0;
   }

   /*
    * Next, if we're using Mesa and displaying on an HP with the "Color
    * Recovery" feature and the visual is 8-bit TrueColor, search for a
    * special colormap initialized for dithering.  Mesa will know how to
    * dither using this colormap.
    */
   if (using_mesa) {
      hp_cr_maps = XInternAtom( dpy, "_HP_RGB_SMOOTH_MAP_LIST", True );
      if (hp_cr_maps
#ifdef __cplusplus
	  && visinfo->visual->c_class==TrueColor
#else
	  && visinfo->visual->class==TrueColor
#endif
	  && visinfo->depth==8) {
	 status = XGetRGBColormaps( dpy, root, &standardCmaps,
				    &numCmaps, hp_cr_maps );
	 if (status) {
	    for (i=0; i<numCmaps; i++) {
	       if (standardCmaps[i].visualid == visinfo->visual->visualid) {
                  Colormap cmap = standardCmaps[i].colormap;
                  XFree( standardCmaps );
		  return cmap;
	       }
	    }
            XFree(standardCmaps);
	 }
      }
   }

   /*
    * Next, try to find a standard X colormap.
    */
#if !HP && !SUN
#ifndef SOLARIS_BUG
   status = XmuLookupStandardColormap( dpy, visinfo->screen,
				       visinfo->visualid, visinfo->depth,
				       XA_RGB_DEFAULT_MAP,
				       /* replace */ False, /* retain */ True);
   if (status == 1) {
      status = XGetRGBColormaps( dpy, root, &standardCmaps,
				 &numCmaps, XA_RGB_DEFAULT_MAP);
      if (status == 1) {
         for (i = 0; i < numCmaps; i++) {
	    if (standardCmaps[i].visualid == visinfo->visualid) {
               Colormap cmap = standardCmaps[i].colormap;
	       XFree(standardCmaps);
	       return cmap;
	    }
	 }
         XFree(standardCmaps);
      }
   }
#endif
#endif

   /*
    * If we get here, give up and just allocate a new colormap.
    */
   return XCreateColormap( dpy, root, visinfo->visual, AllocNone );
}
#elif defined(WIN32)

/* Code to create RGB palette is taken from the GENGL sample program
   of Win32 SDK */

static unsigned char threeto8[8] = {
    0, 0111>>1, 0222>>1, 0333>>1, 0444>>1, 0555>>1, 0666>>1, 0377
};

static unsigned char twoto8[4] = {
    0, 0x55, 0xaa, 0xff
};

static unsigned char oneto8[2] = {
    0, 255
};

static int defaultOverride[13] = {
    0, 3, 24, 27, 64, 67, 88, 173, 181, 236, 247, 164, 91
};

static PALETTEENTRY defaultPalEntry[20] = {
    { 0,   0,   0,    0 },
    { 0x80,0,   0,    0 },
    { 0,   0x80,0,    0 },
    { 0x80,0x80,0,    0 },
    { 0,   0,   0x80, 0 },
    { 0x80,0,   0x80, 0 },
    { 0,   0x80,0x80, 0 },
    { 0xC0,0xC0,0xC0, 0 },

    { 192, 220, 192,  0 },
    { 166, 202, 240,  0 },
    { 255, 251, 240,  0 },
    { 160, 160, 164,  0 },

    { 0x80,0x80,0x80, 0 },
    { 0xFF,0,   0,    0 },
    { 0,   0xFF,0,    0 },
    { 0xFF,0xFF,0,    0 },
    { 0,   0,   0xFF, 0 },
    { 0xFF,0,   0xFF, 0 },
    { 0,   0xFF,0xFF, 0 },
    { 0xFF,0xFF,0xFF, 0 }
};

static unsigned char
ComponentFromIndex(int i, UINT nbits, UINT shift)
{
    unsigned char val;

    val = (unsigned char) (i >> shift);
    switch (nbits) {

    case 1:
        val &= 0x1;
        return oneto8[val];

    case 2:
        val &= 0x3;
        return twoto8[val];

    case 3:
        val &= 0x7;
        return threeto8[val];

    default:
        return 0;
    }
}

static Colormap Win32CreateRgbColormap(PIXELFORMATDESCRIPTOR pfd)
{
    TkWinColormap *cmap = (TkWinColormap *) ckalloc(sizeof(TkWinColormap));
    LOGPALETTE *pPal;
    int n, i;

    n = 1 << pfd.cColorBits;
    pPal = (PLOGPALETTE)LocalAlloc(LMEM_FIXED, sizeof(LOGPALETTE) +
            n * sizeof(PALETTEENTRY));
    pPal->palVersion = 0x300;
    pPal->palNumEntries = n;
    for (i=0; i<n; i++) {
        pPal->palPalEntry[i].peRed =
                ComponentFromIndex(i, pfd.cRedBits, pfd.cRedShift);
        pPal->palPalEntry[i].peGreen =
                ComponentFromIndex(i, pfd.cGreenBits, pfd.cGreenShift);
        pPal->palPalEntry[i].peBlue =
                ComponentFromIndex(i, pfd.cBlueBits, pfd.cBlueShift);
        pPal->palPalEntry[i].peFlags = 0;
    }

    /* fix up the palette to include the default GDI palette */
    if ((pfd.cColorBits == 8)                           &&
        (pfd.cRedBits   == 3) && (pfd.cRedShift   == 0) &&
        (pfd.cGreenBits == 3) && (pfd.cGreenShift == 3) &&
        (pfd.cBlueBits  == 2) && (pfd.cBlueShift  == 6)
       ) {
        for (i = 1 ; i <= 12 ; i++)
            pPal->palPalEntry[defaultOverride[i]] = defaultPalEntry[i];
    }

    cmap->palette = CreatePalette(pPal);
    LocalFree(pPal);
    cmap->size = n;
    cmap->stale = 0;

    /* Since this is a private colormap of a fix size, we do not need
       a valid hash table, but a dummy one */

    Tcl_InitHashTable(&cmap->refCounts, TCL_ONE_WORD_KEYS);
    return (Colormap)cmap;
}

static Colormap Win32CreateCiColormap(struct Togl *togl)
{
    /* Create a colormap with size of togl->CiColormapSize and set all
       entries to black */

    LOGPALETTE logPalette;
    TkWinColormap *cmap = (TkWinColormap *) ckalloc(sizeof(TkWinColormap));

    logPalette.palVersion = 0x300;
    logPalette.palNumEntries = 1;
    logPalette.palPalEntry[0].peRed = 0;
    logPalette.palPalEntry[0].peGreen = 0;
    logPalette.palPalEntry[0].peBlue = 0;
    logPalette.palPalEntry[0].peFlags = 0;

    cmap->palette = CreatePalette(&logPalette);
    cmap->size = togl->CiColormapSize;
    ResizePalette(cmap->palette, cmap->size);  /* sets new entries to black */
    cmap->stale = 0;

    /* Since this is a private colormap of a fix size, we do not need
       a valid hash table, but a dummy one */

    Tcl_InitHashTable(&cmap->refCounts, TCL_ONE_WORD_KEYS);
    return (Colormap)cmap;
}
#endif /*X11*/



/*
 * Togl_Init
 *
 *   Called upon system startup to create Togl command.
 */
int Togl_Init(Tcl_Interp *interp)
{
   int major,minor,patchLevel,releaseType;

#ifdef USE_TCL_STUBS
   if (Tcl_InitStubs(interp, "8.1", 0) == NULL) {return TCL_ERROR;}
#endif
#ifdef USE_TK_STUBS
   if (Tk_InitStubs(interp, "8.1", 0) == NULL) {return TCL_ERROR;}
#endif

   /* Skip all this on Tcl/Tk 8.0 or older.  Seems to work */
#if TCL_MAJOR_VERSION * 100 + TCL_MINOR_VERSION > 800
   Tcl_GetVersion(&major,&minor,&patchLevel,&releaseType);

#ifndef HAVE_TK_SETCLASSPROCS
   if (major >= 8 && minor >= 4) {
     TCL_ERR(interp,"Sorry, this instance of Togl was not compiled to work with Tcl/Tk 8.4 or higher.");
   }
#endif

#endif

   if (Tcl_PkgProvide(interp, "Togl", TOGL_VERSION) != TCL_OK) {
      return TCL_ERROR;
   }

   Tcl_CreateCommand(interp, "togl", (Tcl_CmdProc *)Togl_Cmd,
                     (ClientData) Tk_MainWindow(interp), NULL);

   Tcl_InitHashTable(&CommandTable, TCL_STRING_KEYS);

   return TCL_OK;
}


/*
 * Register a C function to be called when an Togl widget is realized.
 */
void Togl_CreateFunc( Togl_Callback *proc )
{
   DefaultCreateProc = proc;
}


/*
 * Register a C function to be called when an Togl widget must be redrawn.
 */
void Togl_DisplayFunc( Togl_Callback *proc )
{
   DefaultDisplayProc = proc;
}


/*
 * Register a C function to be called when an Togl widget is resized.
 */
void Togl_ReshapeFunc( Togl_Callback *proc )
{
   DefaultReshapeProc = proc;
}


/*
 * Register a C function to be called when an Togl widget is destroyed.
 */
void Togl_DestroyFunc( Togl_Callback *proc )
{
   DefaultDestroyProc = proc;
}


/*
 * Register a C function to be called from TimerEventHandler.
 */
void Togl_TimerFunc( Togl_Callback *proc )
{
   DefaultTimerProc = proc;
}


/*
 * Reset default callback pointers to NULL.
 */
void Togl_ResetDefaultCallbacks( void )
{
   DefaultCreateProc = NULL;
   DefaultDisplayProc = NULL;
   DefaultReshapeProc = NULL;
   DefaultDestroyProc = NULL;
   DefaultOverlayDisplayProc = NULL;
   DefaultTimerProc = NULL;
   DefaultClientData = NULL;
}


/*
 * Change the create callback for a specific Togl widget.
 */
void Togl_SetCreateFunc( struct Togl *togl, Togl_Callback *proc )
{
   togl->CreateProc = proc;
}


/*
 * Change the display/redraw callback for a specific Togl widget.
 */
void Togl_SetDisplayFunc( struct Togl *togl, Togl_Callback *proc )
{
   togl->DisplayProc = proc;
}


/*
 * Change the reshape callback for a specific Togl widget.
 */
void Togl_SetReshapeFunc( struct Togl *togl, Togl_Callback *proc )
{
   togl->ReshapeProc = proc;
}


/*
 * Change the destroy callback for a specific Togl widget.
 */
void Togl_SetDestroyFunc( struct Togl *togl, Togl_Callback *proc )
{
   togl->DestroyProc = proc;
}


/*
 * Togl_Timer
 *
 * Gets called from Tk_CreateTimerHandler.
 */
static void Togl_Timer( ClientData clientData )
{
   struct Togl *togl = (struct Togl *) clientData;
   if (togl->TimerProc) {
      togl->TimerProc(togl);

      /* Re-register this callback since Tcl/Tk timers are "one-shot".
       * That is, after the timer callback is called it not normally
       * called again.  That's not the behavior we want for Togl.
       */
#if (TK_MAJOR_VERSION * 100 + TK_MINOR_VERSION) >= 401
      togl->timerHandler =
         Tcl_CreateTimerHandler( togl->TimerInterval, Togl_Timer, (ClientData)togl );
#else
      togl->timerHandler =
         Tk_CreateTimerHandler( togl->TimeInterval, Togl_Timer, (ClientData)togl );
#endif
   }
}


/*
 * Change the timer callback for a specific Togl widget.
 * Pass NULL to disable the callback.
 */
void Togl_SetTimerFunc( struct Togl *togl, Togl_Callback *proc )
{
   togl->TimerProc = proc;
   if (proc) {
#if (TK_MAJOR_VERSION * 100 + TK_MINOR_VERSION) >= 401
      togl->timerHandler =
         Tcl_CreateTimerHandler( togl->TimerInterval, Togl_Timer, (ClientData)togl );
#else
      togl->timerHandler =
         Tk_CreateTimerHandler( togl->TimeInterval, Togl_Timer, (ClientData)togl );
#endif
   }
}



/*
 * Togl_CreateCommand
 *
 *   Declares a new C sub-command of Togl callable from Tcl.
 *   Every time the sub-command is called from Tcl, the
 *   C routine will be called with all the arguments from Tcl.
 */
void Togl_CreateCommand( char *cmd_name, Togl_CmdProc *cmd_proc)
{
   int new_item;
   Tcl_HashEntry *entry;
   entry = Tcl_CreateHashEntry(&CommandTable, cmd_name, &new_item);
   Tcl_SetHashValue(entry, cmd_proc);
}


/*
 * Togl_MakeCurrent
 *
 *   Bind the OpenGL rendering context to the specified
 *   Togl widget.
 */
void Togl_MakeCurrent( const struct Togl *togl )
{
#if defined(WIN32)
   int res = wglMakeCurrent(togl->tglGLHdc, togl->tglGLHglrc);
   assert(res == TRUE);

#elif defined(X11)
   glXMakeCurrent( Tk_Display(togl->TkWin),
                   Tk_WindowId(togl->TkWin),
                   togl->GlCtx );
#if defined(__sgi) && defined(STEREO)
   stereoMakeCurrent( Tk_Display(togl->TkWin),
                      Tk_WindowId(togl->TkWin),
                      togl->GlCtx );
#endif /*__sgi STEREO */

#elif defined(macintosh)
   aglSetCurrentContext(togl->aglCtx);
#endif /* WIN32 */
}


#ifdef macintosh
/* tell OpenGL which part of the Mac window to render to */
static void SetMacBufRect(struct Togl *togl)
{
   GLint wrect[4];

   /* set wrect[0,1] to lower left corner of widget */
   wrect[2] = ((TkWindow *) (togl->TkWin))->changes.width;
   wrect[3] = ((TkWindow *) (togl->TkWin))->changes.height;
   wrect[0] = ((TkWindow *) (togl->TkWin))->privatePtr->xOff;
   wrect[1] =
       ((TkWindow *) (togl->TkWin))->privatePtr->toplevel->portPtr->portRect.bottom -
       wrect[3] - ((TkWindow *) (togl->TkWin))->privatePtr->yOff;
   aglSetInteger(togl->aglCtx, AGL_BUFFER_RECT, wrect);
   aglEnable(togl->aglCtx, AGL_BUFFER_RECT);
   aglUpdateContext(togl->aglCtx);
}
#endif

/*
 * Called when the widget's contents must be redrawn.  Basically, we
 * just call the user's render callback function.
 *
 * Note that the parameter type is ClientData so this function can be
 * passed to Tk_DoWhenIdle().
 */
static void Togl_Render( ClientData clientData )
{
   struct Togl *togl = (struct Togl *)clientData;

   if (togl->DisplayProc) {

#ifdef macintosh
      /* Mac is complicated here because OpenGL needs to know what part of the parent
         window to render into, and it seems that region need to be invalidated before
         drawing, so that QuickDraw will allow OpenGL to transfer pixels into that
         part of the window. I'm not even totally sure how or why this works as it
         does, since this aspect of Mac OpenGL seems to be totally undocumented.
         This was put together by trial and error! (thiessen) */
      MacRegion r;
      RgnPtr rp = &r;
      GrafPtr curPort, parentWin;
      parentWin = (GrafPtr)
         (((MacDrawable *) (Tk_WindowId(togl->TkWin)))->toplevel->portPtr);
      if (!parentWin) return;
#endif

      Togl_MakeCurrent(togl);

#ifdef macintosh
      /* Set QuickDraw port and clipping region */
      GetPort(&curPort);
      SetPort(parentWin);
      r.rgnBBox.left = ((TkWindow *) (togl->TkWin))->privatePtr->xOff;
      r.rgnBBox.right =
          r.rgnBBox.left + ((TkWindow *) (togl->TkWin))->changes.width - 1;
      r.rgnBBox.top = ((TkWindow *) (togl->TkWin))->privatePtr->yOff;
      r.rgnBBox.bottom =
          r.rgnBBox.top + ((TkWindow *) (togl->TkWin))->changes.height - 1;
      r.rgnSize = sizeof(Region);
      InvalRgn(&rp);
      SetClip(&rp);
      /* this may seem an odd place to put this, with possibly redundant calls to
         aglSetInteger(AGL_BUFFER_RECT...), but for some reason performance is actually
         a lot better if this is called before every render... */
      SetMacBufRect(togl);
#endif

      togl->DisplayProc(togl);

#ifdef macintosh
      SetPort(curPort);         /* restore previous port */
#endif

   }
   togl->UpdatePending = GL_FALSE;
}


static void RenderOverlay( ClientData clientData )
{
   struct Togl *togl = (struct Togl *)clientData;

   if (togl->OverlayFlag && togl->OverlayDisplayProc) {

#if defined(WIN32)
      int res = wglMakeCurrent(togl->tglGLHdc, togl->tglGLHglrc);
      assert(res == TRUE);

#elif defined(X11)
      glXMakeCurrent( Tk_Display(togl->TkWin),
		      togl->OverlayWindow,
		      togl->OverlayCtx );
#if defined(__sgi) && defined(STEREO)
      stereoMakeCurrent( Tk_Display(togl->TkWin),
                         togl->OverlayWindow,
                         togl->OverlayCtx );
#endif /*__sgi STEREO */

#endif /* WIN32 */

      togl->OverlayDisplayProc(togl);
   }
   togl->OverlayUpdatePending = GL_FALSE;
}


/*
 * It's possible to change with this function or in a script some
 * options like RGBA - ColorIndex ; Z-buffer and so on
 */
int Togl_Configure(Tcl_Interp *interp, struct Togl *togl,
                   int argc, char *argv[], int flags)
{
   int oldRgbaFlag    = togl->RgbaFlag;
   int oldRgbaRed     = togl->RgbaRed;
   int oldRgbaGreen   = togl->RgbaGreen;
   int oldRgbaBlue    = togl->RgbaBlue;
   int oldDoubleFlag  = togl->DoubleFlag;
   int oldDepthFlag   = togl->DepthFlag;
   int oldDepthSize   = togl->DepthSize;
   int oldAccumFlag   = togl->AccumFlag;
   int oldAccumRed    = togl->AccumRed;
   int oldAccumGreen  = togl->AccumGreen;
   int oldAccumBlue   = togl->AccumBlue;
   int oldAccumAlpha  = togl->AccumAlpha;
   int oldAlphaFlag   = togl->AlphaFlag;
   int oldAlphaSize   = togl->AlphaSize;
   int oldStencilFlag = togl->StencilFlag;
   int oldStencilSize = togl->StencilSize;
   int oldAuxNumber   = togl->AuxNumber;

#ifndef CONST84
#define CONST84
#endif
   if (Tk_ConfigureWidget(interp, togl->TkWin, configSpecs,
                          argc, (CONST84 char**)argv, (char *)togl, flags) == TCL_ERROR) {
      return(TCL_ERROR);
   }
#ifndef USE_OVERLAY
   if (togl->OverlayFlag) {
     TCL_ERR(interp,"Sorry, overlay was disabled");
   }
#endif
   

   Tk_GeometryRequest(togl->TkWin, togl->Width, togl->Height);
   /* this added per Lou Arata <arata@enya.picker.com> */
   Tk_ResizeWindow(togl->TkWin, togl->Width, togl->Height);

   if (togl->ReshapeProc &&
#if defined(WIN32)
       togl->tglGLHglrc
#elif defined(X11)
       togl->GlCtx
#elif defined(macintosh)
       togl->aglCtx
#endif
       ) {
      Togl_MakeCurrent(togl);
      togl->ReshapeProc(togl);
   }

   if (togl->RgbaFlag != oldRgbaFlag
       || togl->RgbaRed != oldRgbaRed
       || togl->RgbaGreen != oldRgbaGreen
       || togl->RgbaBlue != oldRgbaBlue
       || togl->DoubleFlag != oldDoubleFlag
       || togl->DepthFlag != oldDepthFlag
       || togl->DepthSize != oldDepthSize
       || togl->AccumFlag != oldAccumFlag
       || togl->AccumRed != oldAccumRed
       || togl->AccumGreen != oldAccumGreen
       || togl->AccumBlue != oldAccumBlue
       || togl->AccumAlpha != oldAccumAlpha
       || togl->AlphaFlag != oldAlphaFlag
       || togl->AlphaSize != oldAlphaSize
       || togl->StencilFlag != oldStencilFlag
       || togl->StencilSize != oldStencilSize
       || togl->AuxNumber != oldAuxNumber) {
#ifdef MESA_COLOR_HACK
      free_default_color_cells( Tk_Display(togl->TkWin),
                                Tk_Colormap(togl->TkWin) );
#endif
   }

#if defined(__sgi) && defined(STEREO)
   stereoInit(togl,togl->StereoFlag);
#endif

   return TCL_OK;
}


int Togl_Widget(ClientData clientData, Tcl_Interp *interp,
	       int argc, char *argv[])
{
   struct Togl *togl = (struct Togl *)clientData;
   int result = TCL_OK;
   Tcl_HashEntry *entry;
   Tcl_HashSearch search;
   Togl_CmdProc *cmd_proc;

   if (argc < 2) {
      Tcl_AppendResult(interp, "wrong # args: should be \"",
                       argv[0], " ?options?\"", NULL);
      return TCL_ERROR;
   }

   Tk_Preserve((ClientData)togl);

   if (!strncmp(argv[1], "configure", MAX(1, strlen(argv[1])))) {
      if (argc == 2) {
         /* Return list of all configuration parameters */
         result = Tk_ConfigureInfo(interp, togl->TkWin, configSpecs,
                                   (char *)togl, (char *)NULL, 0);
      }
      else if (argc == 3) {
         if (strcmp(argv[2],"-extensions")==0) {
            /* Return a list of OpenGL extensions available */
            char *extensions;
            extensions = (char *) glGetString(GL_EXTENSIONS);
            Tcl_SetResult( interp, extensions, TCL_STATIC );
            result = TCL_OK;
         }
         else {
            /* Return a specific configuration parameter */
            result = Tk_ConfigureInfo(interp, togl->TkWin, configSpecs,
                                      (char *)togl, argv[2], 0);
         }
      }
      else {
         /* Execute a configuration change */
         result = Togl_Configure(interp, togl, argc-2, argv+2,
                                TK_CONFIG_ARGV_ONLY);
      }
   }
   else if (!strncmp(argv[1], "render", MAX(1, strlen(argv[1])))) {
      /* force the widget to be redrawn */
      Togl_Render((ClientData) togl);
   }
   else if (!strncmp(argv[1], "swapbuffers", MAX(1, strlen(argv[1])))) {
      /* force the widget to be redrawn */
      Togl_SwapBuffers(togl);
   }
   else if (!strncmp(argv[1], "makecurrent", MAX(1, strlen(argv[1])))) {
      /* force the widget to be redrawn */
      Togl_MakeCurrent(togl);
   }
   else if (!strncmp(argv[1], "loadbitmapfont", MAX(1, strlen(argv[1])))){
      if (argc == 3){
         GLuint fontbase;
         Tcl_Obj * fontbaseAsTclObject;
         fontbase = Togl_LoadBitmapFont(togl,argv[2]);
         if (fontbase) {
            fontbaseAsTclObject = Tcl_NewIntObj(fontbase);
            Tcl_SetObjResult(interp, fontbaseAsTclObject);
            result = TCL_OK;
         }
         else {
            Tcl_AppendResult(interp, "Could not allocate font",NULL);
            result = TCL_ERROR;
         }
      }
      else {
         Tcl_AppendResult(interp, "wrong # args",NULL);
         result=TCL_ERROR;
      }
   }
   else if (!strncmp(argv[1], "measurebitmapfont", MAX(1, strlen(argv[1])))) {
      if (argc != 3) {
         Tcl_AppendResult(interp, "wrong # args",NULL);
         result=TCL_ERROR;
      } else {
         int width, linespace;
         if(!Togl_BitmapFontMetrics(togl, argv[2], &width, &linespace))
         {
             Tcl_AppendResult(interp, "Could not allocate font",NULL);
             result=TCL_ERROR;
         } else {
             char buf[32];
             snprintf(buf, sizeof(buf), "%d %d", width, linespace);
             Tcl_AppendResult(interp, buf, NULL);
             result=TCL_OK;
         }
      }
   }
   else if (!strncmp(argv[1], "unloadbitmapfont", MAX(1, strlen(argv[1])))) {
      if (argc == 3) {
         Togl_UnloadBitmapFont(togl, atoi(argv[2]));
         result = TCL_OK;
      }
      else {
         Tcl_AppendResult(interp, "wrong # args",NULL);
         result = TCL_ERROR;
      }
   }
   else {
      /* Probably a user-defined function */
      entry = Tcl_FindHashEntry(&CommandTable, argv[1]);
      if (entry != NULL) {
         cmd_proc = (Togl_CmdProc *)Tcl_GetHashValue(entry);
         result = cmd_proc(togl, argc, argv);
      }
      else {
         Tcl_AppendResult(interp, "Togl: Unknown option: ", argv[1], "\n",
                          "Try: configure or render\n",
                          "or one of the user-defined commands:\n",
                          NULL);
         entry = Tcl_FirstHashEntry(&CommandTable, &search);
         while (entry) {
            Tcl_AppendResult(interp, "  ",
                             Tcl_GetHashKey(&CommandTable, entry),
                             "\n", NULL);
            entry = Tcl_NextHashEntry(&search);
         }
         result = TCL_ERROR;
      }
   }

   Tk_Release((ClientData)togl);
   return result;
}



/*
 * Togl_Cmd
 *
 *   Called when Togl is executed - creation of a Togl widget.
 *     * Creates a new window
 *     * Creates an 'Togl' data structure
 *     * Creates an event handler for this window
 *     * Creates a command that handles this object
 *     * Configures this Togl for the given arguments
 */
static int Togl_Cmd(ClientData clientData, Tcl_Interp *interp,
                    int argc, char **argv)
{
   char *name;
   Tk_Window mainwindow = (Tk_Window)clientData;
   Tk_Window tkwin;
   struct Togl *togl;

   if (argc <= 1) {
      TCL_ERR(interp, "wrong # args: should be \"pathName read filename\"");
   }

   /* Create the window. */
   name = argv[1];
   tkwin = Tk_CreateWindowFromPath(interp, mainwindow, name, (char *) NULL);
   if (tkwin == NULL) {
      return TCL_ERROR;
   }

   Tk_SetClass(tkwin, "Togl");

   /* Create Togl data structure */
   togl = (struct Togl *)malloc(sizeof(struct Togl));
   if (!togl) {
      return TCL_ERROR;
   }

   togl->Next = NULL;
#if defined(WIN32)
   togl->tglGLHdc = NULL;
   togl->tglGLHglrc = NULL;
#elif defined(X11)
   togl->GlCtx = NULL;
   togl->OverlayCtx = NULL;
#elif defined(macintosh)
   togl->aglCtx = NULL;
#endif /* WIN32 */
   togl->display = Tk_Display( tkwin );
   togl->TkWin = tkwin;
   togl->Interp = interp;
#ifndef NO_TK_CURSOR
   togl->Cursor = None;
#endif
   togl->Width = 0;
   togl->Height = 0;
   togl->TimerInterval = 0;
   togl->RgbaFlag = 1;
   togl->RgbaRed = 1;
   togl->RgbaGreen = 1;
   togl->RgbaBlue = 1;
   togl->DoubleFlag = 0;
   togl->DepthFlag = 0;
   togl->DepthSize = 1;
   togl->AccumFlag = 0;
   togl->AccumRed = 1;
   togl->AccumGreen = 1;
   togl->AccumBlue = 1;
   togl->AccumAlpha = 1;
   togl->AlphaFlag = 0;
   togl->AlphaSize = 1;
   togl->StencilFlag = 0;
   togl->StencilSize = 1;
   togl->OverlayFlag = 0;
   togl->StereoFlag = 0;
   togl->AuxNumber = 0;
   togl->Indirect = GL_FALSE;
   togl->UpdatePending = GL_FALSE;
   togl->OverlayUpdatePending = GL_FALSE;
   togl->CreateProc = DefaultCreateProc;
   togl->DisplayProc = DefaultDisplayProc;
   togl->ReshapeProc = DefaultReshapeProc;
   togl->DestroyProc = DefaultDestroyProc;
   togl->TimerProc = DefaultTimerProc;
   togl->OverlayDisplayProc = DefaultOverlayDisplayProc;
   togl->ShareList = NULL;
   togl->ShareContext = NULL;
   togl->Ident = NULL;
   togl->Client_Data = DefaultClientData;

   /* Create command event handler */
   togl->widgetCmd = Tcl_CreateCommand(interp, Tk_PathName(tkwin),
				       (Tcl_CmdProc *)      Togl_Widget, 
				       (ClientData)         togl,
				       (Tcl_CmdDeleteProc*) ToglCmdDeletedProc);
   /*
     Setup the Tk_ClassProcs callbacks to point at our 
     own window creation function

     We need to check at runtime if we should use the new 
     Tk_SetClassProcs() API or if we need to modify the window 
     structure directly
   */


#ifdef HAVE_TK_SETCLASSPROCS
   {        /* use public API (Tk 8.4+) */
     Tk_ClassProcs *procsPtr;
     procsPtr = (Tk_ClassProcs*) Tcl_Alloc(sizeof(Tk_ClassProcs));
     procsPtr->size             = sizeof(Tk_ClassProcs);
     procsPtr->createProc       = Togl_CreateWindow;
     procsPtr->worldChangedProc = NULL;
     procsPtr->modalProc        = NULL;
     Tk_SetClassProcs(togl->TkWin,procsPtr,(ClientData)togl);
   }
#else
   {                                  /* use private API */
     /* 
        We need to set these fields in the Tk_FakeWin structure:
        dummy17 = classProcsPtr
        dummy18 = instanceData
     */
     TkClassProcs *procsPtr;
     Tk_FakeWin *winPtr = (Tk_FakeWin*)(togl->TkWin);
     
     procsPtr = (TkClassProcs*)Tcl_Alloc(sizeof(TkClassProcs));
     procsPtr->createProc     = Togl_CreateWindow;
     procsPtr->geometryProc   = NULL;
     procsPtr->modalProc      = NULL;
     winPtr->dummy17 = (char*)procsPtr;
     winPtr->dummy18 = (ClientData)togl;
   }
#endif
   
   Tk_CreateEventHandler(tkwin,
                         ExposureMask | StructureNotifyMask,
                         Togl_EventProc,
                         (ClientData)togl);

   /* Configure Togl widget */
   if (Togl_Configure(interp, togl, argc-2, argv+2, 0) == TCL_ERROR) {
      Tk_DestroyWindow(tkwin);
      goto error;
   }

   /*
    * If OpenGL window wasn't already created by Togl_Configure() we
    * create it now.  We can tell by checking if the GLX context has
    * been initialized.
    */
   if (!
#if defined(WIN32)
       togl->tglGLHdc
#elif defined(X11)
       togl->GlCtx
#elif defined(macintosh)
       togl->aglCtx
#endif
       ) 
     {
       Tk_MakeWindowExist(togl->TkWin);
       if (Tk_WindowId(togl->TkWin)==DUMMY_WINDOW) {
	 return TCL_ERROR;
       }
       Togl_MakeCurrent(togl);
     }
   
   /* If defined, call create callback */
   if (togl->CreateProc) {
     togl->CreateProc(togl);
   }

   /* If defined, call reshape proc */
   if (togl->ReshapeProc) {
      togl->ReshapeProc(togl);
   }

   /* If defined, setup timer */
   if (togl->TimerProc){
      Tk_CreateTimerHandler( togl->TimerInterval, Togl_Timer, (ClientData)togl );
   }

   Tcl_AppendResult(interp, Tk_PathName(tkwin), NULL);

   /* Add to linked list */
   AddToList(togl);

   return TCL_OK;

error:
   Tcl_DeleteCommand(interp, "togl");
   /*free(togl);   Don't free it, if we do a crash occurs later...*/
   return TCL_ERROR;
}


#ifdef USE_OVERLAY

/*
 * Do all the setup for overlay planes
 * Return:   TCL_OK or TCL_ERROR
 */
static int SetupOverlay( struct Togl *togl )
{
#if defined(X11)

#ifdef GLX_TRANSPARENT_TYPE_EXT
   static int ovAttributeList[] = {
      GLX_BUFFER_SIZE, 2,
      GLX_LEVEL, 1,
      GLX_TRANSPARENT_TYPE_EXT, GLX_TRANSPARENT_INDEX_EXT,
      None
   };
#else
   static int ovAttributeList[] = {
      GLX_BUFFER_SIZE, 2,
      GLX_LEVEL, 1,
      None
   };
#endif

   Display *dpy;
   XVisualInfo *visinfo;
   TkWindow *winPtr = (TkWindow *) togl->TkWin;

   XSetWindowAttributes swa;
   Tcl_HashEntry *hPtr;
   int new_flag;

   dpy = Tk_Display(togl->TkWin);

   visinfo = glXChooseVisual( dpy, Tk_ScreenNumber(winPtr), ovAttributeList );
   if (!visinfo){
      Tcl_AppendResult(togl->Interp,Tk_PathName(winPtr),
                       ": No suitable overlay index visual available",
                       (char *) NULL);
      togl->OverlayCtx = 0;
      togl->OverlayWindow = 0;
      togl->OverlayCmap = 0;
      return TCL_ERROR;
   }

#ifdef GLX_TRANSPARENT_INDEX_EXT
   {
      int fail = glXGetConfig(dpy, visinfo,GLX_TRANSPARENT_INDEX_VALUE_EXT,
                              &togl->OverlayTransparentPixel);
      if (fail)
         togl->OverlayTransparentPixel=0; /* maybe, maybe ... */
   }
#else
   togl->OverlayTransparentPixel=0; /* maybe, maybe ... */
#endif

   /*
   togl->OverlayCtx = glXCreateContext( dpy, visinfo, None, GL_TRUE );
   */
   /* NEW in Togl 1.5 beta 3 */
   /* share display lists with normal layer context */
   togl->OverlayCtx = glXCreateContext( dpy, visinfo,
                                        togl->GlCtx, !togl->Indirect );

   swa.colormap = XCreateColormap( dpy, XRootWindow(dpy, visinfo->screen),
                                   visinfo->visual, AllocNone );
   togl->OverlayCmap = swa.colormap;

   swa.border_pixel = 0;
   swa.event_mask = ALL_EVENTS_MASK;
   togl->OverlayWindow = XCreateWindow( dpy, Tk_WindowId(togl->TkWin), 0, 0,
                                        togl->Width, togl->Height, 0,
                                        visinfo->depth, InputOutput,
                                        visinfo->visual,
                                        CWBorderPixel|CWColormap|CWEventMask,
                                        &swa );

   hPtr = Tcl_CreateHashEntry( &winPtr->dispPtr->winTable,
                               (char *) togl->OverlayWindow, &new_flag );
   Tcl_SetHashValue( hPtr, winPtr );

/*   XMapWindow( dpy, togl->OverlayWindow );*/
   togl->OverlayIsMapped = 0;

   /* Make sure window manager installs our colormap */
   XSetWMColormapWindows( dpy, togl->OverlayWindow, &togl->OverlayWindow, 1 );

   return TCL_OK;

#elif defined(WIN32) || defined(macintosh) /* not yet implemented on these */

   return TCL_ERROR;

#endif /* X11 */
}

#endif /* USE_OVERLAY */



#ifdef WIN32
#define TOGL_CLASS_NAME "Togl Class"
static ToglClassInitialized = 0;

static LRESULT CALLBACK Win32WinProc( HWND hwnd, UINT message,
                                    WPARAM wParam, LPARAM lParam)
{
    LONG result;
    struct Togl *togl = (struct Togl*) GetWindowLong(hwnd, 0);
    WNDCLASS childClass;

    switch( message ){
    case WM_WINDOWPOSCHANGED:
        /* Should be processed by DefWindowProc, otherwise a double buffered
        context is not properly resized when the corresponding window is resized.*/
        break;
    case WM_DESTROY:
        if (togl->tglGLHglrc) {
            wglDeleteContext(togl->tglGLHglrc);
        }
        if (togl->tglGLHdc) {
            ReleaseDC(hwnd, togl->tglGLHdc);
        }
        free(togl);
        break;
    default:
#if USE_STATIC_LIB
        return TkWinChildProc(hwnd, message, wParam, lParam);
#else
        /*
         * OK, since TkWinChildProc is not explicitly exported in the
         * dynamic libraries, we have to retrieve it from the class info
         * registered with windows.
         *
         */
        if (tkWinChildProc == NULL) {
           GetClassInfo(Tk_GetHINSTANCE(),TK_WIN_CHILD_CLASS_NAME,
                        &childClass);
           tkWinChildProc = childClass.lpfnWndProc;
        }
        return tkWinChildProc(hwnd, message, wParam, lParam);
#endif
    }
    result = DefWindowProc(hwnd, message, wParam, lParam);
    Tcl_ServiceAll();
    return result;
}
#endif /* WIN32 */



/*
 * Togl_CreateWindow
 *
 *   Window creation function, invoked as a callback from Tk_MakeWindowExist.
 *   Creates an OpenGL window for the Togl widget.
 */
static Window Togl_CreateWindow(Tk_Window tkwin,
				Window parent, 
				ClientData instanceData) {
  (void)tkwin;
  struct Togl *togl = (struct Togl*) instanceData;
  XVisualInfo *visinfo = NULL;
  Display *dpy;
  Colormap cmap;
  int scrnum;
  int directCtx = GL_TRUE;
  Window window;

#if defined(X11)
  int attrib_list[1000];
  int attrib_count;
  int dummy;
  XSetWindowAttributes swa;
#define MAX_ATTEMPTS 12
   static int ci_depths[MAX_ATTEMPTS] = {
      8, 4, 2, 1, 12, 16, 8, 4, 2, 1, 12, 16
   };
   static int dbl_flags[MAX_ATTEMPTS] = {
      0, 0, 0, 0,  0,  0, 1, 1, 1, 1,  1,  1
   };
#elif defined(WIN32)
   HWND hwnd, parentWin;
   int pixelformat;
   HANDLE hInstance;
   WNDCLASS ToglClass;
   PIXELFORMATDESCRIPTOR pfd;
   XVisualInfo VisInf;
#elif defined(macintosh)
   GLint attribs[20];
   int na;
   AGLPixelFormat fmt;
   XVisualInfo VisInf;
#endif /* X11 */


   dpy = Tk_Display(togl->TkWin);

#if defined(X11)
   /* Make sure OpenGL's GLX extension supported */
   if (!glXQueryExtension(dpy, &dummy, &dummy)) {
     Tcl_SetResult(togl->Interp, "Togl: X server has no OpenGL GLX extension",TCL_STATIC);
     return DUMMY_WINDOW;
   }

   if (togl->ShareContext && FindTogl(togl->ShareContext)) {
      /* share OpenGL context with existing Togl widget */
      struct Togl *shareWith = FindTogl(togl->ShareContext);
      assert(shareWith);
      assert(shareWith->GlCtx);
      togl->GlCtx = shareWith->GlCtx;
      printf("SHARE CTX\n");
   }
   else {
      int attempt;
      /* It may take a few tries to get a visual */
      for (attempt=0; attempt<MAX_ATTEMPTS; attempt++) {
         attrib_count = 0;
         attrib_list[attrib_count++] = GLX_USE_GL;
         if (togl->RgbaFlag) {
            /* RGB[A] mode */
            attrib_list[attrib_count++] = GLX_RGBA;
            attrib_list[attrib_count++] = GLX_RED_SIZE;
            attrib_list[attrib_count++] = togl->RgbaRed;
            attrib_list[attrib_count++] = GLX_GREEN_SIZE;
            attrib_list[attrib_count++] = togl->RgbaGreen;
            attrib_list[attrib_count++] = GLX_BLUE_SIZE;
            attrib_list[attrib_count++] = togl->RgbaBlue;
            if (togl->AlphaFlag) {
               attrib_list[attrib_count++] = GLX_ALPHA_SIZE;
               attrib_list[attrib_count++] = togl->AlphaSize;
            }
         }
         else {
            /* Color index mode */
            int depth;
            attrib_list[attrib_count++] = GLX_BUFFER_SIZE;
            depth = ci_depths[attempt];
            attrib_list[attrib_count++] = depth;
         }
         if (togl->DepthFlag) {
            attrib_list[attrib_count++] = GLX_DEPTH_SIZE;
            attrib_list[attrib_count++] = togl->DepthSize;
         }
         if (togl->DoubleFlag || dbl_flags[attempt]) {
            attrib_list[attrib_count++] = GLX_DOUBLEBUFFER;
         }
         if (togl->StencilFlag) {
            attrib_list[attrib_count++] = GLX_STENCIL_SIZE;
            attrib_list[attrib_count++] = togl->StencilSize;
         }
         if (togl->AccumFlag) {
            attrib_list[attrib_count++] = GLX_ACCUM_RED_SIZE;
            attrib_list[attrib_count++] = togl->AccumRed;
            attrib_list[attrib_count++] = GLX_ACCUM_GREEN_SIZE;
            attrib_list[attrib_count++] = togl->AccumGreen;
            attrib_list[attrib_count++] = GLX_ACCUM_BLUE_SIZE;
            attrib_list[attrib_count++] = togl->AccumBlue;
            if (togl->AlphaFlag) {
               attrib_list[attrib_count++] = GLX_ACCUM_ALPHA_SIZE;
               attrib_list[attrib_count++] = togl->AccumAlpha;
            }
         }
         if (togl->AuxNumber != 0) {
            attrib_list[attrib_count++] = GLX_AUX_BUFFERS;
            attrib_list[attrib_count++] = togl->AuxNumber;
         }
         if (togl->Indirect) {
            directCtx = GL_FALSE;
         }

         /* stereo hack */
         /*
           if (togl->StereoFlag) {
           attrib_list[attrib_count++] = GLX_STEREO;
           }
         */
         attrib_list[attrib_count++] = None;

         visinfo = glXChooseVisual(dpy, Tk_ScreenNumber(togl->TkWin),
                                   attrib_list);
         if (visinfo) {
            /* found a GLX visual! */
            break;
         }
      }

      if (visinfo==NULL) {
	Tcl_SetResult(togl->Interp,"Togl: couldn't get visual",TCL_STATIC);
	return DUMMY_WINDOW;
      }

      /*
       * Create a new OpenGL rendering context.
       */
      if (togl->ShareList) {
         /* share display lists with existing togl widget */
         struct Togl *shareWith = FindTogl(togl->ShareList);
         GLXContext shareCtx;
         if (shareWith)
            shareCtx = shareWith->GlCtx;
         else
            shareCtx = None;
         togl->GlCtx = glXCreateContext(dpy, visinfo, shareCtx, directCtx);
      }
      else {
         /* don't share display lists */
         togl->GlCtx = glXCreateContext(dpy, visinfo, None, directCtx);
      }

      if (togl->GlCtx == NULL) {
         Tcl_SetResult(togl->Interp, "could not create rendering context",TCL_STATIC);
	 return DUMMY_WINDOW;
      }

   }


#endif /* X11 */

#ifdef WIN32
   parentWin = Tk_GetHWND(parent);
   hInstance = Tk_GetHINSTANCE();
   if (ToglClassInitialized == 0) {
       ToglClassInitialized = 1;
       ToglClass.style = CS_HREDRAW | CS_VREDRAW;
       ToglClass.cbClsExtra = 0;
       ToglClass.cbWndExtra = 4;   /* to save struct Togl* */
       ToglClass.hInstance = hInstance;
       ToglClass.hbrBackground = NULL;
       ToglClass.lpszMenuName = NULL;
       ToglClass.lpszClassName = TOGL_CLASS_NAME;
       ToglClass.lpfnWndProc = Win32WinProc;
       ToglClass.hIcon = NULL;
       ToglClass.hCursor = NULL;
       if (!RegisterClass(&ToglClass)){
           Tcl_SetResult(togl->Interp, "unable register Togl window class",TCL_STATIC);
	   return DUMMY_WINDOW;
       }
   }

   hwnd = CreateWindow(TOGL_CLASS_NAME, NULL, WS_CHILD | WS_CLIPCHILDREN
                       | WS_CLIPSIBLINGS, 0, 0, togl->Width, togl->Height,
                       parentWin, NULL, hInstance, NULL);
   SetWindowLong(hwnd, 0, (LONG) togl);
   SetWindowPos(hwnd, HWND_TOP, 0, 0, 0, 0,
  	            SWP_NOACTIVATE | SWP_NOMOVE | SWP_NOSIZE);

   togl->tglGLHdc = GetDC(hwnd);

   pfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);
   pfd.nVersion = 1;
   pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL;
   if (togl->DoubleFlag) {
        pfd.dwFlags |= PFD_DOUBLEBUFFER;
   }
   /* The stereo flag is not supported in the current generic OpenGL
    * implementation, but may be supported by specific hardware devices.
    */
   if (togl->StereoFlag) {
        pfd.dwFlags |= PFD_STEREO;
   }

   pfd.cColorBits = togl->RgbaRed + togl->RgbaGreen + togl->RgbaBlue;
   pfd.iPixelType = togl->RgbaFlag ? PFD_TYPE_RGBA : PFD_TYPE_COLORINDEX;
   /* Alpha bitplanes are not supported in the current generic OpenGL
    * implementation, but may be supported by specific hardware devices.
    */
   pfd.cAlphaBits = togl->AlphaFlag ? togl->AlphaSize : 0;
   pfd.cAccumBits = togl->AccumFlag ? (togl->AccumRed + togl->AccumGreen +
                                       togl->AccumBlue +togl->AccumAlpha) : 0;
   pfd.cDepthBits = togl->DepthFlag ? togl->DepthSize : 0;
   pfd.cStencilBits = togl->StencilFlag ? togl->StencilSize : 0;
   /* Auxiliary buffers are not supported in the current generic OpenGL
    * implementation, but may be supported by specific hardware devices.
    */
   pfd.cAuxBuffers = togl->AuxNumber;
   pfd.iLayerType = PFD_MAIN_PLANE;

   if ( (pixelformat = ChoosePixelFormat(togl->tglGLHdc, &pfd)) == 0 ) {
        Tcl_SetResult(togl->Interp, "Togl: couldn't choose pixel format",TCL_STATIC);
	return DUMMY_WINDOW;
   }
   if (SetPixelFormat(togl->tglGLHdc, pixelformat, &pfd) == FALSE) {
        Tcl_SetResult(togl->Interp, "Togl: couldn't choose pixel format",TCL_STATIC);
	return DUMMY_WINDOW;
   }

   /* Get the actual pixel format */
   DescribePixelFormat(togl->tglGLHdc, pixelformat, sizeof(pfd), &pfd);

   if (togl->ShareContext && FindTogl(togl->ShareContext)) {
      /* share OpenGL context with existing Togl widget */
      struct Togl *shareWith = FindTogl(togl->ShareContext);
      assert(shareWith);
      assert(shareWith->tglGLHglrc);
      togl->tglGLHglrc = shareWith->tglGLHglrc;
      togl->VisInfo = shareWith->VisInfo;
      visinfo = togl->VisInfo;
   }
   else {
      /*
       * Create a new OpenGL rendering context. And check to share lists.
       */
      togl->tglGLHglrc = wglCreateContext(togl->tglGLHdc);

      if (togl->ShareList) {
         /* share display lists with existing togl widget */
         struct Togl *shareWith = FindTogl(togl->ShareList);
         if (shareWith)
            wglShareLists(shareWith->tglGLHglrc, togl->tglGLHglrc);
      }

      if (!togl->tglGLHglrc) {
         Tcl_SetResult(togl->Interp, "could not create rendering context",TCL_STATIC);
	 return DUMMY_WINDOW;
      }

      /* Just for portability, define the simplest visinfo */
      visinfo = &VisInf;
      visinfo->visual = DefaultVisual(dpy, DefaultScreen(dpy));
      visinfo->depth = visinfo->visual->bits_per_rgb;
   }

#endif /*WIN32 */


   /*
    * find a colormap
    */
   scrnum = Tk_ScreenNumber(togl->TkWin);
   if (togl->RgbaFlag) {
      /* Colormap for RGB mode */
#if defined(X11)
      cmap = get_rgb_colormap( dpy, scrnum, visinfo, togl->TkWin );

#elif defined(WIN32)
      if (pfd.dwFlags & PFD_NEED_PALETTE) {
         cmap = Win32CreateRgbColormap(pfd);
      }
      else {
         cmap = DefaultColormap(dpy,scrnum);
      }
#elif defined(macintosh)
      cmap = DefaultColormap(dpy, scrnum);
#endif /* X11 */
   }
   else {
      /* Colormap for CI mode */
#ifdef WIN32
      togl->CiColormapSize = 1 << pfd.cColorBits;
      togl->CiColormapSize = togl->CiColormapSize < MAX_CI_COLORMAP_SIZE ?
                             togl->CiColormapSize : MAX_CI_COLORMAP_SIZE;

#endif /* WIN32 */
      if (togl->PrivateCmapFlag) {
         /* need read/write colormap so user can store own color entries */
#if defined(X11)
         cmap = XCreateColormap(dpy, XRootWindow(dpy, visinfo->screen),
                                visinfo->visual, AllocAll);
#elif defined(WIN32)
         cmap = Win32CreateCiColormap(togl);
#elif defined(macintosh)
         /* need to figure out how to do this correctly on Mac... */
         cmap = DefaultColormap(dpy, scrnum);
#endif /* X11 */
      }
      else {
         if (visinfo->visual==DefaultVisual(dpy, scrnum)) {
            /* share default/root colormap */
            cmap = Tk_Colormap(togl->TkWin);
         }
         else {
            /* make a new read-only colormap */
            cmap = XCreateColormap(dpy, XRootWindow(dpy, visinfo->screen),
                                   visinfo->visual, AllocNone);
         }
      }
   }

   /* Make sure Tk knows to switch to the new colormap when the cursor
    * is over this window when running in color index mode.
    */
   Tk_SetWindowVisual(togl->TkWin, visinfo->visual, visinfo->depth, cmap);
#ifdef WIN32
   /* Install the colormap */
   SelectPalette(togl->tglGLHdc, ((TkWinColormap *)cmap)->palette, TRUE);
   RealizePalette(togl->tglGLHdc);
#endif /* WIN32 */

#if defined(X11)
   swa.colormap = cmap;
   swa.border_pixel = 0;
   swa.event_mask = ALL_EVENTS_MASK;
   window = XCreateWindow(dpy, parent,
                                  0, 0, togl->Width, togl->Height,
                                  0, visinfo->depth,
                                  InputOutput, visinfo->visual,
                                  CWBorderPixel | CWColormap | CWEventMask,
                                  &swa);
   /* Make sure window manager installs our colormap */
   XSetWMColormapWindows( dpy,window, &window, 1 );

#elif defined(WIN32)
   window = Tk_AttachHWND((Tk_Window)winPtr, hwnd);

#elif defined(macintosh)
   window = TkpMakeWindow(winPtr, parent);
#endif /* X11 */

#ifdef USE_OVERLAY
   if (togl->OverlayFlag) {
      if (SetupOverlay( togl )==TCL_ERROR) {
         fprintf(stderr,"Warning: couldn't setup overlay.\n");
         togl->OverlayFlag = 0;
      }
   }
#endif /* USE_OVERLAY */

   /* Request the X window to be displayed */
   XMapWindow(dpy, window);

#ifdef macintosh
   if (togl->ShareContext && FindTogl(togl->ShareContext)) {
      /* share OpenGL context with existing Togl widget */
      struct Togl *shareWith = FindTogl(togl->ShareContext);
      assert(shareWith);
      assert(shareWith->aglCtx);
      togl->aglCtx = shareWith->aglCtx;
      togl->VisInfo = shareWith->VisInfo;
      visinfo = togl->VisInfo;
   
   } else {
       AGLContext shareCtx = NULL;

       /* Need to do this after mapping window, so MacDrawable structure is more
          completely filled in */
       na = 0;
       attribs[na++] = AGL_MINIMUM_POLICY;
       attribs[na++] = AGL_ROBUST;
       if (togl->RgbaFlag) {
          /* RGB[A] mode */
          attribs[na++] = AGL_RGBA;
          attribs[na++] = AGL_RED_SIZE;
          attribs[na++] = togl->RgbaRed;
          attribs[na++] = AGL_GREEN_SIZE;
          attribs[na++] = togl->RgbaGreen;
          attribs[na++] = AGL_BLUE_SIZE;
          attribs[na++] = togl->RgbaBlue;
          if (togl->AlphaFlag) {
             attribs[na++] = AGL_ALPHA_SIZE;
             attribs[na++] = togl->AlphaSize;
          }
       } else {
          /* Color index mode */
          attribs[na++] = AGL_BUFFER_SIZE;
          attribs[na++] = 8;
       }
       if (togl->DepthFlag) {
          attribs[na++] = AGL_DEPTH_SIZE;
          attribs[na++] = togl->DepthSize;
       }
       if (togl->DoubleFlag) {
          attribs[na++] = AGL_DOUBLEBUFFER;
       }
       if (togl->StencilFlag) {
          attribs[na++] = AGL_STENCIL_SIZE;
          attribs[na++] = togl->StencilSize;
       }
       if (togl->AccumFlag) {
          attribs[na++] = AGL_ACCUM_RED_SIZE;
          attribs[na++] = togl->AccumRed;
          attribs[na++] = AGL_ACCUM_GREEN_SIZE;
          attribs[na++] = togl->AccumGreen;
          attribs[na++] = AGL_ACCUM_BLUE_SIZE;
          attribs[na++] = togl->AccumBlue;
          if (togl->AlphaFlag) {
             attribs[na++] = AGL_ACCUM_ALPHA_SIZE;
             attribs[na++] = togl->AccumAlpha;
          }
       }
       if (togl->AuxNumber != 0) {
          attribs[na++] = AGL_AUX_BUFFERS;
          attribs[na++] = togl->AuxNumber;
       }
       attribs[na++] = AGL_NONE;

       if ((fmt = aglChoosePixelFormat(NULL, 0, attribs)) == NULL) {
	 Tcl_SetResult(togl->Interp, "Togl: couldn't choose pixel format",TCL_STATIC);
	 return DUMMY_WINDOW;
       }

       /*
        * Check whether to share lists.
        */
       if (togl->ShareList) {
          /* share display lists with existing togl widget */
          struct Togl *shareWith = FindTogl(togl->ShareList);
          if (shareWith)
             shareCtx = shareWith->aglCtx;
       }
       if ((togl->aglCtx = aglCreateContext(fmt, shareCtx)) == NULL) {
          aglDestroyPixelFormat(fmt);
          Tcl_SetResult(togl->Interp, "Togl: couldn't create context",TCL_STATIC);
	  return DUMMY_WINDOW;
       }
       
       aglDestroyPixelFormat(fmt);
       if (!aglSetDrawable(togl->aglCtx,
                           ((MacDrawable *) (window))->toplevel->portPtr)) {
          aglDestroyContext(togl->aglCtx);
          Tcl_SetResult(togl->Interp, "Togl: couldn't set drawable",TCL_STATIC);
	  return DUMMY_WINDOW;
       }

       /* Just for portability, define the simplest visinfo */
       visinfo = &VisInf;
       visinfo->visual = DefaultVisual(dpy, DefaultScreen(dpy));
       visinfo->depth = visinfo->visual->bits_per_rgb;
   }
#endif                          /* macintosh */

#if defined(X11)
   /* Check for a single/double buffering snafu */
   {
      int dbl_flag;
      if (glXGetConfig( dpy, visinfo, GLX_DOUBLEBUFFER, &dbl_flag )) {
         if (togl->DoubleFlag==0 && dbl_flag) {
            /* We requested single buffering but had to accept a */
            /* double buffered visual.  Set the GL draw buffer to */
            /* be the front buffer to simulate single buffering. */
            glDrawBuffer( GL_FRONT );
         }
      }
   }
#endif /* X11 */

   return window;
}

/*
 * ToglCmdDeletedProc
 *
 *      This procedure is invoked when a widget command is deleted.  If
 *      the widget isn't already in the process of being destroyed,
 *      this command destroys it.
 *
 * Results:
 *      None.
 *
 * Side effects:
 *      The widget is destroyed.
 *
 *----------------------------------------------------------------------
 */
static void ToglCmdDeletedProc( ClientData clientData )
{
   struct Togl *togl = (struct Togl *)clientData;
   Tk_Window tkwin;

   if (togl == NULL) return;

   tkwin = togl->TkWin;

   /*
    * This procedure could be invoked either because the window was
    * destroyed and the command was then deleted (in which case tkwin
    * is NULL) or because the command was deleted, and then this procedure
    * destroys the widget.
    */

   /* NEW in togl 1.5 beta 3 */
   if (togl && tkwin) {
      Tk_DeleteEventHandler(tkwin,
                         ExposureMask | StructureNotifyMask,
                         Togl_EventProc,
                         (ClientData)togl);
   }

   /* NEW in togl 1.5 beta 3 */
#if defined(X11)
   if (togl->GlCtx) {
      /* XXX this might be bad if two or more Togl widgets share a context */
      glXDestroyContext( togl->display, togl->GlCtx );
      togl->GlCtx = NULL;
   }
#ifdef USE_OVERLAY
   if (togl->OverlayCtx) {
      Tcl_HashEntry *entryPtr;
      TkWindow *winPtr = (TkWindow *) togl->TkWin;
      if (winPtr) {
         entryPtr = Tcl_FindHashEntry(&winPtr->dispPtr->winTable,
                                      (char *) togl->OverlayWindow );
         Tcl_DeleteHashEntry(entryPtr);
      }
      glXDestroyContext( togl->display, togl->OverlayCtx );
      togl->OverlayCtx = NULL;
   }
#endif /* USE_OVERLAY */
#endif

   if (tkwin != NULL) {
      togl->TkWin = NULL;
      Tk_DestroyWindow(tkwin);
   }
}


/*
 * Togl_Destroy
 *
 * Gets called when an Togl widget is destroyed.
 */
#if (TK_MAJOR_VERSION * 100 + TK_MINOR_VERSION) >= 401
static void Togl_Destroy( char *clientData )
#else
static void Togl_Destroy( ClientData clientData )
#endif
{
   struct Togl *togl = (struct Togl *)clientData;

   Tk_FreeOptions(configSpecs, (char *)togl, togl->display, 0);

#ifndef NO_TK_CURSOR
   if (togl->Cursor != None) {
      Tk_FreeCursor(togl->display, togl->Cursor);
   }
#endif
   if (togl->DestroyProc) {
      togl->DestroyProc(togl);
   }

   /* remove from linked list */
   RemoveFromList(togl);

#if defined(X11)
   free(togl);
#elif defined(macintosh)
   free ((char *) togl);
#endif
}



/*
 * This gets called to handle Togl window configuration events
 */
static void Togl_EventProc(ClientData clientData, XEvent *eventPtr)
{
   struct Togl *togl = (struct Togl *)clientData;

   switch (eventPtr->type) {
      case Expose:
         if (eventPtr->xexpose.count == 0) {
            if (!togl->UpdatePending &&
                eventPtr->xexpose.window==Tk_WindowId(togl->TkWin)) {
               Togl_PostRedisplay(togl);
            }
#if defined(X11)
            if (!togl->OverlayUpdatePending && togl->OverlayFlag
                && togl->OverlayIsMapped
                && eventPtr->xexpose.window==togl->OverlayWindow){
               Togl_PostOverlayRedisplay(togl);
            }
#endif /*X11*/
         }
         break;
      case ConfigureNotify:
         if (togl->Width != Tk_Width(togl->TkWin) ||
             togl->Height != Tk_Height(togl->TkWin)) {
            togl->Width = Tk_Width(togl->TkWin);
            togl->Height = Tk_Height(togl->TkWin);
            XResizeWindow(Tk_Display(togl->TkWin), Tk_WindowId(togl->TkWin),
                          togl->Width, togl->Height);
#if defined(X11)
            if (togl->OverlayFlag) {
               XResizeWindow( Tk_Display(togl->TkWin), togl->OverlayWindow,
                              togl->Width, togl->Height );
               XRaiseWindow( Tk_Display(togl->TkWin), togl->OverlayWindow );
            }
#endif /*X11*/
            Togl_MakeCurrent(togl);
            if (togl->ReshapeProc) {
               togl->ReshapeProc(togl);
            }
            else {
               glViewport(0, 0, togl->Width, togl->Height);
#if defined(X11)
               if (togl->OverlayFlag) {
                  Togl_UseLayer( togl,TOGL_OVERLAY );
                  glViewport( 0, 0, togl->Width, togl->Height );
                  Togl_UseLayer( togl, TOGL_NORMAL );
               }
#endif /*X11*/
            }
#ifndef WIN32 /* causes double redisplay on Win32 platform */
            Togl_PostRedisplay(togl);
#endif /* WIN32 */
         }
         break;
      case MapNotify:
         break;
      case DestroyNotify:
	 if (togl->TkWin != NULL) {
	    togl->TkWin = NULL;
#if (TCL_MAJOR_VERSION * 100 + TCL_MINOR_VERSION) >= 800
            /* This function new in Tcl/Tk 8.0 */
            Tcl_DeleteCommandFromToken( togl->Interp, togl->widgetCmd );
#endif
	 }
	 if (togl->TimerProc != NULL) {
#if (TK_MAJOR_VERSION * 100 + TK_MINOR_VERSION) >= 401
	    Tcl_DeleteTimerHandler(togl->timerHandler);
#else
	    Tk_DeleteTimerHandler(togl->timerHandler);
#endif

	 }
	 if (togl->UpdatePending) {
#if (TCL_MAJOR_VERSION * 100 + TCL_MINOR_VERSION) >= 705
            Tcl_CancelIdleCall(Togl_Render, (ClientData) togl);
#else
            Tk_CancelIdleCall(Togl_Render, (ClientData) togl);
#endif
	 }

#if (TK_MAJOR_VERSION * 100 + TK_MINOR_VERSION) >= 401
         Tcl_EventuallyFree( (ClientData) togl, Togl_Destroy );
#else
         Tk_EventuallyFree((ClientData)togl, Togl_Destroy);
#endif

         break;
      default:
         /*nothing*/
         ;
   }
}



void Togl_PostRedisplay( struct Togl *togl )
{
   if (!togl->UpdatePending) {
      togl->UpdatePending = GL_TRUE;
      Tk_DoWhenIdle( Togl_Render, (ClientData) togl );
   }
}



void Togl_SwapBuffers( const struct Togl *togl )
{
   if (togl->DoubleFlag) {
#if defined(WIN32)
      int res = SwapBuffers(togl->tglGLHdc);
      assert(res == TRUE);
#elif defined(X11)
      glXSwapBuffers( Tk_Display(togl->TkWin), Tk_WindowId(togl->TkWin) );
#elif defined(macintosh)
      aglSwapBuffers(togl->aglCtx);
#endif /* WIN32 */
   }
   else {
      glFlush();
   }
}



char *Togl_Ident( const struct Togl *togl )
{
   return togl->Ident;
}


int Togl_Width( const struct Togl *togl )
{
   return togl->Width;
}


int Togl_Height( const struct Togl *togl )
{
   return togl->Height;
}


Tcl_Interp *Togl_Interp( const struct Togl *togl )
{
   return togl->Interp;
}


Tk_Window Togl_TkWin( const struct Togl *togl )
{
   return togl->TkWin;
}


#if defined(X11)
/*
 * A replacement for XAllocColor.  This function should never
 * fail to allocate a color.  When XAllocColor fails, we return
 * the nearest matching color.  If we have to allocate many colors
 * this function isn't too efficient; the XQueryColors() could be
 * done just once.
 * Written by Michael Pichler, Brian Paul, Mark Kilgard
 * Input:  dpy - X display
 *         cmap - X colormap
 *         cmapSize - size of colormap
 * In/Out: color - the XColor struct
 * Output:  exact - 1=exact color match, 0=closest match
 */
static void
noFaultXAllocColor( Display *dpy, Colormap cmap, int cmapSize,
                    XColor *color, int *exact )
{
   XColor *ctable, subColor;
   int i, bestmatch;
   double mindist;       /* 3*2^16^2 exceeds long int precision.
                          */

   /* First try just using XAllocColor. */
   if (XAllocColor(dpy, cmap, color)) {
      *exact = 1;
      return;
   }

   /* Retrieve color table entries. */
   /* XXX alloca candidate. */
   ctable = (XColor *) malloc(cmapSize * sizeof(XColor));
   for (i = 0; i < cmapSize; i++) {
      ctable[i].pixel = i;
   }
   XQueryColors(dpy, cmap, ctable, cmapSize);

   /* Find best match. */
   bestmatch = -1;
   mindist = 0.0;
   for (i = 0; i < cmapSize; i++) {
      double dr = (double) color->red - (double) ctable[i].red;
      double dg = (double) color->green - (double) ctable[i].green;
      double db = (double) color->blue - (double) ctable[i].blue;
      double dist = dr * dr + dg * dg + db * db;
      if (bestmatch < 0 || dist < mindist) {
         bestmatch = i;
         mindist = dist;
      }
   }

   /* Return result. */
   subColor.red = ctable[bestmatch].red;
   subColor.green = ctable[bestmatch].green;
   subColor.blue = ctable[bestmatch].blue;

   /* Try to allocate the closest match color.  This should only
    * fail if the cell is read/write.  Otherwise, we're incrementing
    * the cell's reference count.
    */
   if (!XAllocColor(dpy, cmap, &subColor)) {
      /* do this to work around a problem reported by Frank Ortega */
      subColor.pixel = (unsigned long) bestmatch;
      subColor.red   = ctable[bestmatch].red;
      subColor.green = ctable[bestmatch].green;
      subColor.blue  = ctable[bestmatch].blue;
      subColor.flags = DoRed | DoGreen | DoBlue;
   }
   *color = subColor;
   free(ctable);
}

#elif defined(WIN32)

static UINT Win32AllocColor( const struct Togl *togl,
                             float red, float green, float blue )
{
/* Modified version of XAllocColor emulation of Tk.
*      - returns index, instead of color itself
*      - allocates logical palette entry even for non-palette devices
*/

    TkWinColormap *cmap = (TkWinColormap *) Tk_Colormap(togl->TkWin);
    UINT index;
    COLORREF newColor, closeColor;
    PALETTEENTRY entry, closeEntry;
    int new, refCount;
    Tcl_HashEntry *entryPtr;

    entry.peRed   = (unsigned char)(red*255 + .5);
    entry.peGreen = (unsigned char)(green*255 + .5);
    entry.peBlue  = (unsigned char)(blue*255 + .5);
    entry.peFlags = 0;

	/*
	 * Find the nearest existing palette entry.
	 */

    newColor = RGB(entry.peRed, entry.peGreen, entry.peBlue);
    index = GetNearestPaletteIndex(cmap->palette, newColor);
    GetPaletteEntries(cmap->palette, index, 1, &closeEntry);
    closeColor = RGB(closeEntry.peRed, closeEntry.peGreen,  closeEntry.peBlue);

     /*
	 * If this is not a duplicate and colormap is not full, allocate a new entry.
	 */

	if (newColor != closeColor) {
        if (cmap->size == (unsigned int)togl->CiColormapSize) {
            entry = closeEntry;
        }
        else {
            cmap->size++;
		    ResizePalette(cmap->palette, cmap->size);
		    index = cmap->size -1;
		    SetPaletteEntries(cmap->palette, index, 1, &entry);
		    SelectPalette(togl->tglGLHdc, cmap->palette, TRUE);
		    RealizePalette(togl->tglGLHdc);
		}
	}
	newColor = PALETTERGB(entry.peRed, entry.peGreen, entry.peBlue);
	entryPtr = Tcl_CreateHashEntry(&cmap->refCounts, (char *) newColor, &new);
	if (new) {
	    refCount = 1;
	} else {
	    refCount = ((int) Tcl_GetHashValue(entryPtr)) + 1;
	}
	Tcl_SetHashValue(entryPtr, (ClientData)refCount);

    return index;
}

static void Win32FreeColor( const struct Togl *togl, unsigned long index )
{
    TkWinColormap *cmap = (TkWinColormap *) Tk_Colormap(togl->TkWin);
    COLORREF cref;
    UINT count, refCount;
    PALETTEENTRY entry, *entries;
    Tcl_HashEntry *entryPtr;

	if (index >= cmap->size ) {
		panic("Tried to free a color that isn't allocated.");
	}
	GetPaletteEntries(cmap->palette, index, 1, &entry);
	cref = PALETTERGB(entry.peRed, entry.peGreen, entry.peBlue);
	entryPtr = Tcl_FindHashEntry(&cmap->refCounts, (char *) cref);
	if (!entryPtr) {
		panic("Tried to free a color that isn't allocated.");
	}
	refCount = (int) Tcl_GetHashValue(entryPtr) - 1;
	if (refCount == 0) {
		count = cmap->size - index;
		entries = (PALETTEENTRY *) ckalloc(sizeof(PALETTEENTRY)* count);
		GetPaletteEntries(cmap->palette, index+1, count, entries);
		SetPaletteEntries(cmap->palette, index, count, entries);
		SelectPalette(togl->tglGLHdc, cmap->palette, TRUE);
		RealizePalette(togl->tglGLHdc);
		ckfree((char *) entries);
		cmap->size--;
		Tcl_DeleteHashEntry(entryPtr);
	} else {
		Tcl_SetHashValue(entryPtr, (ClientData)refCount);
	}
}

static void Win32SetColor( const struct Togl *togl,
                    unsigned long index, float red, float green, float blue )
{
    TkWinColormap *cmap = (TkWinColormap *) Tk_Colormap(togl->TkWin);
    PALETTEENTRY entry;

    entry.peRed =   (unsigned char)(red*255 + .5);
    entry.peGreen = (unsigned char)(green*255 + .5);
    entry.peBlue =  (unsigned char)(blue*255 + .5);
    entry.peFlags = 0;
    SetPaletteEntries(cmap->palette, index, 1, &entry);
	SelectPalette(togl->tglGLHdc, cmap->palette, TRUE);
	RealizePalette(togl->tglGLHdc);

}
#endif /* X11 */



unsigned long Togl_AllocColor( const struct Togl *togl,
                               float red, float green, float blue )
{
   if (togl->RgbaFlag) {
      fprintf(stderr,"Error: Togl_AllocColor illegal in RGBA mode.\n");
      return 0;
   }
   /* TODO: maybe not... */
   if (togl->PrivateCmapFlag) {
      fprintf(stderr,"Error: Togl_FreeColor illegal with private colormap\n");
      return 0;
   }

#if defined(X11)
   {
     XColor xcol;
     int exact;

     xcol.red   = (short) (red   * 65535.0);
     xcol.green = (short) (green * 65535.0);
     xcol.blue  = (short) (blue  * 65535.0);
     
     noFaultXAllocColor( Tk_Display(togl->TkWin), Tk_Colormap(togl->TkWin),
			 Tk_Visual(togl->TkWin)->map_entries, &xcol, &exact );
     
     return xcol.pixel;
   }

#elif defined(WIN32)
   return Win32AllocColor( togl, red, green, blue );

#elif defined(macintosh)
   /* still need to implement this on Mac... */
   return 0;

#endif /* X11 */
}



void Togl_FreeColor( const struct Togl *togl, unsigned long pixel )
{
   if (togl->RgbaFlag) {
      fprintf(stderr,"Error: Togl_AllocColor illegal in RGBA mode.\n");
      return;
   }
   /* TODO: maybe not... */
   if (togl->PrivateCmapFlag) {
      fprintf(stderr,"Error: Togl_FreeColor illegal with private colormap\n");
      return;
   }

#if defined(X11)
   XFreeColors( Tk_Display(togl->TkWin), Tk_Colormap(togl->TkWin),
                &pixel, 1, 0 );
#elif defined(WIN32)
   Win32FreeColor(togl, pixel);
#endif /* X11 */
}



void Togl_SetColor( const struct Togl *togl,
                    unsigned long index, float red, float green, float blue )
{

   if (togl->RgbaFlag) {
      fprintf(stderr,"Error: Togl_AllocColor illegal in RGBA mode.\n");
      return;
   }
   if (!togl->PrivateCmapFlag) {
      fprintf(stderr,"Error: Togl_SetColor requires a private colormap\n");
      return;
   }

#if defined(X11)
   {
     XColor xcol;
     xcol.pixel = index;
     xcol.red   = (short) (red   * 65535.0);
     xcol.green = (short) (green * 65535.0);
     xcol.blue  = (short) (blue  * 65535.0);
     xcol.flags = DoRed | DoGreen | DoBlue;
     
     XStoreColor( Tk_Display(togl->TkWin), Tk_Colormap(togl->TkWin), &xcol );
   }
#elif defined(WIN32)
   Win32SetColor( togl, index, red, green, blue );
#endif /* X11 */
}


#if defined(WIN32)

/*
 * The following structure represents Windows' implementation of a font.
 */

typedef struct WinFont {
    Tk_Font font;		/* Stuff used by generic font package.  Must
				 * be first in structure. */
    HFONT hFont;		/* Windows information about font. */
    HWND hwnd;			/* Toplevel window of application that owns
				 * this font, used for getting HDC. */
    int widths[256];		/* Widths of first 256 chars in this font. */
} WinFont;
#endif /* WIN32 */


#define MAX_FONTS 1000
static GLuint ListBase[MAX_FONTS];
static GLuint ListCount[MAX_FONTS];

int Togl_BitmapFontMetrics( const struct Togl *togl, const char *fontname,
                                int *width, int *linespace) {
   XFontStruct *fontinfo;
    
   fontinfo = (XFontStruct *) XLoadQueryFont( Tk_Display(togl->TkWin), fontname );
   if (!fontinfo) {
      return 0;
   }
   *width = fontinfo->max_bounds.width;
   *linespace = fontinfo->max_bounds.ascent + fontinfo->max_bounds.descent;
   return 1;
}


/*
 * Load the named bitmap font as a sequence of bitmaps in a display list.
 * fontname may be one of the predefined fonts like TOGL_BITMAP_8_BY_13
 * or an X font name, or a Windows font name, etc.
 */
GLuint Togl_LoadBitmapFont( const struct Togl *togl, const char *fontname )
{
   static int FirstTime = 1;
#if defined(X11)
   XFontStruct *fontinfo;
#elif defined(WIN32)
   WinFont *winfont;
   HFONT oldFont;
   TEXTMETRIC tm;
#endif /* X11 */
   int first, last, count;
   GLuint fontbase;
   const char *name;

   /* Initialize the ListBase and ListCount arrays */
   if (FirstTime) {
      int i;
      for (i=0;i<MAX_FONTS;i++) {
         ListBase[i] = ListCount[i] = 0;
      }
      FirstTime = 0;
   }

   /*
    * This method of selecting X fonts according to a TOGL_ font name
    * is a kludge.  To be fixed when I find time...
    */
   if (fontname==TOGL_BITMAP_8_BY_13) {
      name = "8x13";
   }
   else if (fontname==TOGL_BITMAP_9_BY_15) {
      name = "9x15";
   }
   else if (fontname==TOGL_BITMAP_TIMES_ROMAN_10) {
      name = "-adobe-times-medium-r-normal--10-100-75-75-p-54-iso8859-1";
   }
   else if (fontname==TOGL_BITMAP_TIMES_ROMAN_24) {
      name = "-adobe-times-medium-r-normal--24-240-75-75-p-124-iso8859-1";
   }
   else if (fontname==TOGL_BITMAP_HELVETICA_10) {
      name = "-adobe-helvetica-medium-r-normal--10-100-75-75-p-57-iso8859-1";
   }
   else if (fontname==TOGL_BITMAP_HELVETICA_12) {
      name = "-adobe-helvetica-medium-r-normal--12-120-75-75-p-67-iso8859-1";
   }
   else if (fontname==TOGL_BITMAP_HELVETICA_18) {
      name = "-adobe-helvetica-medium-r-normal--18-180-75-75-p-98-iso8859-1";
   }
   else if (!fontname) {
      name = DEFAULT_FONTNAME;
   }
   else {
      name = (const char *) fontname;
   }

   assert( name );

#if defined(X11)
   fontinfo = (XFontStruct *) XLoadQueryFont( Tk_Display(togl->TkWin), name );
   if (!fontinfo) {
      return 0;
   }
   first = fontinfo->min_char_or_byte2;
   last = fontinfo->max_char_or_byte2;
#elif defined(WIN32)
   winfont = (WinFont*) Tk_GetFont(togl->Interp, togl->TkWin, name);
   if (!winfont) {
      return 0;
   }
   oldFont = SelectObject(togl->tglGLHdc, winfont->hFont);
   GetTextMetrics(togl->tglGLHdc, &tm);
   first = tm.tmFirstChar;
   last = tm.tmLastChar;
#elif defined(macintosh)
   first = 10; /* don't know how to determine font range on Mac... */
   last = 127;
#endif /* X11 */

   count = last-first+1;
   fontbase = glGenLists( (GLuint) (last+1) );
   if (fontbase==0) {
#ifdef WIN32
      SelectObject(togl->tglGLHdc, oldFont);
      Tk_FreeFont((Tk_Font) winfont);
#endif /* WIN32 */
      return 0;
   }

#if defined(WIN32)
   wglUseFontBitmaps(togl->tglGLHdc, first, count, (int) fontbase+first );
   SelectObject(togl->tglGLHdc, oldFont);
   Tk_FreeFont((Tk_Font) winfont);
#elif defined(X11)
   glXUseXFont( fontinfo->fid, first, count, (int) fontbase+first );
#elif defined(macintosh)
   aglUseFont(togl->aglCtx, 1, 0, 14, /* for now, only app font, regular 14-point */
              10, 118, fontbase+first);
#endif

   /* Record the list base and number of display lists
    * for Togl_UnloadBitmapFont().
    */
   {
      int i;
      for (i=0;i<MAX_FONTS;i++) {
         if (ListBase[i]==0) {
            ListBase[i] = fontbase;
            ListCount[i] = last+1;
            break;
         }
      }
   }

   return fontbase;
}



/*
 * Release the display lists which were generated by Togl_LoadBitmapFont().
 */
void Togl_UnloadBitmapFont( const struct Togl *togl, GLuint fontbase )
{
   int i;
   (void) togl;
   for (i=0;i<MAX_FONTS;i++) {
      if (ListBase[i]==fontbase) {
         glDeleteLists( ListBase[i], ListCount[i] );
         ListBase[i] = ListCount[i] = 0;
         return;
      }
   }
}


/*
 * Overlay functions
 */


void Togl_UseLayer( struct Togl *togl, int layer )
{
   if (togl->OverlayWindow) {
      if (layer==TOGL_OVERLAY) {
#if defined(WIN32)
         int res = wglMakeCurrent(togl->tglGLHdc, togl->tglGLOverlayHglrc);
         assert(res == TRUE);
#elif defined(X11)
	 glXMakeCurrent( Tk_Display(togl->TkWin),
			 togl->OverlayWindow,
			 togl->OverlayCtx );
#if defined(__sgi) && defined(STEREO)
	stereoMakeCurrent( Tk_Display(togl->TkWin),
			   togl->OverlayWindow,
			   togl->OverlayCtx );
#endif /* __sgi STEREO */
#endif /*WIN32 */
      }
      else if (layer==TOGL_NORMAL) {
#if defined(WIN32)
	int res = wglMakeCurrent(togl->tglGLHdc, togl->tglGLHglrc);
	assert(res == TRUE);
#elif defined(X11)
	glXMakeCurrent( Tk_Display(togl->TkWin),
			Tk_WindowId(togl->TkWin),
			togl->GlCtx );
#if defined(__sgi) && defined(STEREO)
	stereoMakeCurrent( Tk_Display(togl->TkWin),
			Tk_WindowId(togl->TkWin),
			togl->GlCtx );
#endif /* __sgi STEREO */
#endif /* WIN32 */
      }
      else {
         /* error */
      }
   }
}



#if defined(X11)  /* not yet implemented on Windows*/
void Togl_ShowOverlay( struct Togl *togl )
{
   if (togl->OverlayWindow) {
      XMapWindow( Tk_Display(togl->TkWin), togl->OverlayWindow );
      XInstallColormap(Tk_Display(togl->TkWin),togl->OverlayCmap);
      togl->OverlayIsMapped = 1;
   }
}
#endif /* X11 */



void Togl_HideOverlay( struct Togl *togl )
{
   if (togl->OverlayWindow && togl->OverlayIsMapped) {
      XUnmapWindow( Tk_Display(togl->TkWin), togl->OverlayWindow );
      togl->OverlayIsMapped=0;
   }
}



void Togl_PostOverlayRedisplay( struct Togl *togl )
{
   if (!togl->OverlayUpdatePending
       && togl->OverlayWindow && togl->OverlayDisplayProc) {
      Tk_DoWhenIdle( RenderOverlay, (ClientData) togl );
      togl->OverlayUpdatePending = 1;
   }
}


void Togl_OverlayDisplayFunc( Togl_Callback *proc )
{
   DefaultOverlayDisplayProc = proc;
}


int Togl_ExistsOverlay( const struct Togl *togl )
{
   return togl->OverlayFlag;
}


int Togl_GetOverlayTransparentValue( const struct Togl *togl )
{
   return togl->OverlayTransparentPixel;
}


int Togl_IsMappedOverlay( const struct Togl *togl )
{
   return togl->OverlayFlag && togl->OverlayIsMapped;
}


#if defined(X11) /* not yet implemented on Windows*/
unsigned long Togl_AllocColorOverlay( const struct Togl *togl,
                                      float red, float green, float blue )
{
   if (togl->OverlayFlag && togl->OverlayCmap) {
      XColor xcol;
      xcol.red   = (short) (red* 65535.0);
      xcol.green = (short) (green* 65535.0);
      xcol.blue  = (short) (blue* 65535.0);
      if (!XAllocColor(Tk_Display(togl->TkWin),togl->OverlayCmap,&xcol))
         return (unsigned long) -1;
      return xcol.pixel;
   }
   else {
      return (unsigned long) -1;
   }
}


void Togl_FreeColorOverlay( const struct Togl *togl, unsigned long pixel )
{

   if (togl->OverlayFlag && togl->OverlayCmap) {
      XFreeColors( Tk_Display(togl->TkWin), togl->OverlayCmap,
                   &pixel, 1, 0 );
   }
}
#endif /* X11 */



/*
 * User client data
 */

void Togl_ClientData( ClientData clientData )
{
   DefaultClientData = clientData;
}


ClientData Togl_GetClientData( const struct Togl *togl )
{
   return togl->Client_Data;
}


void Togl_SetClientData( struct Togl *togl, ClientData clientData )
{
   togl->Client_Data = clientData;
}



#ifdef MESA_COLOR_HACK
/*
 * Let's know how many free colors do we have
 */
#if 0
static unsigned char rojo[] = { 4, 39, 74, 110, 145, 181, 216, 251},
                     verde[] = { 4, 39, 74, 110, 145, 181, 216, 251},
		     azul[] = { 4, 39, 74, 110, 145, 181, 216, 251};

unsigned char rojo[] = { 4, 36, 72, 109, 145, 182, 218, 251},
              verde[] = { 4, 36, 72, 109, 145, 182, 218, 251},
              azul[] = { 4, 36, 72, 109, 145, 182, 218, 251};
              azul[] = { 0, 85, 170, 255};
#endif

#define RLEVELS     5
#define GLEVELS     9
#define BLEVELS     5

/* to free dithered_rgb_colormap pixels allocated by Mesa */
static unsigned long *ToglMesaUsedPixelCells = NULL;
static int ToglMesaUsedFreeCells = 0;

static int get_free_color_cells( Display *display, int screen,
                                 Colormap colormap)
{
   if ( !ToglMesaUsedPixelCells) {
      XColor xcol;
      int i;
      int colorsfailed, ncolors = XDisplayCells( display, screen);

      long r, g, b;

      ToglMesaUsedPixelCells = ( unsigned long *)calloc( ncolors, sizeof( unsigned long));

      /* Allocate X colors and initialize color_table[], red_table[], etc */
      /* de Mesa 2.1: xmesa1.c setup_dithered_(...) */
      i = colorsfailed = 0;
      for (r = 0; r < RLEVELS; r++)
         for (g = 0; g < GLEVELS; g++)
            for (b = 0; b < BLEVELS; b++) {
               int exact;
               xcol.red   = ( r*65535)/(RLEVELS-1);
               xcol.green = ( g*65535)/(GLEVELS-1);
               xcol.blue  = ( b*65535)/(BLEVELS-1);
               noFaultXAllocColor( display, colormap, ncolors,
                                   &xcol, &exact );
               ToglMesaUsedPixelCells[ i++] = xcol.pixel;
               if (!exact) {
                  colorsfailed++;
               }
            }
      ToglMesaUsedFreeCells = i;

      XFreeColors( display, colormap, ToglMesaUsedPixelCells,
                   ToglMesaUsedFreeCells, 0x00000000);
   }
   return ToglMesaUsedFreeCells;
}


static void free_default_color_cells( Display *display, Colormap colormap)
{
   if ( ToglMesaUsedPixelCells) {
      XFreeColors( display, colormap, ToglMesaUsedPixelCells,
                   ToglMesaUsedFreeCells, 0x00000000);
      free( ( char *)ToglMesaUsedPixelCells);
      ToglMesaUsedPixelCells = NULL;
      ToglMesaUsedFreeCells = 0;
   }
}
#endif


#if defined(__sgi) && defined(STEREO)

static struct stereoStateRec {
    Bool        useSGIStereo;
    Display     *currentDisplay;
    Window      currentWindow;
    GLXContext  currentContext;
    GLenum      currentDrawBuffer;
    int         currentStereoBuffer;
    Bool        enabled;
    char        *stereoCommand;
    char        *restoreCommand;
} stereo;

/* call instead of glDrawBuffer */
void
Togl_StereoDrawBuffer(GLenum mode)
{
  if (stereo.useSGIStereo) {
    stereo.currentDrawBuffer = mode;
    switch (mode) {
    case GL_FRONT:
    case GL_BACK:
    case GL_FRONT_AND_BACK:
      /*
      ** Simultaneous drawing to both left and right buffers isn't
      ** really possible if we don't have a stereo capable visual.
      ** For now just fall through and use the left buffer.
      */
    case GL_LEFT:
    case GL_FRONT_LEFT:
    case GL_BACK_LEFT:
      stereo.currentStereoBuffer = STEREO_BUFFER_LEFT;
      break;
    case GL_RIGHT:
    case GL_FRONT_RIGHT:
      stereo.currentStereoBuffer = STEREO_BUFFER_RIGHT;
      mode = GL_FRONT;
      break;
    case GL_BACK_RIGHT:
      stereo.currentStereoBuffer = STEREO_BUFFER_RIGHT;
      mode = GL_BACK;
      break;
    default:
      break;
    }
    if (stereo.currentDisplay && stereo.currentWindow) {
      glXWaitGL();  /* sync with GL command stream before calling X */
      XSGISetStereoBuffer(stereo.currentDisplay,
			  stereo.currentWindow,
			  stereo.currentStereoBuffer);
      glXWaitX();   /* sync with X command stream before calling GL */
    }
  }
  glDrawBuffer(mode);
}

/* call instead of glClear */
void
Togl_StereoClear(GLbitfield mask)
{
  GLenum drawBuffer;
  if (stereo.useSGIStereo) {
    drawBuffer = stereo.currentDrawBuffer;
    switch (drawBuffer) {
    case GL_FRONT:
      stereoDrawBuffer(GL_FRONT_RIGHT);
      glClear(mask);
      stereoDrawBuffer(drawBuffer);
      break;
    case GL_BACK:
      stereoDrawBuffer(GL_BACK_RIGHT);
      glClear(mask);
      stereoDrawBuffer(drawBuffer);
      break;
    case GL_FRONT_AND_BACK:
      stereoDrawBuffer(GL_RIGHT);
      glClear(mask);
      stereoDrawBuffer(drawBuffer);
      break;
    case GL_LEFT:
    case GL_FRONT_LEFT:
    case GL_BACK_LEFT:
    case GL_RIGHT:
    case GL_FRONT_RIGHT:
    case GL_BACK_RIGHT:
    default:
      break;
    }
  }
  glClear(mask);
}

static void
stereoMakeCurrent(Display *dpy, Window win, GLXContext ctx)
{

  if (stereo.useSGIStereo) {
    if (dpy && (dpy != stereo.currentDisplay)) {
      int event, error;
      /* Make sure new Display supports SGIStereo */
      if (XSGIStereoQueryExtension(dpy, &event, &error) == False) {
	dpy = NULL;
      }
    }
    if (dpy && win && (win != stereo.currentWindow)) {
      /* Make sure new Window supports SGIStereo */
      if (XSGIQueryStereoMode(dpy, win) == X_STEREO_UNSUPPORTED) {
	win = None;
      }
    }
    if (ctx && (ctx != stereo.currentContext)) {
      GLint drawBuffer;
      glGetIntegerv(GL_DRAW_BUFFER, &drawBuffer);
      stereoDrawBuffer((GLenum) drawBuffer);
    }
    stereo.currentDisplay = dpy;
    stereo.currentWindow = win;
    stereo.currentContext = ctx;
  }
}


/* call before using stereo */
static void
stereoInit(struct Togl *togl,int stereoEnabled)
{
  stereo.useSGIStereo = stereoEnabled;
  stereo.currentDisplay = NULL;
  stereo.currentWindow = None;
  stereo.currentContext = NULL;
  stereo.currentDrawBuffer = GL_NONE;
  stereo.currentStereoBuffer = STEREO_BUFFER_NONE;
  stereo.enabled = False;
}


void
Togl_StereoFrustum(GLfloat left, GLfloat right, GLfloat bottom, GLfloat top,
               GLfloat near, GLfloat far, GLfloat eyeDist, GLfloat eyeOffset)
{
  GLfloat eyeShift = (eyeDist - near) * (eyeOffset / eyeDist);

  glFrustum(left+eyeShift, right+eyeShift, bottom, top, near, far);
  glTranslatef(-eyeShift, 0.0, 0.0);
}

#endif /* __sgi STEREO */


#ifdef macintosh
/* needed to make shared library on Mac with CodeWarrior; should be overridden by user app */
/*
int main(int argc, char *argv[])
{
  return -1;
}
*/

/* the following code is borrowed from tkMacAppInit.c */

/*
 *----------------------------------------------------------------------
 *
 * MacintoshInit --
 *
 *	This procedure calls Mac specific initialization calls.  Most of
 *	these calls must be made as soon as possible in the startup
 *	process.
 *
 * Results:
 *	Returns TCL_OK if everything went fine.  If it didn't the
 *	application should probably fail.
 *
 * Side effects:
 *	Inits the application.
 *
 *----------------------------------------------------------------------
 */

int Togl_MacInit(void)
{
   int i;
   long result, mask = 0x0700;  /* mask = system 7.x */

#if GENERATING68K && !GENERATINGCFM
   SetApplLimit(GetApplLimit() - (TK_MAC_68K_STACK_GROWTH));
#endif
   MaxApplZone();
   for (i = 0; i < 4; i++) {
      (void) MoreMasters();
   }

   /*
    * Tk needs us to set the qd pointer it uses.  This is needed
    * so Tk doesn't have to assume the availability of the qd global
    * variable.  Which in turn allows Tk to be used in code resources.
    */
   tcl_macQdPtr = &qd;

   /*
    * If appearance is present, then register Tk as an Appearance client
    * This means that the mapping from non-Appearance to Appearance cdefs
    * will be done for Tk regardless of the setting in the Appearance
    * control panel.
    */
   if (TkMacHaveAppearance()) {
      RegisterAppearanceClient();
   }

   InitGraf(&tcl_macQdPtr->thePort);
   InitFonts();
   InitWindows();
   InitMenus();
   InitDialogs((long) NULL);
   InitCursor();

   /*
    * Make sure we are running on system 7 or higher
    */
   if ((NGetTrapAddress(_Gestalt, ToolTrap) ==
        NGetTrapAddress(_Unimplemented, ToolTrap))
       || (((Gestalt(gestaltSystemVersion, &result) != noErr)
            || (result < mask)))) {
      panic("Tcl/Tk requires System 7 or higher.");
   }

   /*
    * Make sure we have color quick draw
    * (this means we can't run on 68000 macs)
    */
   if (((Gestalt(gestaltQuickdrawVersion, &result) != noErr)
        || (result < gestalt32BitQD13))) {
      panic("Tk requires Color QuickDraw.");
   }

   FlushEvents(everyEvent, 0);
   SetEventMask(everyEvent);

   Tcl_MacSetEventProc(TkMacConvertEvent);
   return TCL_OK;
}

int Togl_MacSetupMainInterp(Tcl_Interp * interp)
{
   TkMacInitAppleEvents(interp);
   TkMacInitMenus(interp);
   return TCL_OK;
}

#endif                          /* macintosh */
