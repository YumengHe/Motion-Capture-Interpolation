//
// X11 header file for the Fast Light Tool Kit (FLTK).
//
// Copyright 1998-2024 by Bill Spitzak and others.
//
// This library is free software. Distribution and use rights are outlined in
// the file "COPYING" which should have been included with this file.  If this
// file is missing or damaged, see the license at:
//
//     http://www.fltk.org/COPYING.php
//
// Please report all bugs and problems on the following page:
//
//     http://www.fltk.org/str.php
//

// These are internal fltk symbols that are necessary or useful for
// calling Xlib.  You should include this file if (and ONLY if) you
// need to call Xlib directly.  These symbols may not exist on non-X
// systems.

#if !defined(Fl_X_H) && !defined(FL_DOXYGEN) 
#  define Fl_X_H

#  include "Enumerations.H"

#  ifdef WIN32
#    include "win32.H"
#  elif defined(__APPLE__)
#    include "mac.H"
#  else
#    if defined(_ABIN32) || defined(_ABI64) // fix for broken SGI Irix X .h files
#      pragma set woff 3322
#    endif
#    include <X11/Xlib.h>
#    include <X11/Xutil.h>
#    if defined(_ABIN32) || defined(_ABI64)
#      pragma reset woff 3322
#    endif
#    include <X11/Xatom.h>
#    include "Fl_Window.H"
// Mirror X definition of Region to Fl_Region, for portability...
typedef Region Fl_Region;

FL_EXPORT void fl_open_display();
FL_EXPORT void fl_open_display(Display*);
FL_EXPORT void fl_close_display();

// constant info about the X server connection:
extern FL_EXPORT Display *fl_display;
extern FL_EXPORT int fl_screen;
extern FL_EXPORT XVisualInfo *fl_visual;
extern FL_EXPORT Colormap fl_colormap;


// drawing functions:
extern FL_EXPORT GC fl_gc;
extern FL_EXPORT Window fl_window;
FL_EXPORT ulong fl_xpixel(Fl_Color i);
FL_EXPORT ulong fl_xpixel(uchar r, uchar g, uchar b);
FL_EXPORT void fl_clip_region(Fl_Region);
FL_EXPORT Fl_Region fl_clip_region();

// feed events into fltk:
FL_EXPORT int fl_handle(const XEvent&);

// you can use these in Fl::add_handler() to look at events:
extern FL_EXPORT const XEvent* fl_xevent;
extern FL_EXPORT ulong fl_event_time;

// off-screen pixmaps: create, destroy, draw into, copy to window:
typedef ulong Fl_Offscreen;
#    define fl_create_offscreen(w,h) XCreatePixmap(fl_display, RootWindow(fl_display, fl_screen), w, h, fl_visual->depth)
#    define fl_create_offscreen_with_alpha(w,h) XCreatePixmap(fl_display, RootWindow(fl_display, fl_screen), w, h, 32)
// begin/end are macros that save the old state in local variables:
#    define fl_begin_offscreen(pixmap) \
  Window _sw=fl_window; fl_window=pixmap; \
  GC _sgc = fl_gc; if (!_sgc) fl_gc = XCreateGC(fl_display, pixmap, 0, 0); \
  Fl_Surface_Device *_ss = Fl_Surface_Device::surface(); Fl_Display_Device::display_device()->set_current(); \
  fl_push_no_clip()
#    define fl_end_offscreen() \
  fl_pop_clip(); fl_window = _sw; _ss->set_current(); \
  if (!_sgc) XFreeGC(fl_display, fl_gc); \
    fl_gc = _sgc

extern FL_EXPORT void fl_copy_offscreen(int x, int y, int w, int h, Fl_Offscreen pixmap, int srcx, int srcy);
#    define fl_delete_offscreen(pixmap) XFreePixmap(fl_display, pixmap)

// Bitmap masks
typedef ulong Fl_Bitmask;

extern FL_EXPORT Fl_Bitmask fl_create_bitmask(int w, int h, const uchar *data);
extern FL_EXPORT Fl_Bitmask fl_create_alphamask(int w, int h, int d, int ld, const uchar *data);
extern FL_EXPORT void fl_delete_bitmask(Fl_Bitmask bm);

#if defined(FL_LIBRARY) || defined(FL_INTERNALS)
extern FL_EXPORT Window fl_message_window;
extern FL_EXPORT void *fl_xftfont;
FL_EXPORT Fl_Region XRectangleRegion(int x, int y, int w, int h); // in fl_rect.cxx

// access to core fonts:
// This class provides a "smart pointer" that returns a pointer to an XFontStruct.
// The global variable fl_xfont can be called wherever a bitmap "core" font is
// needed, e.g. when rendering to a GL context under X11.
// With Xlib / X11 fonts, fl_xfont will return the current selected font.
// With XFT / X11 fonts, fl_xfont will attempt to return the bitmap "core" font most
// similar to (usually the same as) the current XFT font.
class FL_EXPORT Fl_XFont_On_Demand
{
public:
  Fl_XFont_On_Demand(XFontStruct* p = NULL) : ptr(p) { }
  Fl_XFont_On_Demand& operator=(const Fl_XFont_On_Demand& x)
  { ptr = x.ptr;  return *this; }
  Fl_XFont_On_Demand& operator=(XFontStruct* p)
  { ptr = p;  return *this; }
  XFontStruct* value();
  operator XFontStruct*() { return value(); }
  XFontStruct& operator*() { return *value(); }
  XFontStruct* operator->() { return value(); }
  bool operator==(const Fl_XFont_On_Demand& x) { return ptr == x.ptr; }
  bool operator!=(const Fl_XFont_On_Demand& x) { return ptr != x.ptr; }
private:
  XFontStruct *ptr;
};
extern FL_EXPORT Fl_XFont_On_Demand fl_xfont;
extern FL_EXPORT XFontStruct* fl_X_core_font();

// this object contains all X-specific stuff about a window:
// Warning: this object is highly subject to change!  
// FL_LIBRARY or FL_INTERNALS must be defined to access this class.
class FL_EXPORT Fl_X {
public:
  Window xid;
  Window other_xid;
  Fl_Window *w;
  Fl_Region region;
  Fl_X *next;
  char wait_for_expose;
  char backbuffer_bad; // used for XDBE
  static Fl_X* first;
  static Fl_X* i(const Fl_Window* wi) {return wi->i;}
  void setwindow(Fl_Window* wi) {w=wi; wi->i=this;}
  void sendxjunk();
  static void set_default_icons(const Fl_RGB_Image*[], int);
  void set_icons();
  int set_cursor(Fl_Cursor);
  int set_cursor(const Fl_RGB_Image*, int, int);
  static void make_xid(Fl_Window*,XVisualInfo* =fl_visual, Colormap=fl_colormap);
  static Fl_X* set_xid(Fl_Window*, Window);
  // kludges to get around protection:
  void flush() {w->flush();}
  static void x(Fl_Window* wi, int X) {wi->x(X);}
  static void y(Fl_Window* wi, int Y) {wi->y(Y);}
  static int ewmh_supported();
  static int xrender_supported();
  static void activate_window(Window w);
};

extern FL_EXPORT char fl_override_redirect; // hack into Fl_X::make_xid()
extern FL_EXPORT int fl_background_pixel;  // hack into Fl_X::make_xid()

inline Window fl_xid(const Fl_Window* w) { Fl_X *xTemp = Fl_X::i(w); return xTemp ? xTemp->xid : 0; }

#else

extern FL_EXPORT Window fl_xid_(const Fl_Window* w);
#define fl_xid(w) fl_xid_(w)

#endif // FL_LIBRARY || FL_INTERNALS

FL_EXPORT Fl_Window* fl_find(Window xid);


// Dummy function to register a function for opening files via the window manager...
inline void fl_open_callback(void (*)(const char *)) {}

extern FL_EXPORT int fl_parse_color(const char* p, uchar& r, uchar& g, uchar& b);

#  endif
#endif
