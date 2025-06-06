//
// "$Id$"
//
// WIN32-specific code for the Fast Light Tool Kit (FLTK).
//
// Copyright 1998-2018 by Bill Spitzak and others.
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

// This file contains win32-specific code for fltk which is always linked
// in.  Search other files for "WIN32" or filenames ending in _win32.cxx
// for other system-specific code.

// This file must be #include'd in Fl.cxx and not compiled separately.

#ifndef FL_DOXYGEN
#include <FL/Fl.H>
#include <FL/fl_utf8.h>
#include <FL/Fl_Window.H>
#include <FL/fl_draw.H>
#include <FL/Enumerations.H>
#include <FL/Fl_Tooltip.H>
#include <FL/Fl_Paged_Device.H>
#include "flstring.h"
#include "Fl_Font.H"
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <time.h>
#include <signal.h>
#ifdef __CYGWIN__
#  include <sys/time.h>
#  include <unistd.h>
#endif

#if !defined(NO_TRACK_MOUSE)
#  include <commctrl.h>	// TrackMouseEvent
// fabien: Ms Visual Studio >= 2003 permit embedded lib reference
// that makes fltk use easier as only fltk libs are now requested
// This idea could be extended to fltk libs themselves, 
// implementer should then care about DLL linkage flags ...
#  if defined(_MSC_VER) && (_MSC_VER>=1310)
#    pragma comment (lib, "comctl32.lib")
#  endif
#endif

#if defined(__GNUC__)
# include <wchar.h>
#endif

#include <ole2.h>
#include <shellapi.h>

// New versions of MinGW (as of Feb 2018) need to include winerror.h to
// #define S_OK which used to be defined in ole2.h (STR #3454)

#include <winerror.h>

// old versions of MinGW lack definition of GET_XBUTTON_WPARAM:

#ifndef GET_XBUTTON_WPARAM
#define GET_XBUTTON_WPARAM(wParam) (HIWORD(wParam))
#endif

//
// USE_ASYNC_SELECT - define it if you have WSAAsyncSelect()...
// USE_ASYNC_SELECT is OBSOLETED in 1.3 for the following reasons:
/*
  This feature was supposed to provide an efficient alternative to the current
  polling method, but as it has been discussed (Thanks Albrecht!) :
  - the async mode would imply to change the socket to non blocking mode.
    This can have unexpected side effects for 3rd party apps, especially
    if it is set on-the-fly when socket service is really needed, as it is 
    done today and on purpose, but still the 3rd party developer wouldn't easily
    control the sequencing of socket operations.
  - Finer granularity of events furthered by the async select is a plus only 
    for socket 3rd party impl., it is simply not needed for the 'light' fltk
    use we make of wsock, so here it would also be a bad point, because of all
    the logic add-ons necessary for using this functionality, without a clear
    benefit.

  So async mode select would not add benefits to fltk, worse, it can slowdown
  fltk because of this finer granularity and instrumentation code to be added
  for async mode proper operation, not mentioning the side effects...
*/

// Internal functions
static void fl_clipboard_notify_target(HWND wnd);
static void fl_clipboard_notify_untarget(HWND wnd);

// Internal variables
static HWND clipboard_wnd = 0;
static HWND next_clipboard_wnd = 0;

static bool initial_clipboard = true;

// dynamic wsock dll handling api:
#if defined(__CYGWIN__) && !defined(SOCKET)
# define SOCKET int
#endif

// note: winsock2.h has been #include'd in Fl.cxx
#define WSCK_DLL_NAME "WS2_32.DLL"

// Patch for MinGW (__MINGW32__): see STR #3454 and src/Fl.cxx
#ifdef __MINGW32__
typedef int(WINAPI *fl_wsk_fd_is_set_f)(unsigned int, void *);
#else
typedef int(WINAPI *fl_wsk_fd_is_set_f)(SOCKET, fd_set *);
static fl_wsk_fd_is_set_f fl_wsk_fd_is_set = 0;
#endif

typedef int (WINAPI* fl_wsk_select_f)(int, fd_set*, fd_set*, fd_set*, const struct timeval*);

static HMODULE s_wsock_mod = 0;
static fl_wsk_select_f s_wsock_select = 0;

#ifdef __MINGW32__
static void * get_wsock_mod() {
#else
static HMODULE get_wsock_mod() {
#endif
  if (!s_wsock_mod) {
    s_wsock_mod = LoadLibrary(WSCK_DLL_NAME);
    if (s_wsock_mod==NULL)
      Fl::fatal("FLTK Lib Error: %s file not found! Please check your winsock dll accessibility.\n",WSCK_DLL_NAME);
    s_wsock_select = (fl_wsk_select_f) GetProcAddress(s_wsock_mod, "select");
    fl_wsk_fd_is_set = (fl_wsk_fd_is_set_f) GetProcAddress(s_wsock_mod, "__WSAFDIsSet");
  }
  return s_wsock_mod;
}

/*
 * Dynamic linking of imm32.dll
 * This library is only needed for a hand full (four ATM) functions relating to 
 * international text rendering and locales. Dynamically loading reduces initial
 * size and link dependencies.
 */
static HMODULE s_imm_module = 0;
typedef BOOL (WINAPI* flTypeImmAssociateContextEx)(HWND, HIMC, DWORD);
static flTypeImmAssociateContextEx flImmAssociateContextEx = 0;
typedef HIMC (WINAPI* flTypeImmGetContext)(HWND);
static flTypeImmGetContext flImmGetContext = 0;
typedef BOOL (WINAPI* flTypeImmSetCompositionWindow)(HIMC, LPCOMPOSITIONFORM);
static flTypeImmSetCompositionWindow flImmSetCompositionWindow = 0;
typedef BOOL (WINAPI* flTypeImmReleaseContext)(HWND, HIMC);
static flTypeImmReleaseContext flImmReleaseContext = 0;

static void get_imm_module() {
  s_imm_module = LoadLibrary("IMM32.DLL");
  if (!s_imm_module)
    Fl::fatal("FLTK Lib Error: IMM32.DLL file not found!\n\n"
      "Please check your input method manager library accessibility.");
  flImmAssociateContextEx = (flTypeImmAssociateContextEx)GetProcAddress(s_imm_module, "ImmAssociateContextEx");
  flImmGetContext = (flTypeImmGetContext)GetProcAddress(s_imm_module, "ImmGetContext");
  flImmSetCompositionWindow = (flTypeImmSetCompositionWindow)GetProcAddress(s_imm_module, "ImmSetCompositionWindow");
  flImmReleaseContext = (flTypeImmReleaseContext)GetProcAddress(s_imm_module, "ImmReleaseContext");
}

// USE_TRACK_MOUSE - define NO_TRACK_MOUSE if you don't have
// TrackMouseEvent()...
//
// Now (Dec. 2008) we can assume that current Cygwin/MinGW versions
// support the TrackMouseEvent() function, but WinCE obviously doesn't
// support it (STR 2095). Therefore, USE_TRACK_MOUSE is enabled by 
// default, but you can disable it by defining NO_TRACK_MOUSE.
//
// TrackMouseEvent is only used to support window leave notifications
// under Windows. It can be used to get FL_LEAVE events, when the
// mouse leaves the _main_ application window (FLTK detects subwindow
// leave events by using normal move events).
//
// Implementation note: If the mouse cursor leaves one subwindow and
// enters another window, then Windows sends a WM_MOUSEMOVE message to
// the new window before it sends a WM_MOUSELEAVE message to the old
// (just left) window. We save the current mouse window in a static variable,
// and if we get a WM_MOUSELEAVE event for the current mouse window, this
// means that the top level window has been left (otherwise we would have
// got another WM_MOUSEMOVE message before).

// #define NO_TRACK_MOUSE

#if !defined(NO_TRACK_MOUSE)
# define USE_TRACK_MOUSE
#endif // NO_TRACK_MOUSE

static Fl_Window *track_mouse_win=0;	// current TrackMouseEvent() window

// USE_CAPTURE_MOUSE_WIN - this must be defined for TrackMouseEvent to work
// correctly with subwindows - otherwise a single mouse click and release
// (without a move) would generate phantom leave events.
// This defines, if the current mouse window (maybe a subwindow) or the 
// main window should get mouse events after pushing (and holding) a mouse
// button, i.e. when dragging the mouse. This is done by calling SetCapture
// (see below).

#ifdef USE_TRACK_MOUSE
#define USE_CAPTURE_MOUSE_WIN
#endif // USE_TRACK_MOUSE

//
// WM_SYNCPAINT is an "undocumented" message, which is finally defined in
// VC++ 6.0.
//

#ifndef WM_SYNCPAINT
#  define WM_SYNCPAINT 0x0088
#endif

#ifndef WM_MOUSELEAVE
#  define WM_MOUSELEAVE 0x02a3
#endif

#ifndef WM_MOUSEWHEEL
#  define WM_MOUSEWHEEL 0x020a
#endif

#ifndef WHEEL_DELTA
#  define WHEEL_DELTA 120	// according to MSDN.
#endif

#ifndef SM_CXPADDEDBORDER
#  define SM_CXPADDEDBORDER (92) // STR #3061
#endif

//
// WM_FLSELECT is the user-defined message that we get when one of
// the sockets has pending data, etc.
//

#define WM_FLSELECT	(WM_APP+1)	// WM_APP is used for hide-window


////////////////////////////////////////////////////////////////
// interface to poll/select call:

// fd's are only implemented for sockets.  Microsoft Windows does not
// have a unified IO system, so it doesn't support select() on files,
// devices, or pipes...
//
// Microsoft provides the Berkeley select() call and an asynchronous
// select function that sends a WIN32 message when the select condition
// exists. However, we don't try to use the asynchronous WSAAsyncSelect()
// any more for good reasons (see above).
//
// A.S. Dec 2009: We got reports that current winsock2.h files define
// POLLIN, POLLOUT, and POLLERR with conflicting values WRT what we
// used before (STR #2301).  Therefore we must not use these values
// for our internal purposes, but use FL_READ, FL_WRITE, and
// FL_EXCEPT, as defined for use in Fl::add_fd().
//
static int maxfd = 0;
static fd_set fdsets[3];

extern IDropTarget *flIDropTarget;

static int nfds = 0;
static int fd_array_size = 0;
static struct FD {
  int fd;
  short events;
  void (*cb)(FL_SOCKET, void*); // keep socket api opaque at this level to reduce multiplatform deps headaches
  void* arg;
} *fd = 0;

extern unsigned int fl_codepage;

void fl_reset_spot()
{
}

void fl_set_spot(int font, int size, int X, int Y, int W, int H, Fl_Window *win)
{
  if (!win) return;
  Fl_Window* tw = win;
  while (tw->parent()) tw = tw->window(); // find top level window

  if (!tw->shown())
    return;

  HIMC himc = flImmGetContext(fl_xid(tw));

  if (himc) {
    COMPOSITIONFORM cfs;
    cfs.dwStyle = CFS_POINT;
    cfs.ptCurrentPos.x = X;
    cfs.ptCurrentPos.y = Y - tw->labelsize();
    MapWindowPoints(fl_xid(win), fl_xid(tw), &cfs.ptCurrentPos, 1);
    flImmSetCompositionWindow(himc, &cfs);
    flImmReleaseContext(fl_xid(tw), himc);
  }
}

void fl_set_status(int x, int y, int w, int h)
{
}

void Fl::add_fd(int n, int events, void (*cb)(FL_SOCKET, void*), void *v) {
  remove_fd(n,events);
  int i = nfds++;
  if (i >= fd_array_size) {
    fd_array_size = 2*fd_array_size+1;
    fd = (FD*)realloc(fd, fd_array_size*sizeof(FD));
  }
  fd[i].fd = n;
  fd[i].events = (short)events;
  fd[i].cb = cb;
  fd[i].arg = v;

  if (events & FL_READ) FD_SET((unsigned)n, &fdsets[0]);
  if (events & FL_WRITE) FD_SET((unsigned)n, &fdsets[1]);
  if (events & FL_EXCEPT) FD_SET((unsigned)n, &fdsets[2]);
  if (n > maxfd) maxfd = n;
}

void Fl::add_fd(int fd, void (*cb)(FL_SOCKET, void*), void* v) {
  Fl::add_fd(fd, FL_READ, cb, v);
}

void Fl::remove_fd(int n, int events) {
  int i,j;
  for (i=j=0; i<nfds; i++) {
    if (fd[i].fd == n) {
      short e = fd[i].events & ~events;
      if (!e) continue; // if no events left, delete this fd
      fd[i].events = e;
    }
    // move it down in the array if necessary:
    if (j<i) {
      fd[j]=fd[i];
    }
    j++;
  }
  nfds = j;

  if (events & FL_READ) FD_CLR(unsigned(n), &fdsets[0]);
  if (events & FL_WRITE) FD_CLR(unsigned(n), &fdsets[1]);
  if (events & FL_EXCEPT) FD_CLR(unsigned(n), &fdsets[2]);
}

void Fl::remove_fd(int n) {
  remove_fd(n, -1);
}

// these pointers are set by the Fl::lock() function:
static void nothing() {}
void (*fl_lock_function)() = nothing;
void (*fl_unlock_function)() = nothing;

static void* thread_message_;
void* Fl::thread_message() {
  void* r = thread_message_;
  thread_message_ = 0;
  return r;
}

extern int fl_send_system_handlers(void *e);

MSG fl_msg;

// A local helper function to flush any pending callback requests
// from the awake ring-buffer
static void process_awake_handler_requests(void) {
  Fl_Awake_Handler func;
  void *data;
  while (Fl::get_awake_handler_(func, data) == 0) {
    func(data);
  }
}

// This is never called with time_to_wait < 0.0.
// It *should* return negative on error, 0 if nothing happens before
// timeout, and >0 if any callbacks were done.  This version only
// returns zero if nothing happens during a 0.0 timeout, otherwise
// it returns 1.
int fl_wait(double time_to_wait) {
  int have_message = 0;

  run_checks();

  // idle processing
  static char in_idle;
  if (Fl::idle && !in_idle) {
    in_idle = 1;
    Fl::idle();
    in_idle = 0;
  }
  
  if (nfds) {
    // For WIN32 we need to poll for socket input FIRST, since
    // the event queue is not something we can select() on...
    timeval t;
    t.tv_sec = 0;
    t.tv_usec = 0;

    fd_set fdt[3];
    memcpy(fdt, fdsets, sizeof fdt); // one shot faster fdt init
    if (get_wsock_mod()&& s_wsock_select(maxfd+1,&fdt[0],&fdt[1],&fdt[2],&t)) {
      // We got something - do the callback!
      for (int i = 0; i < nfds; i ++) {
	SOCKET f = fd[i].fd;
	short revents = 0;
	if (fl_wsk_fd_is_set(f, &fdt[0])) revents |= FL_READ;
	if (fl_wsk_fd_is_set(f, &fdt[1])) revents |= FL_WRITE;
	if (fl_wsk_fd_is_set(f, &fdt[2])) revents |= FL_EXCEPT;
	if (fd[i].events & revents) fd[i].cb(f, fd[i].arg);
      }
      time_to_wait = 0.0; // just peek for any messages
    } else {
      // we need to check them periodically, so set a short timeout:
      if (time_to_wait > .001) time_to_wait = .001;
    }
  }

  if (Fl::idle || Fl::damage()) 
    time_to_wait = 0.0;

  // if there are no more windows and this timer is set
  // to FOREVER, continue through or look up indefinitely
  if (!Fl::first_window() && time_to_wait==1e20)
    time_to_wait = 0.0;

  fl_unlock_function();

  time_to_wait = (time_to_wait > 10000 ? 10000 : time_to_wait);
  int t_msec = (int) (time_to_wait * 1000.0 + 0.5);
  MsgWaitForMultipleObjects(0, NULL, FALSE, t_msec, QS_ALLINPUT);

  fl_lock_function();

  // Execute the message we got, and all other pending messages:
  // have_message = PeekMessage(&fl_msg, NULL, 0, 0, PM_REMOVE);
  while ((have_message = PeekMessageW(&fl_msg, NULL, 0, 0, PM_REMOVE)) > 0) {
    if (fl_send_system_handlers(&fl_msg))
      continue;

    // Let applications treat WM_QUIT identical to SIGTERM on *nix
    if (fl_msg.message == WM_QUIT)
      raise(SIGTERM);

    if (fl_msg.message == fl_wake_msg) {
      // Used for awaking wait() from another thread
      thread_message_ = (void*)fl_msg.wParam;
      process_awake_handler_requests();
    }

    TranslateMessage(&fl_msg);
    DispatchMessageW(&fl_msg);
  }

  // The following conditional test:
  //    (Fl::awake_ring_head_ != Fl::awake_ring_tail_)
  // is a workaround / fix for STR #3143. This works, but a better solution
  // would be to understand why the PostThreadMessage() messages are not
  // seen by the main window if it is being dragged/ resized at the time.
  // If a worker thread posts an awake callback to the ring buffer
  // whilst the main window is unresponsive (if a drag or resize operation
  // is in progress) we may miss the PostThreadMessage(). So here, we check if
  // there is anything pending in the awake ring buffer and if so process
  // it. This is not strictly thread safe (for speed it compares the head
  // and tail indices without first locking the ring buffer) but is intended
  // only as a fall-back recovery mechanism if the awake processing stalls.
  // If the test erroneously returns true (may happen if we test the indices
  // whilst they are being modified) we will call process_awake_handler_requests()
  // unnecessarily, but this has no harmful consequences so is safe to do.
  // Note also that if we miss the PostThreadMessage(), then thread_message_
  // will not be updated, so this is not a perfect solution, but it does
  // recover and process any pending awake callbacks.
  // Normally the ring buffer head and tail indices will match and this
  // comparison will do nothing. Addresses STR #3143
  if (Fl::awake_ring_head_ != Fl::awake_ring_tail_) {
    process_awake_handler_requests();
  }

  Fl::flush();

  // This should return 0 if only timer events were handled:
  return 1;
}

// fl_ready() is just like fl_wait(0.0) except no callbacks are done:
int fl_ready() {
  if (PeekMessage(&fl_msg, NULL, 0, 0, PM_NOREMOVE)) return 1;
  if (!nfds) return 0;
  timeval t;
  t.tv_sec = 0;
  t.tv_usec = 0;
  fd_set fdt[3];
  memcpy(fdt, fdsets, sizeof fdt);
  return get_wsock_mod() ? s_wsock_select(0,&fdt[0],&fdt[1],&fdt[2],&t) : 0;
}

void fl_open_display() {
  static char beenHereDoneThat = 0;

  if (beenHereDoneThat)
    return;

  beenHereDoneThat = 1;

  OleInitialize(0L);

  get_imm_module();
}

class Fl_Win32_At_Exit {
public:
  Fl_Win32_At_Exit() { }
  ~Fl_Win32_At_Exit() {
    fl_free_fonts();        // do some WIN32 cleanup
    fl_cleanup_pens();
    OleUninitialize();
    if (fl_gc) fl_brush_action(1);
    fl_cleanup_dc_list();
    // This is actually too late in the cleanup process to remove the
    // clipboard notifications, but we have no earlier hook so we try
    // to work around it anyway.
    if (clipboard_wnd != NULL)
      fl_clipboard_notify_untarget(clipboard_wnd);
  }
};
static Fl_Win32_At_Exit win32_at_exit;

static char im_enabled = 1;

void Fl::enable_im() {
  fl_open_display();

  Fl_X* i = Fl_X::first;
  while (i) {
    flImmAssociateContextEx(i->xid, 0, IACE_DEFAULT);
    i = i->next;
  }

  im_enabled = 1;
}

void Fl::disable_im() {
  fl_open_display();

  Fl_X* i = Fl_X::first;
  while (i) {
    flImmAssociateContextEx(i->xid, 0, 0);
    i = i->next;
  }

  im_enabled = 0;
}

////////////////////////////////////////////////////////////////

int Fl::x()
{
  RECT r;

  SystemParametersInfo(SPI_GETWORKAREA, 0, &r, 0);
  return r.left;
}

int Fl::y()
{
  RECT r;

  SystemParametersInfo(SPI_GETWORKAREA, 0, &r, 0);
  return r.top;
}

int Fl::h()
{
  RECT r;

  SystemParametersInfo(SPI_GETWORKAREA, 0, &r, 0);
  return r.bottom - r.top;
}

int Fl::w()
{
  RECT r;

  SystemParametersInfo(SPI_GETWORKAREA, 0, &r, 0);
  return r.right - r.left;
}

void Fl::get_mouse(int &x, int &y) {
  POINT p;
  GetCursorPos(&p);
  x = p.x;
  y = p.y;
}

////////////////////////////////////////////////////////////////
// code used for selections:

char *fl_selection_buffer[2];
int fl_selection_length[2];
int fl_selection_buffer_length[2];
char fl_i_own_selection[2];

UINT fl_get_lcid_codepage(LCID id)
{
  char buf[8];
  buf[GetLocaleInfo(id, LOCALE_IDEFAULTANSICODEPAGE, buf, 8)] = 0;
  return atol(buf);
}

// Convert \n -> \r\n
class Lf2CrlfConvert {
  char *out;
  int outlen;
public:
  Lf2CrlfConvert(const char *in, int inlen) {
    outlen = 0;
    const char *i;
    char *o;
    int lencount;
    // Predict size of \r\n conversion buffer
    for (i = in, lencount = inlen; lencount > 0; lencount--) {
      if ( *i == '\r' && *(i+1) == '\n' && lencount >= 2 )	// leave \r\n untranslated
	{ i+=2; outlen+=2; lencount--; }
      else if ( *i == '\n' )			// \n by itself? leave room to insert \r
	{ i++; outlen+=2; }
      else
	{ ++i; ++outlen; }
    }
    // Alloc conversion buffer + NULL
    out = new char[outlen+1];
    // Handle \n -> \r\n conversion
    for (i = in, o=out, lencount = inlen; lencount > 0; lencount--) {
      if ( *i == '\r' && *(i+1) == '\n' && lencount >= 2 )	// leave \r\n untranslated
        { *o++ = *i++; *o++ = *i++; lencount--; }
      else if ( *i == '\n' )			// \n by itself? insert \r
        { *o++ = '\r'; *o++ = *i++; }
      else
        { *o++ = *i++; }
    }
    *o++ = 0;
  }
  ~Lf2CrlfConvert() {
    delete[] out;
  }
  int GetLength() const { return(outlen); }
  const char* GetValue() const { return(out); }
};

void fl_update_clipboard(void) {
  Fl_Window *w1 = Fl::first_window();
  if (!w1)
    return;

  HWND hwnd = fl_xid(w1);

  if (!OpenClipboard(hwnd))
    return;

  EmptyClipboard();

  int utf16_len = fl_utf8toUtf16(fl_selection_buffer[1],
                                 fl_selection_length[1], 0, 0);

  HGLOBAL hMem = GlobalAlloc(GHND, utf16_len * 2 + 2); // moveable and zero'ed mem alloc.
  LPVOID memLock = GlobalLock(hMem);

  fl_utf8toUtf16(fl_selection_buffer[1], fl_selection_length[1],
                 (unsigned short*) memLock, utf16_len + 1);

  GlobalUnlock(hMem);
  SetClipboardData(CF_UNICODETEXT, hMem);

  CloseClipboard();

  // In case Windows managed to lob of a WM_DESTROYCLIPBOARD during
  // the above.
  fl_i_own_selection[1] = 1;
}

// call this when you create a selection:
void Fl::copy(const char *stuff, int len, int clipboard, const char *type) {
  if (!stuff || len<0) return;
  if (clipboard >= 2)
    clipboard = 1; // Only on X11 do multiple clipboards make sense.

  // Convert \n -> \r\n (for old apps like Notepad, DOS)
  Lf2CrlfConvert buf(stuff, len);
  len = buf.GetLength();
  stuff = buf.GetValue();

  if (len+1 > fl_selection_buffer_length[clipboard]) {
    delete[] fl_selection_buffer[clipboard];
    fl_selection_buffer[clipboard] = new char[len+100];
    fl_selection_buffer_length[clipboard] = len+100;
  }
  memcpy(fl_selection_buffer[clipboard], stuff, len);
  fl_selection_buffer[clipboard][len] = 0; // needed for direct paste
  fl_selection_length[clipboard] = len;
  fl_i_own_selection[clipboard] = 1;
  if (clipboard)
    fl_update_clipboard();
}

// Call this when a "paste" operation happens:
void Fl::paste(Fl_Widget &receiver, int clipboard, const char *type) {
  if (!clipboard || (fl_i_own_selection[clipboard] && strcmp(type, Fl::clipboard_plain_text) == 0)) {
    // We already have it, do it quickly without window server.
    // Notice that the text is clobbered if set_selection is
    // called in response to FL_PASTE!
    char *i = fl_selection_buffer[clipboard];
    if (i==0L) {
      Fl::e_text = 0; 
      return;
    }
    char *clip_text = new char[fl_selection_length[clipboard]+1];
    char *o = clip_text;
    while (*i) { // Convert \r\n -> \n
      if ( *i == '\r' && *(i+1) == '\n') i++;
      else *o++ = *i++;
    }
    *o = 0;
    Fl::e_text = clip_text;
    Fl::e_length = (int) (o - Fl::e_text);
    Fl::e_clipboard_type = Fl::clipboard_plain_text;
    receiver.handle(FL_PASTE);
    delete [] clip_text;
    Fl::e_text = 0;
  } else if (clipboard) {
    HANDLE h;
    if (!OpenClipboard(NULL)) return;
    if (strcmp(type, Fl::clipboard_plain_text) == 0) { // we want plain text from clipboard
      if ((h = GetClipboardData(CF_UNICODETEXT))) { // there's text in the clipboard
	wchar_t *memLock = (wchar_t*) GlobalLock(h);
	size_t utf16_len = wcslen(memLock);
	char *clip_text = new char[utf16_len * 4 + 1];
	unsigned utf8_len = fl_utf8fromwc(clip_text, (unsigned) (utf16_len * 4), memLock, (unsigned) utf16_len);
	*(clip_text + utf8_len) = 0;
	GlobalUnlock(h);
	LPSTR a,b;
	a = b = clip_text;
	while (*a) { // strip the CRLF pairs ($%$#@^)
	  if (*a == '\r' && a[1] == '\n') a++;
	  else *b++ = *a++;
	}
	*b = 0;
        Fl::e_text = clip_text;
	Fl::e_length = (int) (b - Fl::e_text);
	Fl::e_clipboard_type = Fl::clipboard_plain_text;  // indicates that the paste event is for plain UTF8 text
	receiver.handle(FL_PASTE); // send the FL_PASTE event to the widget
	delete[] clip_text;
	Fl::e_text = 0;
	}
      }
      else if (strcmp(type, Fl::clipboard_image) == 0) { // we want an image from clipboard
	uchar *rgb = NULL;
	int width = 0, height = 0, depth = 0;
	if ( (h = GetClipboardData(CF_DIB)) ) { // if there's a DIB in clipboard
	  LPBITMAPINFO lpBI = (LPBITMAPINFO)GlobalLock(h) ;
	  width = lpBI->bmiHeader.biWidth; // bitmap width & height
	  height = lpBI->bmiHeader.biHeight;
	  if ( (lpBI->bmiHeader.biBitCount == 24 || lpBI->bmiHeader.biBitCount == 32) && 
	      lpBI->bmiHeader.biCompression == BI_RGB &&
	      lpBI->bmiHeader.biClrUsed == 0) { // direct use of the DIB data if it's RGB or RGBA
	    int linewidth; // row length
	    depth = lpBI->bmiHeader.biBitCount/8; // 3 or 4
	    if (depth == 3) linewidth = 4 * ((3*width + 3)/4); // row length: series of groups of 3 bytes, rounded to multiple of 4 bytes
	    else linewidth = 4*width;
	    rgb = new uchar[width * height * depth]; // will hold the image data
	    uchar *p = rgb, *r, rr, gg, bb;
	    for (int i=height-1; i>=0; i--) { // for each row, from last to first
	      r = (uchar*)(lpBI->bmiColors) + i*linewidth; // beginning of pixel data for the ith row
	      for (int j=0; j<width; j++) { // for each pixel in a row
		bb = *r++; // BGR is in DIB
		gg = *r++;
		rr = *r++;
		*p++ = rr; // we want RGB
		*p++ = gg;
		*p++ = bb;
		if (depth == 4) *p++ = *r++; // copy alpha if present
	      }
	    }
	  }
	  else { // the system will decode a complex DIB
	    void *pDIBBits = (void*)(lpBI->bmiColors + 256); 
	    if (lpBI->bmiHeader.biCompression == BI_BITFIELDS) pDIBBits = (void*)(lpBI->bmiColors + 3);
	    else if (lpBI->bmiHeader.biClrUsed > 0) pDIBBits = (void*)(lpBI->bmiColors + lpBI->bmiHeader.biClrUsed);
	    Fl_Offscreen off = fl_create_offscreen(width, height);
	    fl_begin_offscreen(off);
	    SetDIBitsToDevice(fl_gc, 0, 0, width, height, 0, 0, 0, height, pDIBBits, lpBI, DIB_RGB_COLORS);
	    rgb = fl_read_image(NULL, 0, 0, width, height);
	    depth = 3;
	    fl_end_offscreen();
	    fl_delete_offscreen(off);
	  }
	  GlobalUnlock(h);
	}
	else if ((h = GetClipboardData(CF_ENHMETAFILE))) { // if there's an enhanced metafile in clipboard
	  ENHMETAHEADER header;
	  GetEnhMetaFileHeader((HENHMETAFILE)h, sizeof(header), &header); // get structure containing metafile dimensions
	  width = (header.rclFrame.right - header.rclFrame.left + 1); // in .01 mm units
	  height = (header.rclFrame.bottom - header.rclFrame.top + 1);
	  HDC hdc = GetDC(NULL); // get unit correspondance between .01 mm and screen pixels
	  int hmm = GetDeviceCaps(hdc, HORZSIZE);
	  int hdots = GetDeviceCaps(hdc, HORZRES);
          int dhr = GetDeviceCaps(hdc, DESKTOPHORZRES); // true number of pixels on display
	  ReleaseDC(NULL, hdc);
          // Global display scaling factor: 1, 1.25, 1.5, 1.75, etc...
          float scaling = dhr/float(hdots);
	  float factor = (100.f * hmm) / hdots;
	  width = (int)(width*scaling/factor); height = (int)(height*scaling/factor); // convert to screen pixel unit
	  RECT rect = {0, 0, width, height};
	  Fl_Offscreen off = fl_create_offscreen(width, height);
	  fl_begin_offscreen(off);
	  fl_color(FL_WHITE); fl_rectf(0,0,width, height); // draw white background
	  PlayEnhMetaFile(fl_gc, (HENHMETAFILE)h, &rect); // draw metafile to offscreen buffer
	  rgb = fl_read_image(NULL, 0, 0, width, height); // read pixels from offscreen buffer
	  depth = 3;
	  fl_end_offscreen();
	  fl_delete_offscreen(off);
	}
	if (rgb) {
	  Fl_RGB_Image *image = new Fl_RGB_Image(rgb, width, height, depth); // create new image from pixel data
	  image->alloc_array = 1;
	  Fl::e_clipboard_data = image;
	  Fl::e_clipboard_type = Fl::clipboard_image;  // indicates that the paste event is for image data
	  int done = receiver.handle(FL_PASTE); // send FL_PASTE event to widget
	  Fl::e_clipboard_type = "";
	  if (done == 0) { // if widget did not handle the event, delete the image
	    Fl::e_clipboard_data = NULL;
	    delete image;
	  }
	}
      }
     CloseClipboard();
    }
}

int Fl::clipboard_contains(const char *type)
{
  int retval = 0;
  if (!OpenClipboard(NULL)) return 0;
  if (strcmp(type, Fl::clipboard_plain_text) == 0 || type[0] == 0) {
    retval = IsClipboardFormatAvailable(CF_UNICODETEXT);
  }
  else if (strcmp(type, Fl::clipboard_image) == 0) {
    retval = IsClipboardFormatAvailable(CF_DIB) || IsClipboardFormatAvailable(CF_ENHMETAFILE);
  }
  CloseClipboard();
  return retval;
}

static void fl_clipboard_notify_target(HWND wnd) {
  if (clipboard_wnd)
    return;

  // We get one fake WM_DRAWCLIPBOARD immediately, which we therefore
  // need to ignore.
  initial_clipboard = true;

  clipboard_wnd = wnd;
  next_clipboard_wnd = SetClipboardViewer(wnd);
}

static void fl_clipboard_notify_untarget(HWND wnd) {
  if (wnd != clipboard_wnd)
    return;

  // We might be called late in the cleanup where Windows has already
  // implicitly destroyed our clipboard window. At that point we need
  // to do some extra work to manually repair the clipboard chain.
  if (IsWindow(wnd))
    ChangeClipboardChain(wnd, next_clipboard_wnd);
  else {
    HWND tmp, head;

    tmp = CreateWindow("STATIC", "Temporary FLTK Clipboard Window", 0,
                       0, 0, 0, 0, HWND_MESSAGE, NULL, NULL, NULL);
    if (tmp == NULL)
      return;

    head = SetClipboardViewer(tmp);
    if (head == NULL)
      ChangeClipboardChain(tmp, next_clipboard_wnd);
    else {
      SendMessage(head, WM_CHANGECBCHAIN, (WPARAM)wnd, (LPARAM)next_clipboard_wnd);
      ChangeClipboardChain(tmp, head);
    }

    DestroyWindow(tmp);
  }

  clipboard_wnd = next_clipboard_wnd = 0;
}

void fl_clipboard_notify_retarget(HWND wnd) {
  // The given window is getting destroyed. If it's part of the
  // clipboard chain then we need to unregister it and find a
  // replacement window.
  if (wnd != clipboard_wnd)
    return;

  fl_clipboard_notify_untarget(wnd);

  if (Fl::first_window())
    fl_clipboard_notify_target(fl_xid(Fl::first_window()));
}

void fl_clipboard_notify_change() {
  // untarget clipboard monitor if no handlers are registered
  if (clipboard_wnd != NULL && fl_clipboard_notify_empty()) {
    fl_clipboard_notify_untarget(clipboard_wnd);
    return;
  }

  // if there are clipboard notify handlers but no window targeted
  // target first window if available
  if (clipboard_wnd == NULL && Fl::first_window())
    fl_clipboard_notify_target(fl_xid(Fl::first_window()));
}

////////////////////////////////////////////////////////////////
void fl_get_codepage()
{
  HKL hkl = GetKeyboardLayout(0);
  TCHAR ld[8];

  GetLocaleInfo (LOWORD(hkl), LOCALE_IDEFAULTANSICODEPAGE, ld, 6);
  DWORD ccp = atol(ld);
  fl_codepage = ccp;
}

HWND fl_capture;

static int mouse_event(Fl_Window *window, int what, int button,
		       WPARAM wParam, LPARAM lParam)
{
  static int px, py, pmx, pmy;
  POINT pt;
  Fl::e_x = pt.x = (signed short)LOWORD(lParam);
  Fl::e_y = pt.y = (signed short)HIWORD(lParam);
  ClientToScreen(fl_xid(window), &pt);
  Fl::e_x_root = pt.x;
  Fl::e_y_root = pt.y;
#ifdef USE_CAPTURE_MOUSE_WIN
  Fl_Window *mouse_window = window;	// save "mouse window"
#endif
  while (window->parent()) {
    Fl::e_x += window->x();
    Fl::e_y += window->y();
    window = window->window();
  }

  ulong state = Fl::e_state & 0xff0000; // keep shift key states
#if 0
  // mouse event reports some shift flags, perhaps save them?
  if (wParam & MK_SHIFT) state |= FL_SHIFT;
  if (wParam & MK_CONTROL) state |= FL_CTRL;
#endif
  if (wParam & MK_LBUTTON)  state |= FL_BUTTON1;  // left
  if (wParam & MK_MBUTTON)  state |= FL_BUTTON2;  // right
  if (wParam & MK_RBUTTON)  state |= FL_BUTTON3;  // middle
  if (wParam & MK_XBUTTON1) state |= FL_BUTTON4;  // side button 1 (back)
  if (wParam & MK_XBUTTON2) state |= FL_BUTTON5;  // side button 2 (forward)

  Fl::e_state = state;

  switch (what) {
  case 1: // double-click
    if (Fl::e_is_click) {Fl::e_clicks++; goto J1;}
  case 0: // single-click
    Fl::e_clicks = 0;
  J1:
#ifdef USE_CAPTURE_MOUSE_WIN
    if (!fl_capture) SetCapture(fl_xid(mouse_window));  // use mouse window
#else
    if (!fl_capture) SetCapture(fl_xid(window));	// use main window
#endif
    Fl::e_keysym = FL_Button + button;
    Fl::e_is_click = 1;
    px = pmx = Fl::e_x_root; py = pmy = Fl::e_y_root;
    return Fl::handle(FL_PUSH,window);

  case 2: // release:
    if (!fl_capture) ReleaseCapture();
    Fl::e_keysym = FL_Button + button;
    return Fl::handle(FL_RELEASE,window);

  case 3: // move:
  default: // avoid compiler warning
    // MSWindows produces extra events even if mouse does not move, ignore em:
    if (Fl::e_x_root == pmx && Fl::e_y_root == pmy) return 1;
    pmx = Fl::e_x_root; pmy = Fl::e_y_root;
    if (abs(Fl::e_x_root-px)>5 || abs(Fl::e_y_root-py)>5) Fl::e_is_click = 0;
    return Fl::handle(FL_MOVE,window);

  }
}

// convert a MSWindows VK_x to an Fltk (X) Keysym:
// See also the inverse converter in Fl_get_key_win32.cxx
// This table is in numeric order by VK:
static const struct {unsigned short vk, fltk, extended;} vktab[] = {
  {VK_BACK,	FL_BackSpace},
  {VK_TAB,	FL_Tab},
  {VK_CLEAR,	FL_KP+'5',	0xff0b/*XK_Clear*/},
  {VK_RETURN,	FL_Enter,	FL_KP_Enter},
  {VK_SHIFT,	FL_Shift_L,	FL_Shift_R},
  {VK_CONTROL,	FL_Control_L,	FL_Control_R},
  {VK_MENU,	FL_Alt_L,	FL_Alt_R},
  {VK_PAUSE,	FL_Pause},
  {VK_CAPITAL,	FL_Caps_Lock},
  {VK_ESCAPE,	FL_Escape},
  {VK_SPACE,	' '},
  {VK_PRIOR,	FL_KP+'9',	FL_Page_Up},
  {VK_NEXT,	FL_KP+'3',	FL_Page_Down},
  {VK_END,	FL_KP+'1',	FL_End},
  {VK_HOME,	FL_KP+'7',	FL_Home},
  {VK_LEFT,	FL_KP+'4',	FL_Left},
  {VK_UP,	FL_KP+'8',	FL_Up},
  {VK_RIGHT,	FL_KP+'6',	FL_Right},
  {VK_DOWN,	FL_KP+'2',	FL_Down},
  {VK_SNAPSHOT,	FL_Print},	// does not work on NT
  {VK_INSERT,	FL_KP+'0',	FL_Insert},
  {VK_DELETE,	FL_KP+'.',	FL_Delete},
  {VK_LWIN,	FL_Meta_L},
  {VK_RWIN,	FL_Meta_R},
  {VK_APPS,	FL_Menu},
  {VK_SLEEP, FL_Sleep},
  {VK_MULTIPLY,	FL_KP+'*'},
  {VK_ADD,	FL_KP+'+'},
  {VK_SUBTRACT,	FL_KP+'-'},
  {VK_DECIMAL,	FL_KP+'.'},
  {VK_DIVIDE,	FL_KP+'/'},
  {VK_NUMLOCK,	FL_Num_Lock},
  {VK_SCROLL,	FL_Scroll_Lock},
# if defined(_WIN32_WINNT) && (_WIN32_WINNT >= 0x0500)
  {VK_BROWSER_BACK, FL_Back},
  {VK_BROWSER_FORWARD, FL_Forward},
  {VK_BROWSER_REFRESH, FL_Refresh},
  {VK_BROWSER_STOP, FL_Stop},
  {VK_BROWSER_SEARCH, FL_Search},
  {VK_BROWSER_FAVORITES, FL_Favorites},
  {VK_BROWSER_HOME, FL_Home_Page},
  {VK_VOLUME_MUTE, FL_Volume_Mute},
  {VK_VOLUME_DOWN, FL_Volume_Down},
  {VK_VOLUME_UP, FL_Volume_Up},
  {VK_MEDIA_NEXT_TRACK, FL_Media_Next},
  {VK_MEDIA_PREV_TRACK, FL_Media_Prev},
  {VK_MEDIA_STOP, FL_Media_Stop},
  {VK_MEDIA_PLAY_PAUSE, FL_Media_Play},
  {VK_LAUNCH_MAIL, FL_Mail},
#endif
  {0xba,	';'},
  {0xbb,	'='},
  {0xbc,	','},
  {0xbd,	'-'},
  {0xbe,	'.'},
  {0xbf,	'/'},
  {0xc0,	'`'},
  {0xdb,	'['},
  {0xdc,	'\\'},
  {0xdd,	']'},
  {0xde,	'\''},
  {VK_OEM_102,	FL_Iso_Key}
};
static int ms2fltk(WPARAM vk, int extended) {
  static unsigned short vklut[256];
  static unsigned short extendedlut[256];
  if (!vklut[1]) { // init the table
    unsigned int i;
    for (i = 0; i < 256; i++) vklut[i] = tolower(i);
    for (i=VK_F1; i<=VK_F16; i++) vklut[i] = i+(FL_F-(VK_F1-1));
    for (i=VK_NUMPAD0; i<=VK_NUMPAD9; i++) vklut[i] = i+(FL_KP+'0'-VK_NUMPAD0);
    for (i = 0; i < sizeof(vktab)/sizeof(*vktab); i++) {
      vklut[vktab[i].vk] = vktab[i].fltk;
      extendedlut[vktab[i].vk] = vktab[i].extended;
    }
    for (i = 0; i < 256; i++) if (!extendedlut[i]) extendedlut[i] = vklut[i];
  }
  return extended ? extendedlut[vk] : vklut[vk];
}

#if USE_COLORMAP
extern HPALETTE fl_select_palette(void); // in fl_color_win32.cxx
#endif


/////////////////////////////////////////////////////////////////////////////
/// Win32 timers
///

struct Win32Timer
{
  UINT_PTR handle;
  Fl_Timeout_Handler callback;
  void *data;
};
static Win32Timer* win32_timers;
static int win32_timer_alloc;
static int win32_timer_used;
static HWND s_TimerWnd;

static void realloc_timers()
{
  if (win32_timer_alloc == 0) {
    win32_timer_alloc = 8;
  }
  win32_timer_alloc *= 2;
  Win32Timer* new_timers = new Win32Timer[win32_timer_alloc];
  memset(new_timers, 0, sizeof(Win32Timer) * win32_timer_used);
  memcpy(new_timers, win32_timers, sizeof(Win32Timer) * win32_timer_used);
  Win32Timer* delete_me = win32_timers;
  win32_timers = new_timers;
  delete [] delete_me;
}

static void delete_timer(Win32Timer& t)
{
  KillTimer(s_TimerWnd, t.handle);
  memset(&t, 0, sizeof(Win32Timer));
}

/// END TIMERS
/////////////////////////////////////////////////////////////////////////////

static Fl_Window* resize_bug_fix;

extern void fl_save_pen(void);
extern void fl_restore_pen(void);

static LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
  // Copy the message to fl_msg so add_handler code can see it, it is
  // already there if this is called by DispatchMessage, but not if
  // Windows calls this directly.
  fl_msg.hwnd = hWnd;
  fl_msg.message = uMsg;
  fl_msg.wParam = wParam;
  fl_msg.lParam = lParam;
  //fl_msg.time = ???
  //fl_msg.pt = ???
  //fl_msg.lPrivate = ???

  Fl_Window *window = fl_find(hWnd);

  if (window) switch (uMsg) {

  case WM_QUIT: // this should not happen?
    Fl::fatal("WM_QUIT message");

  case WM_CLOSE: // user clicked close box
    Fl::handle(FL_CLOSE, window);
    return 0;

  case WM_SYNCPAINT :
  case WM_NCPAINT :
  case WM_ERASEBKGND :
    // Andreas Weitl - WM_SYNCPAINT needs to be passed to DefWindowProc
    // so that Windows can generate the proper paint messages...
    // Similarly, WM_NCPAINT and WM_ERASEBKGND need this, too...
    break;

  case WM_PAINT: {
    Fl_Region R;
    Fl_X *i = Fl_X::i(window);
    i->wait_for_expose = 0;
    char redraw_whole_window = false;
    if (!i->region && window->damage()) {
      // Redraw the whole window...
      i->region = CreateRectRgn(0, 0, window->w(), window->h());
      redraw_whole_window = true;
    }

    // We need to merge WIN32's damage into FLTK's damage.
    R = CreateRectRgn(0,0,0,0);
    int r = GetUpdateRgn(hWnd,R,0);
    if (r==NULLREGION && !redraw_whole_window) {
      XDestroyRegion(R);
      break;
    }

    if (i->region) {
      // Also tell WIN32 that we are drawing someplace else as well...
      CombineRgn(i->region, i->region, R, RGN_OR);
      XDestroyRegion(R);
    } else {
      i->region = R;
    }
    if (window->type() == FL_DOUBLE_WINDOW) ValidateRgn(hWnd,0);
    else ValidateRgn(hWnd,i->region);

    window->clear_damage((uchar)(window->damage()|FL_DAMAGE_EXPOSE));
    // These next two statements should not be here, so that all update
    // is deferred until Fl::flush() is called during idle.  However WIN32
    // apparently is very unhappy if we don't obey it and draw right now.
    // Very annoying!
    fl_GetDC(hWnd); // Make sure we have a DC for this window...
    fl_save_pen();
    i->flush();
    fl_restore_pen();
    window->clear_damage();
    } return 0;

  case WM_LBUTTONDOWN:  mouse_event(window, 0, 1, wParam, lParam); return 0;
  case WM_LBUTTONDBLCLK:mouse_event(window, 1, 1, wParam, lParam); return 0;
  case WM_LBUTTONUP:    mouse_event(window, 2, 1, wParam, lParam); return 0;
  case WM_MBUTTONDOWN:  mouse_event(window, 0, 2, wParam, lParam); return 0;
  case WM_MBUTTONDBLCLK:mouse_event(window, 1, 2, wParam, lParam); return 0;
  case WM_MBUTTONUP:    mouse_event(window, 2, 2, wParam, lParam); return 0;
  case WM_RBUTTONDOWN:  mouse_event(window, 0, 3, wParam, lParam); return 0;
  case WM_RBUTTONDBLCLK:mouse_event(window, 1, 3, wParam, lParam); return 0;
  case WM_RBUTTONUP:    mouse_event(window, 2, 3, wParam, lParam); return 0;
  case WM_XBUTTONDOWN: {
    int xbutton = GET_XBUTTON_WPARAM(wParam) == XBUTTON1 ? 4 : 5;
    mouse_event(window, 0, xbutton, wParam, lParam);
    return 0;
  }
  case WM_XBUTTONDBLCLK: {
    int xbutton = GET_XBUTTON_WPARAM(wParam) == XBUTTON1 ? 4 : 5;
    mouse_event(window, 1, xbutton, wParam, lParam);
    return 0;
  }
  case WM_XBUTTONUP: {
    int xbutton = GET_XBUTTON_WPARAM(wParam) == XBUTTON1 ? 4 : 5;
    mouse_event(window, 2, xbutton, wParam, lParam);
    return 0;
  }
  case WM_MOUSEMOVE:
#ifdef USE_TRACK_MOUSE
    if (track_mouse_win != window) {
      TRACKMOUSEEVENT tme;
      tme.cbSize    = sizeof(TRACKMOUSEEVENT);
      tme.dwFlags   = TME_LEAVE;
      tme.hwndTrack = hWnd;
      _TrackMouseEvent(&tme);
      track_mouse_win = window;
    }
#endif // USE_TRACK_MOUSE
    mouse_event(window, 3, 0, wParam, lParam);
    return 0;

  case WM_MOUSELEAVE:
    if (track_mouse_win == window) { // we left the top level window !
      Fl_Window *tw = window;
      while (tw->parent()) tw = tw->window(); // find top level window
      Fl::belowmouse(0);
      Fl::handle(FL_LEAVE, tw);
    }
    track_mouse_win = 0; // force TrackMouseEvent() restart
    break;

  case WM_SETFOCUS:
    if ((Fl::modal_) && (Fl::modal_ != window)) {
      SetFocus(fl_xid(Fl::modal_));
      return 0;
    }
    Fl::handle(FL_FOCUS, window);
    break;

  case WM_KILLFOCUS:
    Fl::handle(FL_UNFOCUS, window);
    Fl::flush(); // it never returns to main loop when deactivated...
    break;

  case WM_SHOWWINDOW:
    if (!window->parent()) {
      Fl::handle(wParam ? FL_SHOW : FL_HIDE, window);
    }
    break;

  case WM_ACTIVATEAPP:
    // From eric@vfx.sel.sony.com, we should process WM_ACTIVATEAPP
    // messages to restore the correct state of the shift/ctrl/alt/lock
    // keys...  Added control, shift, alt, and meta keys, and changed
    // to use GetAsyncKeyState and do it when wParam is 1
    // (that means we have focus...)
    if (wParam)
    {
      ulong state = 0;
      if (GetAsyncKeyState(VK_CAPITAL)) state |= FL_CAPS_LOCK;
      if (GetAsyncKeyState(VK_NUMLOCK)) state |= FL_NUM_LOCK;
      if (GetAsyncKeyState(VK_SCROLL)) state |= FL_SCROLL_LOCK;
      if (GetAsyncKeyState(VK_CONTROL)&~1) state |= FL_CTRL;
      if (GetAsyncKeyState(VK_SHIFT)&~1) state |= FL_SHIFT;
      if (GetAsyncKeyState(VK_MENU)) state |= FL_ALT;
      if ((GetAsyncKeyState(VK_LWIN)|GetAsyncKeyState(VK_RWIN))&~1) state |= FL_META;
      Fl::e_state = state;
      return 0;
    }
    break;

  case WM_INPUTLANGCHANGE:
    fl_get_codepage();
    break;
  case WM_IME_COMPOSITION:
//	if (!fl_is_nt4() && lParam & GCS_RESULTCLAUSE) {
//		HIMC himc = ImmGetContext(hWnd);
//		wlen = ImmGetCompositionStringW(himc, GCS_RESULTSTR,
//			wbuf, sizeof(wbuf)) / sizeof(short);
//		if (wlen < 0) wlen = 0;
//		wbuf[wlen] = 0;
//		ImmReleaseContext(hWnd, himc);
//	}
	break;
  case WM_KEYDOWN:
  case WM_SYSKEYDOWN:
  case WM_KEYUP:
  case WM_SYSKEYUP:
    // save the keysym until we figure out the characters:
    Fl::e_keysym = Fl::e_original_keysym = ms2fltk(wParam,lParam&(1<<24));
    // See if TranslateMessage turned it into a WM_*CHAR message:
    if (PeekMessageW(&fl_msg, hWnd, WM_CHAR, WM_SYSDEADCHAR, PM_REMOVE))
    {
      uMsg = fl_msg.message;
      wParam = fl_msg.wParam;
      lParam = fl_msg.lParam;
    }
  case WM_DEADCHAR:
  case WM_SYSDEADCHAR:
  case WM_CHAR:
  case WM_SYSCHAR: {
    ulong state = Fl::e_state & 0xff000000; // keep the mouse button state
    // if GetKeyState is expensive we might want to comment some of these out:
    if (GetKeyState(VK_SHIFT)&~1) state |= FL_SHIFT;
    if (GetKeyState(VK_CAPITAL)) state |= FL_CAPS_LOCK;
    if (GetKeyState(VK_CONTROL)&~1) state |= FL_CTRL;
    // Alt gets reported for the Alt-GR switch on foreign keyboards.
    // so we need to check the event as well to get it right:
    if ((lParam&(1<<29)) //same as GetKeyState(VK_MENU)
	&& uMsg != WM_CHAR) state |= FL_ALT;
    if (GetKeyState(VK_NUMLOCK)) state |= FL_NUM_LOCK;
    if ((GetKeyState(VK_LWIN)|GetKeyState(VK_RWIN))&~1) {
      // WIN32 bug?  GetKeyState returns garbage if the user hit the
      // meta key to pop up start menu.  Sigh.
      if ((GetAsyncKeyState(VK_LWIN)|GetAsyncKeyState(VK_RWIN))&~1)
	state |= FL_META;
    }
    if (GetKeyState(VK_SCROLL)) state |= FL_SCROLL_LOCK;
    Fl::e_state = state;
    static char buffer[1024];
    if (uMsg == WM_CHAR || uMsg == WM_SYSCHAR) {

      xchar u = (xchar) wParam;
//    Fl::e_length = fl_unicode2utf(&u, 1, buffer);
      Fl::e_length = fl_utf8fromwc(buffer, 1024, &u, 1);
      buffer[Fl::e_length] = 0;


    } else if (Fl::e_keysym >= FL_KP && Fl::e_keysym <= FL_KP_Last) {
      if (state & FL_NUM_LOCK) {
        // Convert to regular keypress...
	buffer[0] = Fl::e_keysym-FL_KP;
	Fl::e_length = 1;
      } else {
        // Convert to special keypress...
	buffer[0] = 0;
	Fl::e_length = 0;
	switch (Fl::e_keysym) {
	  case FL_KP + '0' :
	    Fl::e_keysym = FL_Insert;
	    break;
	  case FL_KP + '1' :
	    Fl::e_keysym = FL_End;
	    break;
	  case FL_KP + '2' :
	    Fl::e_keysym = FL_Down;
	    break;
	  case FL_KP + '3' :
	    Fl::e_keysym = FL_Page_Down;
	    break;
	  case FL_KP + '4' :
	    Fl::e_keysym = FL_Left;
	    break;
	  case FL_KP + '6' :
	    Fl::e_keysym = FL_Right;
	    break;
	  case FL_KP + '7' :
	    Fl::e_keysym = FL_Home;
	    break;
	  case FL_KP + '8' :
	    Fl::e_keysym = FL_Up;
	    break;
	  case FL_KP + '9' :
	    Fl::e_keysym = FL_Page_Up;
	    break;
	  case FL_KP + '.' :
	    Fl::e_keysym = FL_Delete;
	    break;
	  case FL_KP + '/' :
	  case FL_KP + '*' :
	  case FL_KP + '-' :
	  case FL_KP + '+' :
	    buffer[0] = Fl::e_keysym-FL_KP;
	    Fl::e_length = 1;
	    break;
	}
      }
    } else if ((lParam & (1<<31))==0) {
#ifdef FLTK_PREVIEW_DEAD_KEYS
      if ((lParam & (1<<24))==0) { // clear if dead key (always?)
        xchar u = (xchar) wParam;
        Fl::e_length = fl_utf8fromwc(buffer, 1024, &u, 1);
        buffer[Fl::e_length] = 0;
      } else { // set if "extended key" (never printable?)
        buffer[0] = 0;
        Fl::e_length = 0;
      }
#else
      buffer[0] = 0;
      Fl::e_length = 0;
#endif
    }
    Fl::e_text = buffer;
    if (lParam & (1<<31)) { // key up events.
      if (Fl::handle(FL_KEYUP, window)) return 0;
      break;
    }
    // for (int i = lParam&0xff; i--;)
    while (window->parent()) window = window->window();
    if (Fl::handle(FL_KEYBOARD,window)) {
	  if (uMsg==WM_DEADCHAR || uMsg==WM_SYSDEADCHAR)
		Fl::compose_state = 1;
	  return 0;
	}
    break;}

  case WM_MOUSEWHEEL: {
    static int delta = 0; // running total of all motion
    delta += (SHORT)(HIWORD(wParam));
    Fl::e_dx = 0;
    Fl::e_dy = -delta / WHEEL_DELTA;
    delta += Fl::e_dy * WHEEL_DELTA;
    if (Fl::e_dy) Fl::handle(FL_MOUSEWHEEL, window);
    return 0;
  }

// This is only defined on Vista and upwards...
#ifndef WM_MOUSEHWHEEL
#define WM_MOUSEHWHEEL 0x020E
#endif
      
  case WM_MOUSEHWHEEL: {
      static int delta = 0; // running total of all motion
      delta += (SHORT)(HIWORD(wParam));
      Fl::e_dy = 0;
      Fl::e_dx = delta / WHEEL_DELTA;
      delta -= Fl::e_dx * WHEEL_DELTA;
      if (Fl::e_dx) Fl::handle(FL_MOUSEWHEEL, window);
      return 0;
    }

  case WM_GETMINMAXINFO:
    Fl_X::i(window)->set_minmax((LPMINMAXINFO)lParam);
    break;

  case WM_SIZE:
    if (!window->parent()) {
      if (wParam == SIZE_MINIMIZED || wParam == SIZE_MAXHIDE) {
	Fl::handle(FL_HIDE, window);
      } else {
	Fl::handle(FL_SHOW, window);
	resize_bug_fix = window;
	window->size(LOWORD(lParam), HIWORD(lParam));
      }
    }
    break;

  case WM_MOVE: {
    resize_bug_fix = window;
    int nx = LOWORD(lParam);
    int ny = HIWORD(lParam);
    if (nx & 0x8000) nx -= 65536;
    if (ny & 0x8000) ny -= 65536;
    window->position(nx, ny); }
    break;

  case WM_SETCURSOR:
    if (LOWORD(lParam) == HTCLIENT) {
      while (window->parent()) window = window->window();
      SetCursor(Fl_X::i(window)->cursor);
      return 0;
    }
    break;

#if USE_COLORMAP
  case WM_QUERYNEWPALETTE :
    fl_GetDC(hWnd);
    if (fl_select_palette()) InvalidateRect(hWnd, NULL, FALSE);
    break;

  case WM_PALETTECHANGED:
    fl_GetDC(hWnd);
    if ((HWND)wParam != hWnd && fl_select_palette()) UpdateColors(fl_gc);
    break;

  case WM_CREATE :
    fl_GetDC(hWnd);
    fl_select_palette();
    break;
#endif

  case WM_DESTROYCLIPBOARD:
    fl_i_own_selection[1] = 0;
    return 1;

  case WM_DISPLAYCHANGE: // occurs when screen configuration (number, position) changes
    Fl::call_screen_init();
    Fl::handle(FL_SCREEN_CONFIGURATION_CHANGED, NULL);
    return 0;

  case WM_CHANGECBCHAIN:
    if ((hWnd == clipboard_wnd) && (next_clipboard_wnd == (HWND)wParam))
      next_clipboard_wnd = (HWND)lParam;
    else
      SendMessage(next_clipboard_wnd, WM_CHANGECBCHAIN, wParam, lParam);
    return 0;

  case WM_DRAWCLIPBOARD:
    // When the clipboard moves between two FLTK windows,
    // fl_i_own_selection will temporarily be false as we are
    // processing this message. Hence the need to use fl_find().
    if (!initial_clipboard && !fl_find(GetClipboardOwner()))
      fl_trigger_clipboard_notify(1);
    initial_clipboard = false;

    if (next_clipboard_wnd)
      SendMessage(next_clipboard_wnd, WM_DRAWCLIPBOARD, wParam, lParam);

    return 0;

  default:
    if (Fl::handle(0,0)) return 0;
    break;
  }


  return DefWindowProcW(hWnd, uMsg, wParam, lParam);
}

////////////////////////////////////////////////////////////////
// This function gets the dimensions of the top/left borders and
// the title bar, if there is one, based on the FL_BORDER, FL_MODAL
// and FL_NONMODAL flags, and on the window's size range.
// It returns the following values:
//
// value | border | title bar
//   0   |  none  |   no
//   1   |  fix   |   yes
//   2   |  size  |   yes

static int fake_X_wm_style(const Fl_Window* w,int &X,int &Y, int &bt,int &bx, int &by, DWORD style, DWORD styleEx,
                     int w_maxw, int w_minw, int w_maxh, int w_minh, uchar w_size_range_set) {
  int W = 0, H = 0, xoff = 0, yoff = 0, dx = 0, dy = 0;
  int ret = bx = by = bt = 0;

  int fallback = 1;
  if (!w->parent()) {
    if (fl_xid(w) || style) {
      // The block below calculates the window borders by requesting the
      // required decorated window rectangle for a desired client rectangle.
      // If any part of the function above fails, we will drop to a 
      // fallback to get the best guess which is always available.
      
	 if (!style) {
	     HWND hwnd = fl_xid(w);
          // request the style flags of this window, as WIN32 sees them
          style = GetWindowLong(hwnd, GWL_STYLE);
          styleEx = GetWindowLong(hwnd, GWL_EXSTYLE);
	 }

      RECT r;
      r.left = w->x();
      r.top = w->y();
      r.right = w->x()+w->w();
      r.bottom = w->y()+w->h();
      // get the decoration rectangle for the desired client rectangle
      BOOL ok = AdjustWindowRectEx(&r, style, FALSE, styleEx);
      if (ok) {
        X = r.left;
        Y = r.top;
        W = r.right - r.left;
        H = r.bottom - r.top;
        bx = w->x() - r.left;
        by = r.bottom - w->y() - w->h(); // height of the bottom frame
        bt = w->y() - r.top - by; // height of top caption bar
        xoff = bx;
        yoff = by + bt;
        dx = W - w->w();
        dy = H - w->h();
        if (w_size_range_set && (w_maxw != w_minw || w_maxh != w_minh))
          ret = 2;
        else
          ret = 1;
        fallback = 0;
      }
    }
  }
  // This is the original (pre 1.1.7) routine to calculate window border sizes.
  if (fallback) {
    if (w->border() && !w->parent()) {
      if (w_size_range_set && (w_maxw != w_minw || w_maxh != w_minh)) {
	ret = 2;
	bx = GetSystemMetrics(SM_CXSIZEFRAME);
	by = GetSystemMetrics(SM_CYSIZEFRAME);
      } else {
	ret = 1;
	int padding = GetSystemMetrics(SM_CXPADDEDBORDER);
	NONCLIENTMETRICS ncm;
	ncm.cbSize = sizeof(NONCLIENTMETRICS);
	SystemParametersInfo(SPI_GETNONCLIENTMETRICS, 0, &ncm, 0);
	bx = GetSystemMetrics(SM_CXFIXEDFRAME) + (padding ? padding + ncm.iBorderWidth : 0);
	by = GetSystemMetrics(SM_CYFIXEDFRAME) + (padding ? padding + ncm.iBorderWidth : 0);
      }
      bt = GetSystemMetrics(SM_CYCAPTION);
    }
    //The coordinates of the whole window, including non-client area
    xoff = bx;
    yoff = by + bt;
    dx = 2*bx;
    dy = 2*by + bt;
    X = w->x()-xoff;
    Y = w->y()-yoff;
    W = w->w()+dx;
    H = w->h()+dy;
  }

  //Proceed to positioning the window fully inside the screen, if possible
  //Find screen that contains most of the window
  //FIXME: this ought to be the "work area" instead of the entire screen !
  int scr_x = 0, scr_y = 0, scr_w = 0, scr_h = 0;
  Fl::screen_xywh(scr_x, scr_y, scr_w, scr_h, X, Y, W, H);
  //Make border's lower right corner visible
  if (scr_x+scr_w < X+W) X = scr_x+scr_w - W;
  if (scr_y+scr_h < Y+H) Y = scr_y+scr_h - H;
  //Make border's upper left corner visible
  if (X<scr_x) X = scr_x;
  if (Y<scr_y) Y = scr_y;
  //Make client area's lower right corner visible
  if (scr_x+scr_w < X+dx+ w->w()) X = scr_x+scr_w - w->w() - dx;
  if (scr_y+scr_h < Y+dy+ w->h()) Y = scr_y+scr_h - w->h() - dy;
  //Make client area's upper left corner visible
  if (X+xoff < scr_x) X = scr_x-xoff;
  if (Y+yoff < scr_y) Y = scr_y-yoff;
  //Return the client area's top left corner in (X,Y)
  X+=xoff;
  Y+=yoff;

  if (w->fullscreen_active()) {
    bx = by = bt = 0;
  }

  return ret;
}

int Fl_X::fake_X_wm(const Fl_Window* w,int &X,int &Y, int &bt,int &bx, int &by) {
  return fake_X_wm_style(w, X, Y, bt, bx, by, 0, 0, w->maxw, w->minw, w->maxh, w->minh, w->size_range_set);
}

////////////////////////////////////////////////////////////////

void Fl_Window::resize(int X,int Y,int W,int H) {
  UINT flags = SWP_NOSENDCHANGING | SWP_NOZORDER 
             | SWP_NOACTIVATE | SWP_NOOWNERZORDER;
  int is_a_resize = (W != w() || H != h());
  int resize_from_program = (this != resize_bug_fix);
  if (!resize_from_program) resize_bug_fix = 0;
  if (X != x() || Y != y()) {
    force_position(1);
  } else {
    if (!is_a_resize) return;
    flags |= SWP_NOMOVE;
  }
  if (is_a_resize) {
    Fl_Group::resize(X,Y,W,H);
    if (visible_r()) {
      redraw(); 
      // only wait for exposure if this window has a size - a window 
      // with no width or height will never get an exposure event
      if (i && W>0 && H>0)
        i->wait_for_expose = 1;
    }
  } else {
    x(X); y(Y);
    flags |= SWP_NOSIZE;
  }
  if (!border()) flags |= SWP_NOACTIVATE;
  if (resize_from_program && shown()) {
    if (!resizable()) size_range(w(),h(),w(),h());
    int dummy_x, dummy_y, bt, bx, by;
    //Ignore window managing when resizing, so that windows (and more
    //specifically menus) can be moved offscreen.
    if (Fl_X::fake_X_wm(this, dummy_x, dummy_y, bt, bx, by)) {
      X -= bx;
      Y -= by+bt;
      W += 2*bx;
      H += 2*by+bt;
    }
    // avoid zero size windows. A zero sized window on Win32
    // will cause continouly  new redraw events.
    if (W<=0) W = 1;
    if (H<=0) H = 1;
    SetWindowPos(i->xid, 0, X, Y, W, H, flags);
  }
}

void Fl_X::make_fullscreen(int X, int Y, int W, int H) {
  int top, bottom, left, right;
  int sx, sy, sw, sh;

  top = w->fullscreen_screen_top;
  bottom = w->fullscreen_screen_bottom;
  left = w->fullscreen_screen_left;
  right = w->fullscreen_screen_right;

  if ((top < 0) || (bottom < 0) || (left < 0) || (right < 0)) {
    top = Fl::screen_num(X, Y, W, H);
    bottom = top;
    left = top;
    right = top;
  }

  Fl::screen_xywh(sx, sy, sw, sh, top);
  Y = sy;
  Fl::screen_xywh(sx, sy, sw, sh, bottom);
  H = sy + sh - Y;
  Fl::screen_xywh(sx, sy, sw, sh, left);
  X = sx;
  Fl::screen_xywh(sx, sy, sw, sh, right);
  W = sx + sw - X;

  DWORD flags = GetWindowLong(xid, GWL_STYLE);
  flags = flags & ~(WS_THICKFRAME|WS_CAPTION);
  SetWindowLong(xid, GWL_STYLE, flags);

  // SWP_NOSENDCHANGING is so that we can override size limits
  SetWindowPos(xid, HWND_TOP, X, Y, W, H, SWP_NOSENDCHANGING | SWP_FRAMECHANGED);
}

void Fl_Window::fullscreen_x() {
  _set_fullscreen();
  i->make_fullscreen(x(), y(), w(), h());
  Fl::handle(FL_FULLSCREEN, this);
}

void Fl_Window::fullscreen_off_x(int X, int Y, int W, int H) {
  _clear_fullscreen();
  DWORD style = GetWindowLong(fl_xid(this), GWL_STYLE);
  // Remove the xid temporarily so that Fl_X::fake_X_wm() behaves like it
  // does in Fl_X::make().
  HWND xid = fl_xid(this);
  Fl_X::i(this)->xid = NULL;
  int wx, wy, bt, bx, by;
  switch (Fl_X::fake_X_wm(this, wx, wy, bt, bx, by)) {
  case 0: 
    break;
  case 1: 
    style |= WS_CAPTION; 
    break;
  case 2: 
    if (border()) {
      style |= WS_THICKFRAME | WS_CAPTION; 
    }
    break;
  }
  Fl_X::i(this)->xid = xid;
  // Adjust for decorations (but not if that puts the decorations
  // outside the screen)
  if ((X != x()) || (Y != y())) {
    X -= bx;
    Y -= by+bt;
  }
  W += bx*2;
  H += by*2+bt;
  SetWindowLong(fl_xid(this), GWL_STYLE, style);
  SetWindowPos(fl_xid(this), 0, X, Y, W, H,
               SWP_NOACTIVATE | SWP_NOZORDER | SWP_FRAMECHANGED);
  Fl::handle(FL_FULLSCREEN, this);
}


////////////////////////////////////////////////////////////////

/*
 * This silly little class remembers the name of all window classes 
 * we register to avoid double registration. It has the added bonus 
 * of freeing everything on application close as well.
 */
class NameList {
public:
  NameList() { name = (char**)malloc(sizeof(char**)); NName = 1; nName = 0; }
  ~NameList() { 
    int i;
    for (i=0; i<nName; i++) free(name[i]);
    if (name) free(name); 
  }
  void add_name(const char *n) {
    if (NName==nName) {
      NName += 5;
      name = (char**)realloc(name, NName * sizeof(char*));
    }
    name[nName++] = strdup(n);
  }
  char has_name(const char *n) {
    int i;
    for (i=0; i<nName; i++) {
      if (strcmp(name[i], n)==0) return 1;
    }
    return 0;
  }
private:
  char **name;
  int nName, NName;
};

void fl_fix_focus(); // in Fl.cxx

char fl_show_iconic;	// hack for Fl_Window::iconic()
// int fl_background_pixel = -1; // color to use for background
UINT fl_wake_msg = 0;
int fl_disable_transient_for; // secret method of removing TRANSIENT_FOR

Fl_X* Fl_X::make(Fl_Window* w) {
  Fl_Group::current(0); // get rid of very common user bug: forgot end()

  fl_open_display();

  // if the window is a subwindow and our parent is not mapped yet, we
  // mark this window visible, so that mapping the parent at a later
  // point in time will call this function again to finally map the subwindow.
  if (w->parent() && !Fl_X::i(w->window())) {
    w->set_visible();
    return 0L;
  }

  static NameList class_name_list;
  static const char *first_class_name = 0L;
  const char *class_name = w->xclass();
  if (!class_name) class_name = first_class_name; // reuse first class name used
  if (!class_name) class_name = "FLTK"; // default to create a "FLTK" WNDCLASS
  if (!first_class_name) {
    first_class_name = class_name;
  }

  wchar_t class_namew[100]; // (limited) buffer for Windows class name

  // convert UTF-8 class_name to wchar_t for RegisterClassExW and CreateWindowExW

  fl_utf8toUtf16(class_name,
		 (unsigned)strlen(class_name),		// in
		 (unsigned short*)class_namew,		// out
		 (unsigned)sizeof(class_namew)/sizeof(wchar_t));	// max. size

  if (!class_name_list.has_name(class_name)) {
    WNDCLASSEXW wcw;
    memset(&wcw, 0, sizeof(wcw));
    wcw.cbSize = sizeof(WNDCLASSEXW);

    // Documentation states a device context consumes about 800 bytes
    // of memory... so who cares? If 800 bytes per window is what it
    // takes to speed things up, I'm game.
    //wc.style = CS_HREDRAW | CS_VREDRAW | CS_CLASSDC | CS_DBLCLKS;
    wcw.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC | CS_DBLCLKS;
    wcw.lpfnWndProc = (WNDPROC)WndProc;
    wcw.cbClsExtra = wcw.cbWndExtra = 0;
    wcw.hInstance = fl_display;
    if (!w->icon() && !w->icon_->count)
      w->icon((void *)LoadIcon(NULL, IDI_APPLICATION));
    wcw.hIcon = wcw.hIconSm = (HICON)w->icon();
    wcw.hCursor = LoadCursor(NULL, IDC_ARROW);
    //uchar r,g,b; Fl::get_color(FL_GRAY,r,g,b);
    //wc.hbrBackground = (HBRUSH)CreateSolidBrush(RGB(r,g,b));
    wcw.hbrBackground = NULL;
    wcw.lpszMenuName = NULL;
    wcw.lpszClassName = class_namew;
    RegisterClassExW(&wcw);
    class_name_list.add_name(class_name);
  }

  const wchar_t* message_namew = L"FLTK::ThreadWakeup";
  if (!fl_wake_msg) fl_wake_msg = RegisterWindowMessageW(message_namew);

  HWND parent;
  DWORD style = WS_CLIPCHILDREN | WS_CLIPSIBLINGS;
  DWORD styleEx = WS_EX_LEFT;

  int xp = w->x();
  int yp = w->y();
  int wp = w->w();
  int hp = w->h();

  int showit = 1;

  if (w->parent()) {
    style |= WS_CHILD;
    styleEx |= WS_EX_WINDOWEDGE | WS_EX_CONTROLPARENT;
    parent = fl_xid(w->window());
  } else {
    if (!w->size_range_set) {
      if (w->resizable()) {
	Fl_Widget *o = w->resizable();
	int minw = o->w(); if (minw > 100) minw = 100;
	int minh = o->h(); if (minh > 100) minh = 100;
	w->size_range(w->w() - o->w() + minw, w->h() - o->h() + minh, 0, 0);
      } else {
	w->size_range(w->w(), w->h(), w->w(), w->h());
      }
    }
    styleEx |= WS_EX_WINDOWEDGE | WS_EX_CONTROLPARENT;

    int wintype = 0;
    if (w->border() && !w->parent()) {
      if (w->size_range_set && (w->maxw != w->minw || w->maxh != w->minh)) wintype = 2;
	  else wintype = 1;
    }

    switch (wintype) {
      // No border (used for menus)
      case 0:
        style |= WS_POPUP;
        styleEx |= WS_EX_TOOLWINDOW;
	      break;

      // Thin border and title bar
      case 1:
        style |= WS_DLGFRAME | WS_CAPTION;
        if (!w->modal())
          style |= WS_SYSMENU | WS_MINIMIZEBOX;
        break;

      // Thick, resizable border and title bar, with maximize button
      case 2:
        style |= WS_THICKFRAME | WS_SYSMENU | WS_MAXIMIZEBOX | WS_CAPTION;
        if (!w->modal())
          style |= WS_MINIMIZEBOX;
        break;
    }

    int xwm = xp , ywm = yp , bt, bx, by;
    fake_X_wm_style(w, xwm, ywm, bt, bx, by, style, styleEx, w->maxw, w->minw, w->maxh, w->minh, w->size_range_set);
    if (by+bt) {
      wp += 2*bx;
      hp += 2*by+bt;
    }
    if (!w->force_position()) {
      xp = yp = CW_USEDEFAULT;
    } else {
      if (!Fl::grab()) {
	xp = xwm; yp = ywm;
        w->x(xp);w->y(yp);
      }
      xp -= bx;
      yp -= by+bt;
    }

    parent = 0;
    if (w->non_modal() && Fl_X::first && !fl_disable_transient_for) {
      // find some other window to be "transient for":
      Fl_Window* w = Fl_X::first->w;
      while (w->parent()) w = w->window();
      parent = fl_xid(w);
      if (!w->visible()) showit = 0;
    } else if (Fl::grab()) parent = fl_xid(Fl::grab());
  }

  Fl_X* x = new Fl_X;
  x->other_xid = 0;
  x->setwindow(w);
  x->region = 0;
  x->private_dc = 0;
  x->cursor = LoadCursor(NULL, IDC_ARROW);
  x->custom_cursor = 0;
  if (!fl_codepage) fl_get_codepage();

  WCHAR *lab = NULL;
  if (w->label()) {
    size_t l = strlen(w->label());
//  lab = (WCHAR*) malloc((l + 1) * sizeof(short));
//  l = fl_utf2unicode((unsigned char*)w->label(), l, (xchar*)lab);
//  lab[l] = 0;
    unsigned wlen = fl_utf8toUtf16(w->label(), (unsigned) l, NULL, 0); // Pass NULL to query length
    wlen++;
    lab = (WCHAR *) malloc(sizeof(WCHAR)*wlen);
    wlen = fl_utf8toUtf16(w->label(), (unsigned) l, (unsigned short*)lab, wlen);
    lab[wlen] = 0;
  }
  x->xid = CreateWindowExW(
    styleEx,
    class_namew, lab, style,
    xp, yp, wp, hp,
    parent,
    NULL, // menu
    fl_display,
    NULL // creation parameters
  );
  if (lab) free(lab);

  x->next = Fl_X::first;
  Fl_X::first = x;

  x->set_icons();

  if (w->fullscreen_active()) {
  /* We need to make sure that the fullscreen is created on the
     default monitor, ie the desktop where the shortcut is located
     etc. This requires that CreateWindow is called with CW_USEDEFAULT
     for x and y. We can then use GetWindowRect to determine which
     monitor the window was placed on. */
    RECT rect;
    GetWindowRect(x->xid, &rect);
    x->make_fullscreen(rect.left, rect.top, 
                       rect.right - rect.left, rect.bottom - rect.top);
  }

  // Setup clipboard monitor target if there are registered handlers and
  // no window is targeted.
  if (!fl_clipboard_notify_empty() && clipboard_wnd == NULL)
    fl_clipboard_notify_target(x->xid);

  x->wait_for_expose = 1;
  if (fl_show_iconic) {showit = 0; fl_show_iconic = 0;}
  if (showit) {
    w->set_visible();
    int old_event = Fl::e_number;
    w->handle(Fl::e_number = FL_SHOW); // get child windows to appear
    Fl::e_number = old_event;
    w->redraw(); // force draw to happen
  }

  // Needs to be done before ShowWindow() to get the correct behaviour
  // when we get WM_SETFOCUS.
  if (w->modal()) {Fl::modal_ = w; fl_fix_focus();}

  // If we've captured the mouse, we dont want to activate any
  // other windows from the code, or we lose the capture.
  ShowWindow(x->xid, !showit ? SW_SHOWMINNOACTIVE :
	     (Fl::grab() || (styleEx & WS_EX_TOOLWINDOW)) ? SW_SHOWNOACTIVATE : SW_SHOWNORMAL);

  // Register all windows for potential drag'n'drop operations
  RegisterDragDrop(x->xid, flIDropTarget);

  if (!im_enabled)
    flImmAssociateContextEx(x->xid, 0, 0);

  return x;
}




/////////////////////////////////////////////////////////////////////////////
/// Win32 timers
///


static LRESULT CALLBACK s_TimerProc(HWND hwnd, UINT msg,
                                    WPARAM wParam, LPARAM lParam)
{
  switch (msg) {
  case WM_TIMER:
    {
      unsigned int id = (unsigned) (wParam - 1);
      if (id < (unsigned int)win32_timer_used && win32_timers[id].handle) {
        Fl_Timeout_Handler cb   = win32_timers[id].callback;
        void*              data = win32_timers[id].data;
        delete_timer(win32_timers[id]);
        if (cb) {
          (*cb)(data);
        }
      }
    }
    return 0;

  default:
    break;
  }

  return DefWindowProc(hwnd, msg, wParam, lParam);
}

void Fl::add_timeout(double time, Fl_Timeout_Handler cb, void* data)
{
  repeat_timeout(time, cb, data);
}

void Fl::repeat_timeout(double time, Fl_Timeout_Handler cb, void* data)
{
  int timer_id = -1;
  for (int i = 0;  i < win32_timer_used;  ++i) {
    if ( !win32_timers[i].handle ) {
      timer_id = i;
      break;
    }
  }
  if (timer_id == -1) {
    if (win32_timer_used == win32_timer_alloc) {
      realloc_timers();
    }
    timer_id = win32_timer_used++;
  }
  unsigned int elapsed = (unsigned int)(time * 1000);

  if ( !s_TimerWnd ) {
    const char* timer_class = "FLTimer";
    WNDCLASSEX wc;
    memset(&wc, 0, sizeof(wc));
    wc.cbSize = sizeof (wc);
    wc.style = CS_CLASSDC;
    wc.lpfnWndProc = (WNDPROC)s_TimerProc;
    wc.hInstance = fl_display;
    wc.lpszClassName = timer_class;
    /*ATOM atom =*/ RegisterClassEx(&wc);
    // create a zero size window to handle timer events
    s_TimerWnd = CreateWindowEx(WS_EX_LEFT | WS_EX_TOOLWINDOW,
                                timer_class, "",
                                WS_POPUP,
                                0, 0, 0, 0,
                                NULL, NULL, fl_display, NULL);
    // just in case this OS won't let us create a 0x0 size window:
    if (!s_TimerWnd)
      s_TimerWnd = CreateWindowEx(WS_EX_LEFT | WS_EX_TOOLWINDOW,
				  timer_class, "",
				  WS_POPUP,
				  0, 0, 1, 1,
				  NULL, NULL, fl_display, NULL);
    ShowWindow(s_TimerWnd, SW_SHOWNOACTIVATE);
  }

  win32_timers[timer_id].callback = cb;
  win32_timers[timer_id].data     = data;

  win32_timers[timer_id].handle =
    SetTimer(s_TimerWnd, timer_id + 1, elapsed, NULL);
}

int Fl::has_timeout(Fl_Timeout_Handler cb, void* data)
{
  for (int i = 0;  i < win32_timer_used;  ++i) {
    Win32Timer& t = win32_timers[i];
    if (t.handle  &&  t.callback == cb  &&  t.data == data) {
      return 1;
    }
  }
  return 0;
}

void Fl::remove_timeout(Fl_Timeout_Handler cb, void* data)
{
  int i;
  for (i = 0;  i < win32_timer_used;  ++i) {
    Win32Timer& t = win32_timers[i];
    if (t.handle  &&  t.callback == cb  &&
      (t.data == data  ||  data == NULL)) {
      delete_timer(t);
    }
  }
}

/// END TIMERS
/////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////

HINSTANCE fl_display = GetModuleHandle(NULL);

void Fl_Window::size_range_() {
  size_range_set = 1;
}

void Fl_X::set_minmax(LPMINMAXINFO minmax)
{
  int td, wd, hd, dummy_x, dummy_y;

  fake_X_wm(w, dummy_x, dummy_y, td, wd, hd);
  wd *= 2;
  hd *= 2;
  hd += td;

  minmax->ptMinTrackSize.x = w->minw + wd;
  minmax->ptMinTrackSize.y = w->minh + hd;
  if (w->maxw) {
    minmax->ptMaxTrackSize.x = w->maxw + wd;
    minmax->ptMaxSize.x = w->maxw + wd;
  }
  if (w->maxh) {
    minmax->ptMaxTrackSize.y = w->maxh + hd;
    minmax->ptMaxSize.y = w->maxh + hd;
  }
}

////////////////////////////////////////////////////////////////

#include <FL/filename.H> // need so FL_EXPORT fl_filename_name works

// returns pointer to the filename, or null if name ends with '/'
const char *fl_filename_name(const char *name) {
  const char *p,*q;
  if (!name) return (0);
  q = name;
  if (q[0] && q[1]==':') q += 2; // skip leading drive letter
  for (p = q; *p; p++) if (*p == '/' || *p == '\\') q = p+1;
  return q;
}

void Fl_Window::label(const char *name,const char *iname) {
  Fl_Widget::label(name);
  iconlabel_ = iname;
  if (shown() && !parent()) {
    if (!name) name = "";
    size_t l = strlen(name);
//  WCHAR *lab = (WCHAR*) malloc((l + 1) * sizeof(short));
//  l = fl_utf2unicode((unsigned char*)name, l, (xchar*)lab);
    unsigned wlen = fl_utf8toUtf16(name, (unsigned) l, NULL, 0); // Pass NULL to query length
    wlen++;
    unsigned short * lab = (unsigned short*)malloc(sizeof(unsigned short)*wlen);
    wlen = fl_utf8toUtf16(name, (unsigned) l, lab, wlen);
    lab[wlen] = 0;
    SetWindowTextW(i->xid, (WCHAR *)lab);
    free(lab);
  }
}

////////////////////////////////////////////////////////////////

static HICON image_to_icon(const Fl_RGB_Image *image, bool is_icon,
                           int hotx, int hoty) {
  BITMAPV5HEADER bi;
  HBITMAP bitmap, mask;
  DWORD *bits;
  HICON icon;

  if (!is_icon) {
    if ((hotx < 0) || (hotx >= image->w()))
      return NULL;
    if ((hoty < 0) || (hoty >= image->h()))
      return NULL;
  }

  memset(&bi, 0, sizeof(BITMAPV5HEADER));

  bi.bV5Size        = sizeof(BITMAPV5HEADER);
  bi.bV5Width       = image->w();
  bi.bV5Height      = -image->h(); // Negative for top-down
  bi.bV5Planes      = 1;
  bi.bV5BitCount    = 32;
  bi.bV5Compression = BI_BITFIELDS;
  bi.bV5RedMask     = 0x00FF0000;
  bi.bV5GreenMask   = 0x0000FF00;
  bi.bV5BlueMask    = 0x000000FF;
  bi.bV5AlphaMask   = 0xFF000000;

  HDC hdc;

  hdc = GetDC(NULL);
  bitmap = CreateDIBSection(hdc, (BITMAPINFO*)&bi, DIB_RGB_COLORS, (void**)&bits, NULL, 0);
  ReleaseDC(NULL, hdc);

  if (bits == NULL)
    return NULL;

  const uchar *i = (const uchar*)*image->data();
  const int extra_data = image->ld() ? (image->ld()-image->w()*image->d()) : 0;

  for (int y = 0; y < image->h(); y++) {
    for (int x = 0; x < image->w(); x++) {
      switch (image->d()) {
      case 1:
        *bits = (0xff<<24) | (i[0]<<16) | (i[0]<<8) | i[0];
        break;
      case 2:
        *bits = (i[1]<<24) | (i[0]<<16) | (i[0]<<8) | i[0];
        break;
      case 3:
        *bits = (0xff<<24) | (i[0]<<16) | (i[1]<<8) | i[2];
        break;
      case 4:
        *bits = (i[3]<<24) | (i[0]<<16) | (i[1]<<8) | i[2];
        break;
      }
      i += image->d();
      bits++;
    }
    i += extra_data;
  }

  // A mask bitmap is still needed even though it isn't used
  mask = CreateBitmap(image->w(),image->h(),1,1,NULL);
  if (mask == NULL) {
    DeleteObject(bitmap);
    return NULL;
  }

  ICONINFO ii;

  ii.fIcon    = is_icon;
  ii.xHotspot = hotx;
  ii.yHotspot = hoty;
  ii.hbmMask  = mask;
  ii.hbmColor = bitmap;

  icon = CreateIconIndirect(&ii);

  DeleteObject(bitmap);
  DeleteObject(mask);

  return icon;
}

////////////////////////////////////////////////////////////////

static HICON default_big_icon = NULL;
static HICON default_small_icon = NULL;

static const Fl_RGB_Image *find_best_icon(int ideal_width, 
                                          const Fl_RGB_Image *icons[],
                                          int count) {
  const Fl_RGB_Image *best;

  best = NULL;

  for (int i = 0;i < count;i++) {
    if (best == NULL)
      best = icons[i];
    else {
      if (best->w() < ideal_width) {
        if (icons[i]->w() > best->w())
          best = icons[i];
      } else {
        if ((icons[i]->w() >= ideal_width) &&
            (icons[i]->w() < best->w()))
          best = icons[i];
      }
    }
  }

  return best;
}

void Fl_X::set_default_icons(const Fl_RGB_Image *icons[], int count) {
  const Fl_RGB_Image *best_big, *best_small;

  if (default_big_icon != NULL)
    DestroyIcon(default_big_icon);
  if (default_small_icon != NULL)
    DestroyIcon(default_small_icon);

  default_big_icon = NULL;
  default_small_icon = NULL;

  best_big = find_best_icon(GetSystemMetrics(SM_CXICON), icons, count);
  best_small = find_best_icon(GetSystemMetrics(SM_CXSMICON), icons, count);

  if (best_big != NULL)
    default_big_icon = image_to_icon(best_big, true, 0, 0);

  if (best_small != NULL)
    default_small_icon = image_to_icon(best_small, true, 0, 0);
}

void Fl_X::set_default_icons(HICON big_icon, HICON small_icon) {
  if (default_big_icon != NULL)
    DestroyIcon(default_big_icon);
  if (default_small_icon != NULL)
    DestroyIcon(default_small_icon);

  default_big_icon = NULL;
  default_small_icon = NULL;

  if (big_icon != NULL)
    default_big_icon = CopyIcon(big_icon);
  if (small_icon != NULL)
    default_small_icon = CopyIcon(small_icon);
}

void Fl_X::set_icons() {
  HICON big_icon, small_icon;

  // Windows doesn't copy the icons, so we have to "leak" them when
  // setting, and clean up when we change to some other icons.
  big_icon = (HICON)SendMessage(xid, WM_GETICON, ICON_BIG, 0);
  if ((big_icon != NULL) && (big_icon != default_big_icon))
    DestroyIcon(big_icon);
  small_icon = (HICON)SendMessage(xid, WM_GETICON, ICON_SMALL, 0);
  if ((small_icon != NULL) && (small_icon != default_small_icon))
    DestroyIcon(small_icon);

  big_icon = NULL;
  small_icon = NULL;

  if (w->icon_->count) {
    const Fl_RGB_Image *best_big, *best_small;

    best_big = find_best_icon(GetSystemMetrics(SM_CXICON),
                              (const Fl_RGB_Image **)w->icon_->icons,
                              w->icon_->count);
    best_small = find_best_icon(GetSystemMetrics(SM_CXSMICON),
                                (const Fl_RGB_Image **)w->icon_->icons,
                                w->icon_->count);

    if (best_big != NULL)
      big_icon = image_to_icon(best_big, true, 0, 0);
    if (best_small != NULL)
      small_icon = image_to_icon(best_small, true, 0, 0);
  } else {
    if ((w->icon_->big_icon != NULL) || (w->icon_->small_icon != NULL)) {
      big_icon = w->icon_->big_icon;
      small_icon = w->icon_->small_icon;
    } else {
      big_icon = default_big_icon;
      small_icon = default_small_icon;
    }
  }

  SendMessage(xid, WM_SETICON, ICON_BIG, (LPARAM)big_icon);
  SendMessage(xid, WM_SETICON, ICON_SMALL, (LPARAM)small_icon);
}

/** Sets the default window icons.

  Convenience function to set the default icons using Windows'
  native HICON icon handles.

  The given icons are copied. You can free the icons immediately after
  this call.

  \param[in] big_icon default large icon for all windows
                      subsequently created
  \param[in] small_icon default small icon for all windows
                        subsequently created

  \see Fl_Window::default_icon(const Fl_RGB_Image *)
  \see Fl_Window::default_icons(const Fl_RGB_Image *[], int)
  \see Fl_Window::icon(const Fl_RGB_Image *)
  \see Fl_Window::icons(const Fl_RGB_Image *[], int)
  \see Fl_Window::icons(HICON, HICON)
 */
void Fl_Window::default_icons(HICON big_icon, HICON small_icon) {
  Fl_X::set_default_icons(big_icon, small_icon);
}

/** Sets the window icons.

  Convenience function to set this window's icons using Windows'
  native HICON icon handles.

  The given icons are copied. You can free the icons immediately after
  this call.

  \param[in] big_icon large icon for this window
  \param[in] small_icon small icon for this windows

  \see Fl_Window::default_icon(const Fl_RGB_Image *)
  \see Fl_Window::default_icons(const Fl_RGB_Image *[], int)
  \see Fl_Window::default_icons(HICON, HICON)
  \see Fl_Window::icon(const Fl_RGB_Image *)
  \see Fl_Window::icons(const Fl_RGB_Image *[], int)
 */
void Fl_Window::icons(HICON big_icon, HICON small_icon) {
  free_icons();

  if (big_icon != NULL)
    icon_->big_icon = CopyIcon(big_icon);
  if (small_icon != NULL)
    icon_->small_icon = CopyIcon(small_icon);

  if (i)
    i->set_icons();
}

////////////////////////////////////////////////////////////////

#ifndef IDC_HAND
#  define IDC_HAND  MAKEINTRESOURCE(32649)
#endif // !IDC_HAND

int Fl_X::set_cursor(Fl_Cursor c) {
  LPSTR n;
  HCURSOR new_cursor;

  if (c == FL_CURSOR_NONE)
    new_cursor = NULL;
  else {
    switch (c) {
    case FL_CURSOR_ARROW:   n = IDC_ARROW; break;
    case FL_CURSOR_CROSS:   n = IDC_CROSS; break;
    case FL_CURSOR_WAIT:    n = IDC_WAIT; break;
    case FL_CURSOR_INSERT:  n = IDC_IBEAM; break;
    case FL_CURSOR_HAND:    n = IDC_HAND; break;
    case FL_CURSOR_HELP:    n = IDC_HELP; break;
    case FL_CURSOR_MOVE:    n = IDC_SIZEALL; break;
    case FL_CURSOR_N:
    case FL_CURSOR_S:
      // FIXME: Should probably have fallbacks for these instead
    case FL_CURSOR_NS:      n = IDC_SIZENS; break;
    case FL_CURSOR_NE:
    case FL_CURSOR_SW:
      // FIXME: Dito.
    case FL_CURSOR_NESW:    n = IDC_SIZENESW; break;
    case FL_CURSOR_E:
    case FL_CURSOR_W:
      // FIXME: Dito.
    case FL_CURSOR_WE:      n = IDC_SIZEWE; break;
    case FL_CURSOR_SE:
    case FL_CURSOR_NW:
      // FIXME: Dito.
    case FL_CURSOR_NWSE:    n = IDC_SIZENWSE; break;
    default:
      return 0;
    }

    new_cursor = LoadCursor(NULL, n);
    if (new_cursor == NULL)
      return 0;
  }

  if ((cursor != NULL) && custom_cursor)
    DestroyIcon(cursor);

  cursor = new_cursor;
  custom_cursor = 0;

  SetCursor(cursor);

  return 1;
}

int Fl_X::set_cursor(const Fl_RGB_Image *image, int hotx, int hoty) {
  HCURSOR new_cursor;

  new_cursor = image_to_icon(image, false, hotx, hoty);
  if (new_cursor == NULL)
    return 0;

  if ((cursor != NULL) && custom_cursor)
    DestroyIcon(cursor);

  cursor = new_cursor;
  custom_cursor = 1;

  SetCursor(cursor);

  return 1;
}

////////////////////////////////////////////////////////////////
// Implement the virtual functions for the base Fl_Window class:

// If the box is a filled rectangle, we can make the redisplay *look*
// faster by using X's background pixel erasing.  We can make it
// actually *be* faster by drawing the frame only, this is done by
// setting fl_boxcheat, which is seen by code in fl_drawbox.cxx:
// For WIN32 it looks like all windows share a background color, so
// I use FL_GRAY for this and only do this cheat for windows that are
// that color.
// Actually it is totally disabled.
// Fl_Widget *fl_boxcheat;
//static inline int can_boxcheat(uchar b) {return (b==1 || (b&2) && b<=15);}

void Fl_Window::show() {
  image(Fl::scheme_bg_);
  if (Fl::scheme_bg_) {
    labeltype(FL_NORMAL_LABEL);
    align(FL_ALIGN_CENTER | FL_ALIGN_INSIDE | FL_ALIGN_CLIP);
  } else {
    labeltype(FL_NO_LABEL);
  }
  Fl_Tooltip::exit(this);
  if (!shown()) {
    // if (can_boxcheat(box())) fl_background_pixel = fl_xpixel(color());
    Fl_X::make(this);
  } else {
    // Once again, we would lose the capture if we activated the window.
    if (IsIconic(i->xid)) OpenIcon(i->xid);
    if (!fl_capture) BringWindowToTop(i->xid);
    //ShowWindow(i->xid,fl_capture?SW_SHOWNOACTIVATE:SW_RESTORE);
  }
#ifdef USE_PRINT_BUTTON
  void preparePrintFront(void);
  preparePrintFront();
#endif
}

Fl_Window *Fl_Window::current_;
// the current context
HDC fl_gc = 0;
// the current window handle, initially set to -1 so we can correctly
// allocate fl_GetDC(0)
HWND fl_window = NULL;

// Here we ensure only one GetDC is ever in place.
HDC fl_GetDC(HWND w) {
  if (fl_gc) {
    if (w == fl_window  &&  fl_window != NULL) return fl_gc;
    if (fl_window) fl_release_dc(fl_window, fl_gc); // ReleaseDC
  }
  fl_gc = GetDC(w);
  fl_save_dc(w, fl_gc);
  fl_window = w;
  // calling GetDC seems to always reset these: (?)
  SetTextAlign(fl_gc, TA_BASELINE|TA_LEFT);
  SetBkMode(fl_gc, TRANSPARENT);

  return fl_gc;
}

// make X drawing go into this window (called by subclass flush() impl.)
void Fl_Window::make_current() {
  fl_GetDC(fl_xid(this));

#if USE_COLORMAP
  // Windows maintains a hardware and software color palette; the
  // SelectPalette() call updates the current soft->hard mapping
  // for all drawing calls, so we must select it here before any
  // code does any drawing...

  fl_select_palette();
#endif // USE_COLORMAP

  current_ = this;
  fl_clip_region(0);


}

/* Make sure that all allocated fonts are released. This works only if 
   Fl::run() is allowed to exit by closing all windows. Calling 'exit(int)'
   will not automatically free any fonts. */
void fl_free_fonts(void)
{
// remove the Fl_Font_Descriptor chains
  int i;
  Fl_Fontdesc * s;
  Fl_Font_Descriptor * f;
  Fl_Font_Descriptor * ff;
  for (i=0; i<FL_FREE_FONT; i++) {
    s = fl_fonts + i;
    for (f=s->first; f; f=ff) {
      ff = f->next;
      delete f;
      s->first = ff;
    }
  }
}


///////////////////////////////////////////////////////////////////////
//
//  The following routines help fix a problem with the leaking of Windows
//  Device Context (DC) objects. The 'proper' protocol is for a program to
//  acquire a DC, save its state, do the modifications needed for drawing,
//  perform the drawing, restore the initial state, and release the DC. In
//  FLTK, the save and restore steps have previously been omitted and DCs are
//  not properly released, leading to a great number of DC leaks. As some
//  Windows "OSs" will hang when any process exceeds roughly 10,000 GDI objects,
//  it is important to control GDI leaks, which are much more important than memory
//  leaks. The following struct, global variable, and routines help implement
//  the above protocol for those cases where the GetDC and RestoreDC are not in
//  the same routine. For each GetDC, fl_save_dc is used to create an entry in 
//  a linked list that saves the window handle, the DC handle, and the initial
//  state. When the DC is to be released, 'fl_release_dc' is called. It restores
//  the initial state and releases the DC. When the program exits, 'fl_cleanup_dc_list'
//  frees any remaining nodes in the list.

struct Win_DC_List {      // linked list 
  HWND    window;         // window handle
  HDC     dc;             // device context handle
  int     saved_dc;       // initial state of DC
  Win_DC_List * next;     // pointer to next item
};

static Win_DC_List * win_DC_list = 0;

void fl_save_dc( HWND w, HDC dc) {
  Win_DC_List * t;
  t = new Win_DC_List;
  t->window = w;
  t->dc = dc;
  t->saved_dc = SaveDC(dc);
  if (win_DC_list)
    t->next = win_DC_list;
  else
    t->next = NULL;
  win_DC_list = t;
}

void fl_release_dc(HWND w, HDC dc) {
  Win_DC_List * t= win_DC_list;
  Win_DC_List * prev = 0;
  if (!t)
    return;
  do {
    if (t->dc == dc) {
      RestoreDC(dc, t->saved_dc);
      ReleaseDC(w, dc);
      if (!prev) {
        win_DC_list = t->next;   // delete first item
      } else {
        prev->next = t->next;       // one in the middle
      }
      delete (t);
      return;
    }
    prev = t;
    t = t->next;
  } while (t);
}

void fl_cleanup_dc_list(void) {          // clean up the list
  Win_DC_List * t = win_DC_list;
  if (!t)return;
  do {
    RestoreDC(t->dc, t->saved_dc);
    ReleaseDC(t->window, t->dc);
    win_DC_list = t->next;
    delete (t);
    t = win_DC_list;
  } while(t);
}

Fl_Region XRectangleRegion(int x, int y, int w, int h) {
  if (Fl_Surface_Device::surface() == Fl_Display_Device::display_device()) return CreateRectRgn(x,y,x+w,y+h);
  // because rotation may apply, the rectangle becomes a polygon in device coords
  POINT pt[4] = { {x, y}, {x + w, y}, {x + w, y + h}, {x, y + h} };
  LPtoDP(fl_gc, pt, 4);
  return CreatePolygonRgn(pt, 4, ALTERNATE);
}

FL_EXPORT Window fl_xid_(const Fl_Window *w) {
  Fl_X *temp = Fl_X::i(w); 
  return temp ? temp->xid : 0;
}

static RECT border_width_title_bar_height(Fl_Window *win, int &bx, int &by, int &bt, float *pscaling=0)
{
  RECT r = {0,0,0,0};
  bx = by = bt = 0;
  float scaling = 1;
  if (win->shown() && !win->parent() && win->border() && win->visible()) {
    static HMODULE dwmapi_dll = LoadLibrary("dwmapi.dll");
    typedef HRESULT (WINAPI* DwmGetWindowAttribute_type)(HWND hwnd, DWORD dwAttribute, PVOID pvAttribute, DWORD cbAttribute);
    static DwmGetWindowAttribute_type DwmGetWindowAttribute = dwmapi_dll ?
    (DwmGetWindowAttribute_type)GetProcAddress(dwmapi_dll, "DwmGetWindowAttribute") : NULL;
    int need_r = 1;
    if (DwmGetWindowAttribute) {
      const DWORD DWMWA_EXTENDED_FRAME_BOUNDS = 9;
      if ( DwmGetWindowAttribute(fl_xid(win), DWMWA_EXTENDED_FRAME_BOUNDS, &r, sizeof(RECT)) == S_OK ) {
        need_r = 0;
        // Compute the global display scaling factor: 1, 1.25, 1.5, 1.75, etc...
        // This factor can be set in Windows 10 by
        // "Change the size of text, apps and other items" in display settings.
        HDC hdc = GetDC(NULL);
        int hr = GetDeviceCaps(hdc, HORZRES); // pixels visible to the app
#ifndef DESKTOPHORZRES
#define DESKTOPHORZRES 118
#endif
        int dhr = GetDeviceCaps(hdc, DESKTOPHORZRES); // true number of pixels on display
        ReleaseDC(NULL, hdc);
        scaling = dhr/float(hr); // display scaling factor
        scaling = int(scaling * 100 + 0.5)/100.; // round to 2 digits after decimal point
      }
    }
    if (need_r) {
      GetWindowRect(fl_xid(win), &r);
    }
    bx = (r.right - r.left - int(win->w() * scaling))/2;
    if (bx < 1) bx = 1;
    by = bx;
    bt = r.bottom - r.top - int(win->h() * scaling) - 2 * by;
  }
  if (pscaling) *pscaling = scaling;
  return r;
}

int Fl_Window::decorated_w()
{
  int bt, bx, by;
  border_width_title_bar_height(this, bx, by, bt);
  return w() + 2 * bx;
}

int Fl_Window::decorated_h()
{
  int bt, bx, by;
  float scaling;
  border_width_title_bar_height(this, bx, by, bt, &scaling);
  return h() + bt/scaling + 2 * by;
}

void Fl_Paged_Device::print_window(Fl_Window *win, int x_offset, int y_offset)
{
  draw_decorated_window(win, x_offset, y_offset, this);
}

void Fl_Paged_Device::draw_decorated_window(Fl_Window *win, int x_offset, int y_offset, Fl_Surface_Device *toset)
{
  int bt, bx, by; // border width and title bar height of window
  float scaling;
  RECT r = border_width_title_bar_height(win, bx, by, bt, &scaling);
  if (bt) {
    Fl_Display_Device::display_device()->set_current(); // make window current
    win->show();
    Fl::check();
    win->make_current();
    HDC save_gc = fl_gc;
    fl_gc = GetDC(NULL); // get the screen device context
    int ww = win->w() + 2 * bx;
    int wh = win->h() + bt + 2 * by;
    // capture the 4 window sides from screen
    Window save_win = fl_window;
    fl_window = NULL; // force use of read_win_rectangle() by fl_read_image()
    uchar *top_image = fl_read_image(NULL, r.left, r.top, r.right - r.left + 1, bt + by);
    uchar *left_image = bx ? fl_read_image(NULL, r.left, r.top, bx, wh) : NULL;
    uchar *right_image = bx ? fl_read_image(NULL, r.right - bx, r.top, bx, wh) : NULL;
    uchar *bottom_image = by ? fl_read_image(NULL, r.left, r.bottom-by, ww, by) : NULL;
    fl_window = save_win;
    ReleaseDC(NULL, fl_gc);  fl_gc = save_gc;
    toset->set_current();
    // draw the 4 window sides
    //fl_draw_image(top_image, x_offset, y_offset, ww, bt + by, 3);
    Fl_RGB_Image *top_r = new Fl_RGB_Image(top_image, r.right - r.left + 1, bt + by, 3);
    top_r->alloc_array = 1;
    if (scaling > 1) {
      Fl_RGB_Scaling current = Fl_Image::RGB_scaling();
      Fl_Image::RGB_scaling(FL_RGB_SCALING_BILINEAR);
      Fl_RGB_Image *tmp_img = (Fl_RGB_Image*)top_r->copy(ww, (bt + by)/scaling);
      Fl_Image::RGB_scaling(current);
      delete top_r;
      top_r = tmp_img;
    }
    top_r->draw(x_offset, y_offset);
    delete top_r;
    
    if (left_image) { fl_draw_image(left_image, x_offset, y_offset, bx, wh, 3); delete left_image; }
    if (right_image) { fl_draw_image(right_image, x_offset + win->w() + bx, y_offset, bx, wh, 3); delete right_image; }
    if (bottom_image) { fl_draw_image(bottom_image, x_offset, y_offset + win->h() + bt + by, ww, by, 3); delete bottom_image; }
  }
  // draw the window inner part
  this->print_widget(win, x_offset + bx, y_offset + (bt + by)/scaling);
}

#ifdef USE_PRINT_BUTTON
// to test the Fl_Printer class creating a "Print front window" button in a separate window
// contains also preparePrintFront call above
#include <FL/Fl_Printer.H>
#include <FL/Fl_Button.H>
void printFront(Fl_Widget *o, void *data)
{
  Fl_Printer printer;
  o->window()->hide();
  Fl_Window *win = Fl::first_window();
  if(!win) return;
  int w, h;
  if( printer.start_job(1) ) { o->window()->show(); return; }
  if( printer.start_page() ) { o->window()->show(); return; }
  printer.printable_rect(&w,&h);
  int  wh, ww;
  wh = win->decorated_h();
  ww = win->decorated_w();
  // scale the printer device so that the window fits on the page
  float scale = 1;
  if (ww > w || wh > h) {
    scale = (float)w/ww;
    if ((float)h/wh < scale) scale = (float)h/wh;
    printer.scale(scale, scale);
  }
// #define ROTATE 20.0
#ifdef ROTATE
  printer.scale(scale * 0.8, scale * 0.8);
  printer.printable_rect(&w, &h);
  printer.origin(w/2, h/2 );
  printer.rotate(ROTATE);
  printer.print_widget( win, - win->w()/2, - win->h()/2 );
  //printer.print_window_part( win, 0,0, win->w(), win->h(), - win->w()/2, - win->h()/2 );
#else  
  printer.print_window(win);
#endif
  printer.end_page();
  printer.end_job();
  o->window()->show();
}

#include <FL/Fl_Copy_Surface.H>
void copyFront(Fl_Widget *o, void *data)
{
  o->window()->hide();
  Fl_Window *win = Fl::first_window();
  if (!win) return;
  Fl_Copy_Surface *surf = new Fl_Copy_Surface(win->decorated_w(), win->decorated_h());
  surf->set_current();
  surf->draw_decorated_window(win); // draw the window content
  delete surf; // put the window on the clipboard
  Fl_Display_Device::display_device()->set_current();
  o->window()->show();
}

void preparePrintFront(void)
{
  static BOOL first=TRUE;
  if(!first) return;
  first=FALSE;
  static Fl_Window w(0,0,120,60);
  static Fl_Button bp(0,0,w.w(),30, "Print front window");
  bp.callback(printFront);
  static Fl_Button bc(0,30,w.w(),30, "Copy front window");
  bc.callback(copyFront);
  w.end();
  w.show();
}
#endif // USE_PRINT_BUTTON

#endif // FL_DOXYGEN

//
// End of "$Id$".
//
