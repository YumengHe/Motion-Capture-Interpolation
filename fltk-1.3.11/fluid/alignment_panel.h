//
// "$Id$"
//
// Setting and shell dialogs for the Fast Light Tool Kit (FLTK).
//
// Copyright 1998-2023 by Bill Spitzak and others.
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

// generated by Fast Light User Interface Designer (fluid) version 1.0310

#ifndef alignment_panel_h
#define alignment_panel_h
#include <FL/Fl.H>
#include <FL/Fl_Text_Buffer.H>
#include <FL/Fl_Text_Display.H>
#include <FL/filename.H>
extern void load_history();
extern void redraw_browser();
extern int show_comments;
extern int G_use_external_editor;
extern char G_external_editor_command[512];
extern int show_coredevmenus;
extern struct Fl_Menu_Item *dbmanager_item;
extern Fl_Preferences fluid_prefs;
extern Fl_Text_Buffer *shell_run_buffer; 
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Preferences.H>
#include <FL/Fl_Tooltip.H>
extern Fl_Double_Window *project_window;
#include <FL/Fl_Button.H>
#include <FL/Fl_Tabs.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Input.H>
extern void header_input_cb(Fl_Input*, void*);
extern Fl_Input *header_file_input;
extern void code_input_cb(Fl_Input*, void*);
extern Fl_Input *code_file_input;
#include <FL/Fl_Check_Button.H>
extern void include_H_from_C_button_cb(Fl_Check_Button*, void*);
extern Fl_Check_Button *include_H_from_C_button;
extern void use_FL_COMMAND_button_cb(Fl_Check_Button*, void*);
extern Fl_Check_Button *use_FL_COMMAND_button;
#include <FL/Fl_Choice.H>
extern void i18n_type_cb(Fl_Choice*, void*);
extern Fl_Choice *i18n_type_chooser;
extern void i18n_text_cb(Fl_Input*, void*);
extern Fl_Input *i18n_include_input;
extern Fl_Input *i18n_file_input;
#include <FL/Fl_Int_Input.H>
extern void i18n_int_cb(Fl_Int_Input*, void*);
extern Fl_Int_Input *i18n_set_input;
extern Fl_Input *i18n_function_input;
Fl_Double_Window* make_project_window();
extern Fl_Menu_Item menu_i18n_type_chooser[];
extern void i18n_cb(Fl_Choice *,void *);
extern void scheme_cb(Fl_Choice *, void *); 
extern Fl_Double_Window *settings_window;
extern void scheme_cb(Fl_Choice*, void*);
extern Fl_Choice *scheme_choice;
extern Fl_Check_Button *tooltips_button;
extern Fl_Check_Button *completion_button;
extern Fl_Check_Button *openlast_button;
extern Fl_Check_Button *prevpos_button;
extern Fl_Check_Button *show_comments_button;
#include <FL/Fl_Spinner.H>
extern Fl_Spinner *recent_spinner;
extern Fl_Check_Button *use_external_editor_button;
extern Fl_Input *editor_command_input;
Fl_Double_Window* make_settings_window();
extern Fl_Menu_Item menu_scheme_choice[];
extern Fl_Double_Window *shell_window;
extern Fl_Input *shell_command_input;
extern Fl_Check_Button *shell_writecode_button;
extern Fl_Check_Button *shell_writemsgs_button;
extern Fl_Check_Button *shell_savefl_button;
#include <FL/Fl_Return_Button.H>
extern void do_shell_command(Fl_Return_Button*, void*);
extern Fl_Double_Window *shell_run_window;
#include <FL/Fl_Text_Display.H>
extern Fl_Text_Display *shell_run_display;
extern Fl_Return_Button *shell_run_button;
Fl_Double_Window* make_shell_window();
extern Fl_Double_Window *grid_window;
extern void grid_cb(Fl_Int_Input*, long);
extern Fl_Int_Input *horizontal_input;
extern Fl_Int_Input *vertical_input;
extern Fl_Int_Input *snap_input;
extern void guides_cb(Fl_Check_Button*, long);
extern Fl_Check_Button *guides_toggle;
#include <FL/Fl_Round_Button.H>
extern void default_widget_size_cb(Fl_Round_Button*, long);
extern Fl_Round_Button *def_widget_size[6];
Fl_Double_Window* make_layout_window();
void show_global_settings_window();
extern Fl_Double_Window *global_settings_window;
extern Fl_Choice *wVisibleFocus;
extern Fl_Choice *wArrowFocus;
extern Fl_Choice *wShowTooltips;
extern Fl_Choice *wDNDText;
extern Fl_Choice *wGTKText;
extern Fl_Choice *wUserOrSystem;
Fl_Double_Window* make_global_settings_window();
extern Fl_Menu_Item menu_wVisibleFocus[];
extern Fl_Menu_Item menu_wArrowFocus[];
extern Fl_Menu_Item menu_wShowTooltips[];
extern Fl_Menu_Item menu_wDNDText[];
extern Fl_Menu_Item menu_wGTKText[];
extern Fl_Menu_Item menu_wUserOrSystem[];
#endif

//
// End of "$Id$".
//
