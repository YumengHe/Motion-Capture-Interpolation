#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Choice.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_PNG_Image.H>
#include <iostream>
#include <sstream>
#include <string>
#include <cstdio>
#include <filesystem>
#include <cstdlib>
#include <vector>

// -----------------------------------------------------------------------------
static const char* JOINT_NAMES[] = {
    "root", "lfemur", "ltibia", "lfoot", "ltoes",
    "rfemur", "rtibia", "rfoot", "rtoes",
    "lowerback", "upperback", "thorax",
    "lowerneck", "upperneck", "head",
    "lclavicle", "lhumerus", "lradius",
    "lwrist", "lhand", "lfingers", "lthumb",
    "rclavicle", "rhumerus", "rradius",
    "rwrist", "rhand", "rfingers", "rthumb",
    nullptr // sentinel
};

// -----------------------------------------------------------------------------

static std::string skeletonFilePath;
static std::string motionFilePath;
static std::string File1Path;
static std::string File1LB = "linear"; // "linear" or "bezier"
static std::string File1EQ = "euler"; // "euler" or "quaternion"
static std::string File2Path;
static std::string File2LB = "linear"; // "linear" or "bezier"
static std::string File2EQ = "euler"; // "euler" or "quaternion"
static int inputN = 20; // default N = 20

static std::string chosenJoint = "lfemur";
static std::string chosenAxis = "x";
static int startFrame = 600;
static int endFrame = 800;

// display the output PNG
static Fl_Box* imageBox = nullptr;
static std::filesystem::path lastPlotPngPath; 

// -----------------------------------------------------------------------------
// Callback: "Select Skeleton File"
void cb_selectSkeletonFile(Fl_Widget*, void*) {
  // open file chooser
  Fl_File_Chooser chooser(".", "*", Fl_File_Chooser::SINGLE, "Select Skeleton File (.asf)");
  chooser.show();
  while (chooser.shown()) {
    Fl::wait();
  }
  // select file
  if (chooser.value() != nullptr) {
    skeletonFilePath = chooser.value();
    std::cout << "Skeleton file selected: " << skeletonFilePath << "\n";
  }
}

// Callback: "Select Motion File"
void cb_selectMotionFile(Fl_Widget*, void*) {
  // open file chooser
  Fl_File_Chooser chooser(".", "*", Fl_File_Chooser::SINGLE, "Select Motion File (.amc)");
  chooser.show();
  while (chooser.shown()) {
    Fl::wait();
  }
  // select file
  if (chooser.value() != nullptr) {
    motionFilePath = chooser.value();
    std::cout << "Motion file selected: " << motionFilePath << "\n";
  }
}

// Callback: "linear/bezier" choice
void cb_File1LB(Fl_Widget* w, void*) {
  Fl_Choice* c = dynamic_cast<Fl_Choice*>(w);
  if (c) {
    File1LB = c->text(); // "linear" or "bezier"
  }
}

void cb_File2LB(Fl_Widget* w, void*) {
  Fl_Choice* c = dynamic_cast<Fl_Choice*>(w);
  if (c) {
    File2LB = c->text(); // "linear" or "bezier"
  }
}

// Callback: "Euler/Quaternion" choice
void cb_File1EQ(Fl_Widget* w, void*) {
  Fl_Choice* c = dynamic_cast<Fl_Choice*>(w);
  if (c) {
    File1EQ = c->text(); // "euler" or "quaternion"
  }
}

void cb_File2EQ(Fl_Widget* w, void*) {
  Fl_Choice* c = dynamic_cast<Fl_Choice*>(w);
  if (c) {
    File2EQ = c->text(); // "euler" or "quaternion"
  }
}

// Callback: "N" input
void cb_inputN(Fl_Widget* w, void*) {
  Fl_Input* inp = dynamic_cast<Fl_Input*>(w);
  if (inp) {
    try {
      inputN = std::stoi(inp->value());
      std::cout << "N has been assigned to " << inputN << ".\n";
    } catch (...) {
      std::cerr << "Invalid N input.\n";
    }
  }
}

// Callback: "Generate" button => runs the interpolate command
void cb_File1Generate(Fl_Widget*, void*) {
  if (skeletonFilePath.empty()) {
    fl_message("Please select a skeleton file.");
    return;
  }
  if (motionFilePath.empty()) {
    fl_message("Please select a motion file.");
    return;
  }

  // Map "linear" -> "l", "bezier" -> "b"
  std::string LB = (File1LB == "linear") ? "l" : "b";
  // Map "euler" -> "e", "quaternion" -> "q"
  std::string EQ = (File1EQ == "euler") ? "e" : "q";

  // Build output filename:
  // e.g. if motionFilePath = "/path/to/motion.amc",
  // output name = "motion-l-e-20.amc"
  // and place it in "output/" folder
  std::filesystem::path mPath(motionFilePath);
  auto motionFilename = mPath.stem().string();
  std::ostringstream outName;
  outName << motionFilename 
          << "-" << LB 
          << "-" << EQ
          << "-" << inputN << ".amc";

  // output path
  std::filesystem::path outputDir = "output"; // output directory
  std::filesystem::create_directories(outputDir); // ensure "output/" exists
  auto outputPath = outputDir / outName.str();

  // Build command:
  // ./interpolate <skeletonFile> <motionFile> <b/l> <e/q> <N> <outputFile>
  std::ostringstream cmd;
  cmd << "./interpolate \"" << skeletonFilePath << "\" \"" << motionFilePath << "\" "
      << LB << " " << EQ << " " << inputN << " \""
      << outputPath.string() << "\"";

  std::cout << "Running command: " << cmd.str() << std::endl;

  int ret = std::system(cmd.str().c_str());
  if (ret == 0) {
    File1Path = outputPath;
    fl_message("Interpolation successful! Output file:\n%s", outputPath.string().c_str());
  } else {
    fl_message("Interpolation command failed.");
  }
}

// Callback: "Generate" button => runs the interpolate command
void cb_File2Generate(Fl_Widget*, void*) {
  if (skeletonFilePath.empty()) {
    fl_message("Please select a skeleton file.");
    return;
  }
  if (motionFilePath.empty()) {
    fl_message("Please select a motion file.");
    return;
  }

  // Map "linear" -> "l", "bezier" -> "b"
  std::string LB = (File2LB == "linear") ? "l" : "b";
  // Map "euler" -> "e", "quaternion" -> "q"
  std::string EQ = (File2EQ == "euler") ? "e" : "q";

  // Build output filename:
  // e.g. if motionFilePath = "/path/to/motion.amc",
  // output name = "motion-l-e-20.amc"
  // and place it in "output/" folder
  std::filesystem::path mPath(motionFilePath);
  auto motionFilename = mPath.stem().string();
  std::ostringstream outName;
  outName << motionFilename 
          << "-" << LB 
          << "-" << EQ
          << "-" << inputN << ".amc";

  // output path
  std::filesystem::path outputDir = "output"; // output directory
  std::filesystem::create_directories(outputDir); // ensure "output/" exists
  auto outputPath = outputDir / outName.str();

  // Build command:
  // ./interpolate <skeletonFile> <motionFile> <b/l> <e/q> <N> <outputFile>
  std::ostringstream cmd;
  cmd << "./interpolate \"" << skeletonFilePath << "\" \"" << motionFilePath << "\" "
      << LB << " " << EQ << " " << inputN << " \""
      << outputPath.string() << "\"";

  std::cout << "Running command: " << cmd.str() << std::endl;

  int ret = std::system(cmd.str().c_str());
  if (ret == 0) {
    File2Path = outputPath;
    fl_message("Interpolation successful! Output file:\n%s", outputPath.string().c_str());
  } else {
    fl_message("Interpolation command failed.");
  }
}

// -----------------------------------------------------------------------------
// Callback for "joint choice"
void cb_jointChoice(Fl_Widget *w, void*) {
  Fl_Choice* c = dynamic_cast<Fl_Choice*>(w);
  if (c) {
    chosenJoint = c->text();
  }
}

// Callback for "axis choice"
void cb_axisChoice(Fl_Widget *w, void*) {
  Fl_Choice* c = dynamic_cast<Fl_Choice*>(w);
  if (c) {
    chosenAxis = c->text();
  }
}

// Callback for "start frame" input
void cb_startFrame(Fl_Widget* w, void*) {
  Fl_Input* inp = dynamic_cast<Fl_Input*>(w);
  if (inp) {
    startFrame = std::stoi(inp->value());
  }
}

// Callback for "end frame" input
void cb_endFrame(Fl_Widget* w, void*) {
  Fl_Input* inp = dynamic_cast<Fl_Input*>(w);
  if (inp) {
    endFrame = std::stoi(inp->value());
  }
}

// -----------------------------------------------------------------------------
// This function builds a "legend" or "title" from the user’s picks:
static std::string makePlotTitle() {
  // Example requested:
  // "comparison of [linear or bezier of file2] [euler or quaternion of file2]
  //  and [linear or bezier of file3] [euler or quaternion of file3]
  //  for [select joint] joint, rotation around [angle] axis,
  //  frames [startFrame] - [endFrame], for N=[N], for [file1 name] input file"
  std::filesystem::path f1(File1Path);
  std::filesystem::path f2(File2Path);
  std::filesystem::path f3(motionFilePath);

  // Extract just filenames
  auto f1name = f1.filename().string();
  auto f2name = f2.filename().string();
  auto f3name = f3.filename().string();

  std::ostringstream oss;
  oss << "Comparison of " << File1LB << " " << File1EQ
      << " (" << f1name << ") and " << File2LB << " " << File2EQ
      << " (" << f2name << ") for " << chosenJoint << " joint, rotation around "
      << chosenAxis << " axis, frames " << startFrame << "-" << endFrame
      << ", N=" << inputN << ", using input file " << f3name;
  return oss.str();
}

// -----------------------------------------------------------------------------
// helper function to scale output image
Fl_Image* scaleToFit(Fl_Image* orig, int boxW, int boxH) {
  int iw = orig->w();
  int ih = orig->h();
  float scaleW = float(boxW) / float(iw);
  float scaleH = float(boxH) / float(ih);
  float scale  = (scaleW < scaleH) ? scaleW : scaleH; // minimum
  int newW = int(iw * scale);
  int newH = int(ih * scale);
  return orig->copy(newW, newH);
}

// Callback: RUN button
void cb_run(Fl_Widget*, void*) {
  if (File1Path.empty() || File2Path.empty() || motionFilePath.empty()) {
    fl_message("Please select all three AMC files first.");
    return;
  }
  if (chosenJoint.empty() || chosenAxis.empty()) {
    fl_message("Please select a joint and axis.");
    return;
  }
  if (startFrame <= 0 || endFrame <= 0 || endFrame < startFrame) {
    fl_message("Invalid frame range.");
    return;
  }

  // Build command line
  // "./graph file1Path file2Path file3Path lfemur x 600 800 N"
  std::ostringstream cmd;
  cmd << "./graph "
      << "\"" << File1Path << "\" " << File1LB << " " << File1EQ << " "
      << "\"" << File2Path << "\" " << File2LB << " " << File2EQ << " "
      << "\"" << motionFilePath << "\" "
      << chosenJoint << " " << chosenAxis << " " << inputN << " " 
      << startFrame << " " << endFrame;

  std::string command = cmd.str();
  std::cout << "Running command: " << command << std::endl;

  int ret = std::system(command.c_str());
  if (ret != 0) {
    fl_message("graph execution failed or returned non-zero.");
    return;
  }

  // Now, "graph.cpp" presumably created "plot_output.png" in the same directory
  // as the input AMC files. We'll guess it’s in the directory of file1. 
  std::filesystem::path p(motionFilePath);
  std::filesystem::path parentDir = p.parent_path();
  std::filesystem::path pngPath = "output/plot_output.png";

  // Let's load that PNG into the FLTK box
  if (!std::filesystem::exists(pngPath)) {
    fl_message("PNG file not found. Possibly the Python script failed?");
    return;
  }

  Fl_PNG_Image *img = new Fl_PNG_Image(pngPath.string().c_str());
  if (img->fail()) {
    delete img;
    fl_message("Failed to load PNG image.");
    return;
  }
  
  // copy path
  lastPlotPngPath = pngPath;

  // window display
  std::string windowTitle = makePlotTitle();
  imageBox->parent()->label(windowTitle.c_str());

  // Set the image in our imageBox
  // imageBox->image(img);
  Fl_Image* scaled = scaleToFit(img, imageBox->w(), imageBox->h());
  imageBox->image(scaled);
  imageBox->redraw();
  fl_message("Plot image updated!");
}

// Callback: save image
void cb_saveImage(Fl_Widget*, void*) {
  if (lastPlotPngPath.empty() || !std::filesystem::exists(lastPlotPngPath)) {
      fl_message("No image to save yet! Please generate a plot first.");
      return;
  }

  // Let user pick where to copy the PNG. We use CREATE mode so user can type a new filename.
  Fl_File_Chooser chooser(".", "*.png", Fl_File_Chooser::CREATE, "Save Image As...");
  chooser.directory(".");
  chooser.show();
  while (chooser.shown()) {
    Fl::wait();
  }
  if (chooser.value() != nullptr) {
    // Copy from lastPlotPngPath to the chosen path
    std::filesystem::path destPath = chooser.value();
    try {
      // Overwrite if user picks an existing file
      std::filesystem::copy_file(lastPlotPngPath, destPath, std::filesystem::copy_options::overwrite_existing);
      fl_message("Image saved to: %s", destPath.string().c_str());
    } catch (std::exception& e) {
      fl_message("Error copying file: %s", e.what());
    }
  } else {
    fl_message("Save canceled.");
  }
}

// -----------------------------------------------------------------------------
int main(int argc, char** argv) {
  // FLTK window
  Fl_Window *win = new Fl_Window(800, 400, "Motion Comparison Plotter");

  int x = 10; // left-edge position (in pixels) relative to the window
  int y = 10; // top-edge position (in pixels) relative to the window
  int w = 100; // width (in pixels)
  int h = 25; // height (in pixels)
  int spacing = 30;

  // ------ generate new motion files ------

  // "Select Skeleton File (.asf)"
  Fl_Button *btnSkeleton = new Fl_Button(x, y, w + 100, h, "Select Skeleton File (.asf)");
  btnSkeleton->callback(cb_selectSkeletonFile);
  y += spacing; // new line
  
  // "Select Motion File (.amc)"
  Fl_Button *btnMotion = new Fl_Button(x, y, w + 100, h, "Select Motion File (.amc)");
  btnMotion->callback(cb_selectMotionFile);
  y += spacing; // new line

  // "N" input
  Fl_Input *inputN = new Fl_Input(x + 40, y, w, h, "N:");
  inputN->callback(cb_inputN);
  y += spacing; // new line

  // ------ file 1 ------
  // "linear/bezier"
  Fl_Choice *File1LB = new Fl_Choice(x + 40, y, w, h, "File 1:");
  File1LB->add("linear");
  File1LB->add("bezier");
  File1LB->value(0); // default "linear"
  File1LB->callback(cb_File1LB);

  // "euler/quaternion"
  Fl_Choice *File1EQ = new Fl_Choice(x + 140, y, w, h, "");
  File1EQ->add("euler");
  File1EQ->add("quaternion");
  File1EQ->value(0); // default "euler"
  File1EQ->callback(cb_File1EQ);
  
  // Generate button
  Fl_Button *File1Generate = new Fl_Button(x + 240, y, w, h, "Generate");
  File1Generate->callback(cb_File1Generate);
  y += spacing;

  // ------ file 2 ------
  // "linear/bezier"
  Fl_Choice *File2LB = new Fl_Choice(x + 40, y, w, h, "File 2:");
  File2LB->add("linear");
  File2LB->add("bezier");
  File2LB->value(0); // default "linear"
  File2LB->callback(cb_File2LB);

  // "euler/quaternion"
  Fl_Choice *File2EQ = new Fl_Choice(x + 140, y, w, h, "");
  File2EQ->add("euler");
  File2EQ->add("quaternion");
  File2EQ->value(0); // default "euler"
  File2EQ->callback(cb_File2EQ);
  
  // Generate button
  Fl_Button *File2Generate = new Fl_Button(x + 240, y, w, h, "Generate");
  File2Generate->callback(cb_File2Generate);
  y += spacing;

  // ------ plot graph ------
  // select joint
  Fl_Choice *jointChoice = new Fl_Choice(x+40, y, w, h, "Joint:");
  for (int i = 0; JOINT_NAMES[i]; i++) {
    jointChoice->add(JOINT_NAMES[i]);
  }
  jointChoice->callback(cb_jointChoice);
  jointChoice->value(1); // e.g. "lfemur" by default
  y += spacing;

  // select axis
  Fl_Choice *axisChoice = new Fl_Choice(x+40, y, w, h, "Axis:");
  axisChoice->add("x");
  axisChoice->add("y");
  axisChoice->add("z");
  axisChoice->value(0); // "x" by default
  axisChoice->callback(cb_axisChoice);
  y += spacing;

  // [input start frame], 
  Fl_Input *startInp = new Fl_Input(x + 40, y, w, h, "Frame:");
  startInp->value("600"); // default
  startInp->callback(cb_startFrame);

  // [input end frame]
  Fl_Input *endInp = new Fl_Input(x + 150, y, w, h, "-");
  endInp->value("800"); // default
  endInp->callback(cb_endFrame);
  y += spacing;

  // RUN button
  Fl_Button *btnRun = new Fl_Button(x, y, w + 100, h, "Plot Graph");
  btnRun->callback(cb_run);
  y += spacing;

  // save image button
  Fl_Button *btnSaveImage = new Fl_Button(x, y, w + 100, h, "Save Image");
  btnSaveImage->callback(cb_saveImage);
  y += spacing;

  // A box for showing the resulting PNG
  imageBox = new Fl_Box(360, 10, 400, 300);
  imageBox->box(FL_FLAT_BOX);
  imageBox->labelsize(14);

  win->end();
  win->show(argc, argv);

  return Fl::run();
}