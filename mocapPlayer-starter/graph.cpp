#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <stdexcept>
#include <filesystem> // C++17
#include <cstdlib>    // for std::system

// A structure to hold rotation angles for each frame and each joint.
struct FrameData {
  // key: joint name, value: vector of angles
  std::map<std::string, std::vector<double>> jointAngles;
};

std::vector<FrameData> parseAMC(const std::string &filename)
{
  std::ifstream inFile(filename);
  if (!inFile.is_open()) {
    throw std::runtime_error("Could not open file: " + filename);
  }

  std::vector<FrameData> allFrames;
  std::string line;

  // Skip header lines until we find a numeric line (frame index).
  while (std::getline(inFile, line)) {
    if (line.empty()) continue;
    std::istringstream iss(line);
    int possibleFrameIndex;
    if (iss >> possibleFrameIndex) {
      // Found the first frame index. Rewind and break.
      inFile.seekg(-static_cast<long>(line.size()) - 1, std::ios::cur);
      break;
    }
  }

  int currentFrame = -1;
  while (true) {
    // Try reading frame index
    if (!std::getline(inFile, line)) {
      break; // EOF
    }
    if (line.empty()) {
      continue;
    }

    int frameIndex = -1;
    {
      std::istringstream iss(line);
      iss >> frameIndex;
    }
    if (frameIndex < 0) {
      break; // malformed line
    }
    currentFrame = frameIndex;

    // Expand allFrames if necessary
    if ((int)allFrames.size() < currentFrame) {
        allFrames.resize(currentFrame);
    }

    // Read joint lines until next frame index or EOF
    while (true) {
      std::streampos pos = inFile.tellg();
      if (!std::getline(inFile, line)) {
        break; // EOF
      }
      // check if line is a new frame index
      {
        std::istringstream iss(line);
        int nextFrameCheck;
        if (iss >> nextFrameCheck) {
          // next frame
          inFile.seekg(pos);
          break;
        }
      }

      // parse joint line
      std::istringstream jointLine(line);
      std::string jointName;
      jointLine >> jointName;
      std::vector<double> angles;
      double value;
      while (jointLine >> value) {
          angles.push_back(value);
      }
      allFrames[currentFrame - 1].jointAngles[jointName] = angles;
    }
  }

  inFile.close();
  return allFrames;
}

int main(int argc, char** argv) {
  if (argc != 13) {
    std::cerr << "Usage: " << argv[0]
              << " File1Path File1LB File1EQ File2Path File2LB File2EQ motionFilePath"
              << " <jointName> <axis: x|y|z> <N> <startFrame> <endFrame>\n";
    return 1;
  }

  std::string File1Path = argv[1];
  std::string File1LB = argv[2];
  std::string File1EQ = argv[3];
  std::string File2Path = argv[4];
  std::string File2LB = argv[5];
  std::string File2EQ = argv[6];
  std::string motionFilePath = argv[7];
  std::filesystem::path p(motionFilePath);
  std::string motionFileName = p.filename().string();
  std::string joint = argv[8];
  char axisChar = argv[9][0]; // x, y, or z
  int inputN = std::stoi(argv[10]);
  int startFrame = std::stoi(argv[11]);
  int endFrame = std::stoi(argv[12]);

  
  int axisIndex = -1;
  if (joint == "root") {
    if (axisChar == 'x') axisIndex = 3;
    else if (axisChar == 'y') axisIndex = 4;
    else if (axisChar == 'z') axisIndex = 5;
    else {
      std::cerr << "ERROR: axis must be x, y, or z.\n";
      return 1;
    }
  } else {
    if (axisChar == 'x') axisIndex = 0;
    else if (axisChar == 'y') axisIndex = 1;
    else if (axisChar == 'z') axisIndex = 2;
    else {
      std::cerr << "ERROR: axis must be x, y, or z.\n";
      return 1;
    }
  }
  

  // Parse the AMC files
  std::vector<FrameData> framesFile1, framesFile2, framesFile3;
  try {
    framesFile1 = parseAMC(File1Path);
    framesFile2 = parseAMC(File2Path);
    framesFile3 = parseAMC(motionFilePath);
  } catch (const std::exception &e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
    return 1;
  }

  // csv path
  std::filesystem::path csvPath = "output/plot_data.csv";

  // open the CSV for writing
  std::ofstream outCSV(csvPath);
  if (!outCSV.is_open()) {
    std::cerr << "ERROR: Could not create output CSV file at: " << csvPath << "\n";
    return 1;
  }

  // Write CSV header
  outCSV << "frame,angle_file1,angle_file2,angle_file3\n";

  // For each frame in [startFrame, endFrame], gather the angle data
  for (int f = startFrame; f <= endFrame; ++f) {
    // AMC is 1-based indexing
    int idx = f - 1;
    if (idx < 0) continue;
    if (idx >= (int)framesFile1.size() || idx >= (int)framesFile2.size() || idx >= (int)framesFile3.size()) {
      // out of range
      continue;
    }

    auto &data1 = framesFile1[idx].jointAngles;
    auto &data2 = framesFile2[idx].jointAngles;
    auto &data3 = framesFile3[idx].jointAngles;
    double angle1 = 0.0, angle2 = 0.0, angle3 = 0.0;

    if (data1.find(joint) != data1.end()) {
      const auto &angVec1 = data1.at(joint);
      if ((int)angVec1.size() > axisIndex) {
        angle1 = angVec1[axisIndex];
      }
    }

    if (data2.find(joint) != data2.end()) {
      const auto &angVec2 = data2.at(joint);
      if ((int)angVec2.size() > axisIndex) {
        angle2 = angVec2[axisIndex];
      }
    }

    if (data3.find(joint) != data3.end()) {
      const auto &angVec3 = data3.at(joint);
      if ((int)angVec3.size() > axisIndex) {
        angle3 = angVec3[axisIndex];
      }
    }

    outCSV << f << "," << angle1 << "," << angle2 << "," << angle3 << "\n";
  }

  outCSV.close();
  std::cout << "Data successfully written to: " << csvPath << "\n";

  // run python script
  std::filesystem::path pythonScriptPath = "plot-graph.py";
  std::filesystem::path pngPath = "output/plot_output.png";

  // plot title
  std::ostringstream oss;
  oss << File1LB << " " << File1EQ << " vs " << File2LB << " " << File2EQ
      << " for " << joint 
      << " joint around " << axisChar << " axis\n"
      << "frames "
      << startFrame << "-" << endFrame << ", " 
      << "N = " << inputN << ", input file " << motionFileName;
  std::string plotTitle = oss.str();

  // file1 title
  std::ostringstream oss1;
  oss1 << File1LB << " " << File1EQ;
  std::string file1Title = oss1.str();

  // file2 title
  std::ostringstream oss2;
  oss2 << File2LB << " " << File2EQ;
  std::string file2Title = oss2.str();

  // build a command string to run the python script.
  // e.g. "python /path/to/plot-graph.py /path/to/plot_data.csv /path/to/plot_output.png"
  std::string command = std::string("python \"")
                      + pythonScriptPath.string() + "\" \""
                      + csvPath.string() + "\" \""
                      + pngPath.string() + "\" \""
                      + plotTitle + "\" \""
                      + file1Title + "\" \""
                      + file2Title + "\"";

  std::cout << "Executing: " << command << std::endl;
  int retVal = std::system(command.c_str());

  if (retVal == 0) {
      std::cout << "Plot generated successfully: " << pngPath << "\n";
  } else {
      std::cout << "WARNING: Could not run Python script. Return code = "
                << retVal << "\n";
  }

  return 0;
}