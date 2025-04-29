# CSCI 520 Assignment 2: Motion Capture Interpolation

**Link:** [CSCI 520 Assignment 2 Page](https://viterbi-web.usc.edu/~jbarbic/cs520-s25/assign2/)

This project implements motion capture interpolation techniques for an ASF/AMC mocap system. It also includes a motion comparison plotter tool that graphs selected joint angles over a specified frame range.

---

### Building FLTK
```bash
cd fltk-1.3.11
make
```

### Compiling the Project
```bash
cd mocapPlayter-starter
make
```

### Run the Mocap Player
```bash
./mocapPlayer
```

### Run the motion comparison plotter
I provide a separate executable and frontend for comparison/plotting.
```bash
./graph-frontend
```
It allows you to select multiple AMC files, a joint, an axis, a frame range, and then automatically generate plots comparing interpolation methods.

## Core Features

1. **Quaternion Representation**  
   - Implemented routines to convert between **Euler angles** and **quaternions**, enabling a robust, gimbal-lock-free rotation representation for all joints (including the root).

2. **Quaternion Interpolation (SLERP)**  
   - Developed **Slerp** functions to smoothly blend quaternions.

3. **Interpolation Methods**  
   - **Bezier (Euler)**: De Casteljau-based splines on Euler angles.  
   - **Linear + Slerp (Quaternion)**: Uses simple linear interpolation for positions and root translation, plus quaternion Slerp for rotations.  
   - **Bezier + Slerp (Quaternion)**: Combines De Casteljau construction with quaternion Slerp to yield smooth, continuous rotations.

4. **De Casteljau Implementations**  
   - **Euler De Casteljau**: Repeated linear interpolation on Euler angles.  
   - **Quaternion De Casteljau (Slerp-based)**: Replaced standard LERP with Slerp to remain on the unit quaternion sphere, ensuring smooth, gimbal-lock-free rotations.

## Extra Features

1. **Prettier OpenGL Renderer**  
    - Modified the color of the floor (light blue instead of yellow).  
    - Changed bone and joint colors; each skeleton has distinct colors.

2. **Screenshot Folder**  
    - Updated the screenshot functionality so that images are saved in a `screenshots` folder rather than the root directory.

3. **Computation Time Analysis**  
   - Measured and analyzed the performance of the different interpolation techniques.

4. **Graphing Utility**  
   - Implemented `graph.cpp` and `plot-graph.py` to automatically:
     - Parse AMC files and extract a specific joint angle (for a chosen axis and frame range).
     - Output to CSV and invoke Python plotting.
     - Usage:
       ```bash
       ./graph <File1Path> <File2Path> <File3Path> <jointName> <axis: x|y|z> <startFrame> <endFrame>
       ```
     - Automatically produces a PNG plot.

5. **Frontend AMC Generation**  
   - Allows selecting an ASF skeleton, an AMC file, and a parameter `N`.
   - Users choose between linear or Bezier, Euler or quaternion interpolation (default value has been enabled). 
   - Automatically generates a new AMC in the `output/` folder and can immediately graph it with a single button click.
   - The resulting plot can be saved under chosen directory with a user-chosen filename (e.g., `image1.png`).
   - Usage: `./graph-frontend`
   ![Graph #1](<mocapPlayer-starter/comparison/frontend.png>)

## Graph
**Graph #1:** Compares **linear Euler** to **Bezier Euler** interpolation (and input) for **lfemur** joint, rotation around **X** axis, frames **600-800**, for **N=20**, for **131_04-dance.amc** input file. \
Linear Euler computation time: 0.002259 \
Bezier Euler computation time: 0.020925
![Graph #1](<mocapPlayer-starter/comparison/image1.png>)

**Graph #2:** Compares **SLERP quaternion** to **Bezier SLERP quaternion** interpolation (and input) for **lfemur** joint, rotation around **X** axis, frames **600-800**, for **N=20**, for **131_04-dance.amc** input file. \
Linear Quaternion computation time: 0.019986 \
Bezier Quaternion computation time: 0.061128
![Graph #2](<mocapPlayer-starter/comparison/image2.png>)

**Graph #3:** Compares **linear Euler** to **SLERP quaternion** (and input) ​​for **root** joint, rotation around **Z** axis, frames **200-500**, for **N=20**, for **131_04-dance.amc** input file. \
Linear Euler computation time: 0.001789 \
Linear Quaternion computation time: 0.019898
![Graph #3](<mocapPlayer-starter/comparison/image3.png>)

**Graph #4:** Compares **Bezier Euler** to **Bezier SLERP quaternion** (and input) for **root** joint, rotation around **Z** axis, frames **200-500**, for **N=20**, for **131_04-dance.amc** input file. \
Bezier Euler computation time: 0.022329 \
Bezier Quaternion computation time: 0.060573 \
![Graph #4](<mocapPlayer-starter/comparison/image4.png>)

## Video
**Video #1:** Compare **input motion** and **Bezier Euler** for **N = 40**, for **131_06-martialArts.amc** input file. [Youtube Link](https://youtu.be/dnA_cB88-fw)
```bash
./interpolate 135-martialArts.asf 135_06-martialArts.amc b e 40 output/135_06-martialArts-b-e-40.amc
```

**Video #2:** Compare **input motion** and **SLERP quaternion** for **N = 40**, for **131_06-martialArts.amc** input file. [Youtube Link](https://youtu.be/RHFU74L93kI)
```bash
./interpolate 135-martialArts.asf 135_06-martialArts.amc l q 40 output/135_06-martialArts-l-q-40.amc
```

**Video #3:** Compare **input motion** and **Bezier SLERP quaternion** for **N = 40**, for **131_06-martialArts.amc** input file. [Youtube Link](https://youtu.be/eAM_PrUDZvM)
```bash
./interpolate 135-martialArts.asf 135_06-martialArts.amc b q 40 output/135_06-martialArts-b-q-40.amc
```


## Findings and Observations

**Which techniques did you successfully implement?**  
I successfully implemented **Bezier interpolation** for Euler angles, as well as **SLERP** and **Bezier SLERP** interpolations for quaternions.

**How well does each technique work?**  
- Bezier interpolation (Euler) generally produces smoother results than linear interpolation (Euler).  
- Quaternion-based interpolation (SLERP) avoids gimbal lock and often yields more natural motions than Euler-based methods.

**Strengths and weaknesses of each technique**  
- **Linear interpolation (Euler):**  
  - **Strength:** Very fast, simplest to implement.  
  - **Weakness:** Only C0 continuity; angles can jump abruptly, potential for gimbal lock.  
- **Bezier interpolation (Euler):**  
  - **Strength:** C1 continuity, smoother curves, better visual quality than linear.  
  - **Weakness:** More computation than linear; still susceptible to Euler angle limitations (possible gimbal lock).  
- **SLERP (Quaternion):**  
  - **Strength:** Avoids gimbal lock entirely; rotational interpolation is more robust.  
  - **Weakness:** Slightly higher computational cost than Euler methods.  
- **Bezier SLERP (Quaternion):**  
  - **Strength:** Best visual results, full C1 continuity on the unit quaternion sphere.  
  - **Weakness:** Most complex to implement; highest runtime overhead.

**Additional findings and insights**  
1. **Runtime comparison:**  
   - Linear Euler is the fastest.  
   - Bezier Euler is slower but visually smoother.  
   - Quaternion methods cost more compute time; Bezier SLERP is the slowest but yields the best rotational continuity.  

2. **Bezier vs. Catmull-Rom observation:**  
   - I realized that the internal control points for Bezier splines (P1, P2) is computed via Catmull-Rom–like tangent calculations, **an_dash** is equal to
   ```math
   m_n = \frac12(p_{n+1} - p_{n-1}).  
   ```
   
   - This makes standalone Catmull-Rom interpolation somewhat redundant in my implementation, since Bezier already provides a high-quality result and is more straightforward to integrate into the existing pipeline.