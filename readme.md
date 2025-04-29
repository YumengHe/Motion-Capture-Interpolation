CSCI 520 Assignment 2: Motion Capture Interpolation

link: https://viterbi-web.usc.edu/~jbarbic/cs520-s25/assign2/

build fltk: 
cd fltk-1.3.11
make

compile:
cd mocapPlayter-starter
make

run mocapPlayer:
./mocapPlayer

run 
./graph-frontend



printf("Interpolates motion capture data.");
    printf("Usage: %s <input skeleton file> <input motion capture file> <interpolation type> <angle representation for interpolation> <N> <output motion capture file>\n", argv[0]);
    printf("  interpolation method:\n");
    printf("    l: linear\n");
    printf("    b: Bezier\n");
    printf("  angle representation for interpolation:\n");
    printf("    e: Euler angles\n");
    printf("    q: quaternions\n");
    printf("  N: number of skipped frames\n");
    printf("Example: %s skeleton.asf motion.amc l e 5 outputMotion.amc\n", argv[0]);  

./interpolate <input skeleton file> <input motion caputure file> <b/l> <e/q> <N> <ouput motion capture file>
./interpolate 07-walk.asf 07_05-walk.amc l e 5 output1.amc


./interpolate 07-walk.asf 07_05-walk.amc b e 5 output-walk-be5.amc

./mocapPlayer 


Three videos demonstrating your results (all three for 135_06-martialArts.amc, N=40):
Input motion and Bezier Euler (superimposed on top of each other)
Input motion and SLERP quaternion (superimposed on top of each other)
Input motion and Bezier SLERP quaternion (superimposed on top of each other)

Bezier Euler
./interpolate 135-martialArts.asf 135_06-martialArts.amc b e 40 ouput-martialArts-be40.amc

./graph 




1. Linear Euler vs Bezier Euler (N=20, 131_04-dance.amc)
```bash
./interpolate 131-dance.asf 131_04-dance.amc l e 20 comparison/compare1/output-dance-le20.amc
./interpolate 131-dance.asf 131_04-dance.amc b e 20 comparison/compare1/output-dance-be20.amc
./graph comparison/compare1/131_04-dance.amc L E comparison/compare1/output-dance-le20.amc B E comparison/compare1/output-dance-be20.amc lfemur x 600 800

./graph comparison/compare1/output-dance-le20.amc L E comparison/compare1/output-dance-be20.amc B E comparison/compare1/131_04-dance.amc lfemur x 20 600 800

./interpolate "/Users/rainyhe/Desktop/USC/CSCI520/assignments/A2/csci520-assignment2-startercode/mocapPlayer-starter/131-dance.asf" "/Users/rainyhe/Desktop/USC/CSCI520/assignments/A2/csci520-assignment2-startercode/mocapPlayer-starter/131_04-dance.amc" l e 20 "output/131_04-dance-l-e-20.amc"

./interpolate "/Users/rainyhe/Desktop/USC/CSCI520/assignments/A2/csci520-assignment2-startercode/mocapPlayer-starter/131-dance.asf" "/Users/rainyhe/Desktop/USC/CSCI520/assignments/A2/csci520-assignment2-startercode/mocapPlayer-starter/131_04-dance.amc" l q 20 "output/131_04-dance-l-q-20.amc"

./graph "output/131_04-dance-l-e-20.amc" linear euler "output/131_04-dance-l-q-20.amc" linear quaternion "/Users/rainyhe/Desktop/USC/CSCI520/assignments/A2/csci520-assignment2-startercode/mocapPlayer-starter/131_04-dance.amc" lfemur x 20 600 800
```
## Graph
Graph #1: Compares **linear Euler** to **Bezier Euler** interpolation (and input) for **lfemur** joint, rotation around **X** axis, frames **600-800**, for **N=20**, for **131_04-dance.amc** input file.
![Graph #1](<mocapPlayer-starter/comparison/image1.png>)

Graph #2: Compares **SLERP quaternion** to **Bezier SLERP quaternion** interpolation (and input) for **lfemur** joint, rotation around **X** axis, frames **600-800**, for **N=20**, for **131_04-dance.amc** input file.
![Graph #2](<mocapPlayer-starter/comparison/image2.png>)

Graph #3: Compares **linear Euler** to **SLERP quaternion** (and input) ​​for **root** joint, rotation around **Z** axis, frames **200-500**, for **N=20**, for **131_04-dance.amc** input file.
![Graph #3](<mocapPlayer-starter/comparison/image3.png>)

Graph #4: Compares **Bezier Euler** to **Bezier SLERP quaternion** (and input) for **root** joint, rotation around **Z** axis, frames **200-500**, for **N=20**, for **131_04-dance.amc** input file.
![Graph #4](<mocapPlayer-starter/comparison/image4.png>)

## Findings and Observation
Which techniques did you successfully implement? \
I successfully implemented Bezier interpolation for Euler angles, as well as SLERP and Bezier SLERP interpolations for quaternions.

How well does each of the techniques work? \
Bezier perform better than linear interpolation, and quaternion perform better than EUler angles.

What are the strengths and weaknesses of each technique?\
Bezier splines has C1 continuity, linear interpolation has C0 continuity.
Euler angles might facing gimbal lock, which quaternions representation can be avoid.

Include any additional findings, interesting observations or insights gained during this homework.