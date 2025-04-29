#!/usr/bin/env python

import sys
import pandas as pd
import matplotlib.pyplot as plt

if len(sys.argv) != 6:
  print("Usage: plot-graph.py <inputCSV> <outputPNG> <title> <file1name> <file2name>")
  sys.exit(1)

csv_path = sys.argv[1]
png_path = sys.argv[2]
plot_title = sys.argv[3]
file1_title = sys.argv[4]
file2_title = sys.argv[5]

# Read the CSV data
df = pd.read_csv(csv_path)

# Basic plot
plt.plot(df["frame"], df["angle_file1"], label=file1_title)
plt.plot(df["frame"], df["angle_file2"], label=file2_title)
plt.plot(df["frame"], df["angle_file3"], label="Input File")

plt.xlabel("Frame")
plt.ylabel("Angle (degrees)")
plt.title(plot_title)
plt.legend()

# Save directly as a PNG
plt.savefig(png_path, dpi=200)  # 200 dpi (or whatever you like)

print(f"Plot saved to {png_path}")