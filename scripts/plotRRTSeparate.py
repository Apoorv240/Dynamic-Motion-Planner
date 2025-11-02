from matplotlib import pyplot as plt
import matplotlib.patches as patches
import subprocess
import math

subprocess.run(['../testAll.exe'], cwd="../")

# ----------------------
# First figure (outPath.txt)
# ----------------------
fig1, ax1 = plt.subplots()
ax1.set_xlim(-182.88, 182.88)
ax1.set_ylim(-182.88, 182.88)

with open("outPath.txt", "r") as f:
    lines = f.readlines()

# Draw polygons
for i in range(0, 12, 4):
    vertices = [
        (float(lines[i+0].split(" ")[0]), float(lines[i+0].split(" ")[1])),
        (float(lines[i+1].split(" ")[0]), float(lines[i+1].split(" ")[1])),
        (float(lines[i+2].split(" ")[0]), float(lines[i+2].split(" ")[1])),
        (float(lines[i+3].split(" ")[0]), float(lines[i+3].split(" ")[1]))
    ]
    polygon = patches.Polygon(vertices, closed=True, edgecolor='black', facecolor='skyblue')
    ax1.add_patch(polygon)

# Draw lines
for line in lines[13:]:
    points = line.split(" ")
    ax1.plot([float(points[0]), float(points[2])], [float(points[1]), float(points[3])], color="black")

# ----------------------
# Second figure (outAll.txt)
# ----------------------
fig3, ax3 = plt.subplots()
ax3.set_xlim(-182.88, 182.88)
ax3.set_ylim(-182.88, 182.88)

f = open("outAll.txt", "r")
lines = f.readlines()
for i in range(0, 12, 4):
    vertices = [
        (float(lines[i+0].split(" ")[0]), float(lines[i+0].split(" ")[1])),
        (float(lines[i+1].split(" ")[0]), float(lines[i+1].split(" ")[1])),
        (float(lines[i+2].split(" ")[0]), float(lines[i+2].split(" ")[1])),
        (float(lines[i+3].split(" ")[0]), float(lines[i+3].split(" ")[1]))
    ]
    polygon = patches.Polygon(vertices, closed=True, edgecolor='black', facecolor='skyblue')
    ax3.add_patch(polygon)

for line in lines[13:]:
    points = line.split(" ")
    ax3.plot([float(points[0]), float(points[2])], [float(points[1]), float(points[3])], color="black")


# ----------------------
# Second figure (outSpline.txt)
# ----------------------
fig2, ax2 = plt.subplots()
ax2.set_xlim(-182.88, 182.88)
ax2.set_ylim(-182.88, 182.88)

with open("outSpline.txt", "r") as f:
    lines = f.readlines()

# Draw polygons
for i in range(0, 12, 4):
    vertices = [
        (float(lines[i+0].split(" ")[0]), float(lines[i+0].split(" ")[1])),
        (float(lines[i+1].split(" ")[0]), float(lines[i+1].split(" ")[1])),
        (float(lines[i+2].split(" ")[0]), float(lines[i+2].split(" ")[1])),
        (float(lines[i+3].split(" ")[0]), float(lines[i+3].split(" ")[1]))
    ]
    polygon = patches.Polygon(vertices, closed=True, edgecolor='black', facecolor='skyblue')
    ax2.add_patch(polygon)

# Collect all points
x_spline_list = []
y_spline_list = []

for line in lines[13:]:
    points = line.split(" ")
    x_spline_list.append(float(points[0]))
    y_spline_list.append(float(points[1]))

# Draw as connected line segments (polyline)
ax2.plot(x_spline_list, y_spline_list, color="black", linestyle='-', linewidth=2)

# Optional: draw nodes as small dots
ax2.scatter(x_spline_list, y_spline_list, color="red", s=10)

# ----------------------
# Fourth figure (outSpline.txt)
# ----------------------
fig4, ax4 = plt.subplots()
ax4.set_xlim(-182.88, 182.88)
ax4.set_ylim(-182.88, 182.88)

with open("outGVF.txt", "r") as f:
    lines = f.readlines()

# Draw polygons
for i in range(0, 12, 4):
    vertices = [
        (float(lines[i+0].split(" ")[0]), float(lines[i+0].split(" ")[1])),
        (float(lines[i+1].split(" ")[0]), float(lines[i+1].split(" ")[1])),
        (float(lines[i+2].split(" ")[0]), float(lines[i+2].split(" ")[1])),
        (float(lines[i+3].split(" ")[0]), float(lines[i+3].split(" ")[1]))
    ]
    polygon = patches.Polygon(vertices, closed=True, edgecolor='black', facecolor='skyblue')
    ax4.add_patch(polygon)

# Collect all points
x_list = []
y_list = []
dx_list = []
dy_list = []

for line in lines[13:]:
    points = line.split(" ")
    x_list.append(float(points[0]))
    y_list.append(float(points[1]))
    dx_list.append(float(points[2])/(math.sqrt(float(points[2])*float(points[2]) + float(points[3])*float(points[3]))+0.0000001))
    dy_list.append(float(points[3])/(math.sqrt(float(points[2])*float(points[2]) + float(points[3])*float(points[3]))+0.0000001))

ax4.quiver(x_list, y_list, dx_list, dy_list, angles='xy', scale_units='xy', scale=0.1, color='blue')
# Draw as connected line segments (polyline)
#ax4.plot(x_spline_list, y_spline_list, color="black", linestyle='-', linewidth=2)

# ----------------------
# Show both figures
# ----------------------
plt.show()