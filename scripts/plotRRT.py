from matplotlib import pyplot as plt
import matplotlib.patches as patches

fig, axes = plt.subplots(1, 2)

ax1 = axes[0]
#plt.axis([-182.88, 182.88, -182.88, 182.88])
ax1.set_xlim(-182.88, 182.88)
ax1.set_ylim(-182.88, 182.88)

f = open("outPath.txt", "r")
lines = f.readlines()
for i in range(0, 12, 4):
    vertices = [
        (float(lines[i+0].split(" ")[0]), float(lines[i+0].split(" ")[1])),
        (float(lines[i+1].split(" ")[0]), float(lines[i+1].split(" ")[1])),
        (float(lines[i+2].split(" ")[0]), float(lines[i+2].split(" ")[1])),
        (float(lines[i+3].split(" ")[0]), float(lines[i+3].split(" ")[1]))
    ]
    print(i)
    polygon = patches.Polygon(vertices, closed=True, edgecolor='black', facecolor='skyblue')
    ax1.add_patch(polygon)

for line in lines[13:]:
    points = line.split(" ")
    ax1.plot([float(points[0]), float(points[2])], [float(points[1]), float(points[3])], color="black")

#fig, ax = plt.subplots()
# plt.axis([-182.88, 182.88, -182.88, 182.88])

'''
ax2 = axes[1]
ax2.set_xlim(-182.88, 182.88)
ax2.set_ylim(-182.88, 182.88)
ax2.plot(-170, 0, 'ro')
ax2.plot(100, 0, 'bo')

f = open("outAll.txt", "r")
lines = f.readlines()
for i in range(0, 12, 4):
    vertices = [
        (float(lines[i+0].split(" ")[0]), float(lines[i+0].split(" ")[1])),
        (float(lines[i+1].split(" ")[0]), float(lines[i+1].split(" ")[1])),
        (float(lines[i+2].split(" ")[0]), float(lines[i+2].split(" ")[1])),
        (float(lines[i+3].split(" ")[0]), float(lines[i+3].split(" ")[1]))
    ]
    print(i)
    polygon = patches.Polygon(vertices, closed=True, edgecolor='black', facecolor='skyblue')
    ax2.add_patch(polygon)

for line in lines[13:]:
    points = line.split(" ")
    ax2.plot([float(points[0]), float(points[2])], [float(points[1]), float(points[3])], color="black")
'''

ax2 = axes[1]
ax2.set_xlim(-182.88, 182.88)
ax2.set_ylim(-182.88, 182.88)
f = open("outSpline.txt", "r")
lines = f.readlines()

for i in range(0, 12, 4):
    vertices = [
        (float(lines[i+0].split(" ")[0]), float(lines[i+0].split(" ")[1])),
        (float(lines[i+1].split(" ")[0]), float(lines[i+1].split(" ")[1])),
        (float(lines[i+2].split(" ")[0]), float(lines[i+2].split(" ")[1])),
        (float(lines[i+3].split(" ")[0]), float(lines[i+3].split(" ")[1]))
    ]
    print(i)
    polygon = patches.Polygon(vertices, closed=True, edgecolor='black', facecolor='skyblue')
    ax2.add_patch(polygon)

for line in lines[13:]:
    points = line.split(" ")
    print(points[0])
    ax2.plot(0,0,'ro')
    ax2.plot(float(points[0]), float(points[1]), 'ro', color="black")

plt.tight_layout()
plt.show()