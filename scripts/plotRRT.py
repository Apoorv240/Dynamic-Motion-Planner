from matplotlib import pyplot as plt
import matplotlib.patches as patches

fig, ax = plt.subplots()
plt.axis([-182.88, 182.88, -182.88, 182.88])
ax.plot(-170, 0, 'ro')
ax.plot(170, 0, 'bo')

f = open("out.txt", "r")
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
    ax.add_patch(polygon)

for line in lines[13:]:
    points = line.split(" ")
    ax.plot([float(points[0]), float(points[2])], [float(points[1]), float(points[3])], color="black")

plt.show()