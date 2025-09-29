from matplotlib import pyplot as plt

fig, ax = plt.subplots()
plt.axis([-1000, 1000, -1000, 1000])

ax.plot(0, 0, 'ro')
ax.plot(900, 900, 'bo')

f = open("./scripts/out.txt", "r")
lines = f.readlines()[1:]
for line in lines:
    points = line.split(" ")
    ax.plot([float(points[0]), float(points[2])], [float(points[1]), float(points[3])])

plt.show()