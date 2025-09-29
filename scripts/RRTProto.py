import random, math
from matplotlib import pyplot as plt
import matplotlib.patches as patches
import time


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def dist_to_point(self, point):
        return math.sqrt(math.pow(self.x - point.x, 2) + math.pow(self.y - point.y, 2))

class Obstacle:
    def __init__(self, pointbl, width, height):
        self.width = width
        self.height = height
        self.bl = pointbl
        self.br = Point(pointbl.x + width, pointbl.y)
        self.tl = Point(pointbl.x, pointbl.y + height)
        self.tr = Point(pointbl.x + width, pointbl.y + height)

    def check_collision(self, point):
        if (point.x > self.bl.x and point.x < self.br.x and point.y > self.bl.y and point.y < self.tl.y):
            return True
        return False

class Node:
    def __init__(self, parent, point):
        self.parent = parent
        self.point = point
        self.children = []
        self.calculate_cost()

    def calculate_cost(self):
        self.cost = (self.parent.cost + self.point.dist_to_point(self.parent.point)) if self.parent != None else 0
    
    def propogate_cost(self):
        for child in self.children:
            child.calculate_cost()
            child.propogate_cost()
    
    def dist_to_point(self, point):
        return(math.sqrt(math.pow(self.point.x - point.x, 2) + math.pow(self.point.y - point.y, 2)))
    
    def __str__(self):
        if self.parent == None:
            return f"(,) -> ({self.point.x:.1f}, {self.point.y:.1f})"
        return f"({self.parent.point.x:.1f}, {self.parent.point.y:.1f}) -> ({self.point.x:.1f}, {self.point.y:.1f})"

class Tree:
    def __init__(self, start, goal, obstacles):
        self.root = Node(None, start)
        self.goal = goal
        self.max_dist = 30
        self.node_list = [self.root]
        self.obstacles = obstacles

    def nodes_in_radius(self, radius, point):
        nearby_nodes = []
        for node in self.node_list:
            if node.dist_to_point(point) <= radius:
                nearby_nodes.append(node)
        return nearby_nodes

    def iteration(self):
        # Random point
        rand_point = self.goal if random.random() < 0.08 else Point(random.uniform(-1000, 1000), random.uniform(-1000, 1000))

        # Nearest Node
        nearest_node = self.node_list[0]
        dist_min = nearest_node.dist_to_point(rand_point)
        for node in self.node_list:
            if node.dist_to_point(rand_point) < dist_min:
                dist_min = node.dist_to_point(rand_point) 
                nearest_node = node

        # New normalized node
        dx = rand_point.x - nearest_node.point.x
        dy = rand_point.y - nearest_node.point.y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 1e-9:
            return False
        new_point = Point(
            dx / dist * self.max_dist + nearest_node.point.x,
            dy / dist * self.max_dist + nearest_node.point.y,
        )

        # Collision Check
        for obstacle in self.obstacles:
            if obstacle.check_collision(new_point):
                return False

        # Find nearby nodes
        r = max(self.max_dist * 2, self.max_dist * math.sqrt(math.log(len(self.node_list)) / len(self.node_list)))
        nearby_nodes = self.nodes_in_radius(r, new_point)

        # Choose best parent
        best_parent = nearest_node
        best_cost = nearest_node.cost + nearest_node.dist_to_point(new_point)
        for node in nearby_nodes:
            cost = node.cost + node.dist_to_point(new_point)
            if cost < best_cost:
                best_parent = node
                best_cost = cost

        # Create and wire new node
        new_node = Node(best_parent, new_point)
        best_parent.children.append(new_node)
        self.node_list.append(new_node)

        # Rewire other nodes within radius
        r = self.max_dist * 3
        nearby_nodes = self.nodes_in_radius(r, new_point)
        for node in nearby_nodes:
            if node.cost > new_node.cost + new_node.dist_to_point(node.point):
                for child in node.parent.children:
                    if child.point.x == node.point.x and child.point.y == node.point.y:
                        node.parent.children.remove(child)
                node.parent = new_node
                node.calculate_cost()
                node.propogate_cost()

        # Check if goal has been reached
        if new_node.dist_to_point(self.goal) <= self.max_dist:
            final_node = Node(new_node, self.goal)
            self.node_list.append(final_node)
            return True
        
        return False


start = Point(100, 0)
goal = Point(-750, -250)#Point(-750, 300)
obstacle1 = Obstacle(Point(-400, -500), 300, 1000)
obstacle2 = Obstacle(Point(-200, -500), 900, 100)  
obstacle3 = Obstacle(Point(-150, 500), 800, 100)
obstacles = [obstacle1, obstacle2, obstacle3]

ts = time.process_time()
iter_list = []
for i in range(1):
    t = Tree(start, goal, obstacles)
    iter = 0
    found = False
    while iter < 8000:# and not found:
        found_this_iter = t.iteration()
        if found_this_iter:
            found = True
        iter += 1
        pass
    iter_list.append(iter)
    print(iter)

time_elapsed = time.process_time() - ts
print(time_elapsed)
print(len(t.node_list))

# n_bins = 20
# fig, axs = plt.subplots()
# # We can set the number of bins with the *bins* keyword argument.
# axs.hist(iter_list, bins=n_bins)
# plt.show()

fig, ax = plt.subplots()
plt.axis([-1000, 1000, -1000, 1000])

for obstacle in obstacles:
    ax.add_patch(patches.Rectangle((obstacle.bl.x, obstacle.bl.y), obstacle.width, obstacle.height))

ax.plot(start.x, start.y, 'ro')
ax.plot(goal.x, goal.y, 'bo')
for i in t.node_list:
    plt.plot([i.parent.point.x if i.parent != None else start.x, i.point.x], [i.parent.point.y if i.parent != None else start.y, i.point.y], color='black')

# best_node = t.node_list[0]
# best_distance = best_node.dist_to_point(goal)
# for i in t.node_list:
#     distance = i.dist_to_point(goal)
#     if distance < best_distance:
#         best_node = i
#         best_distance = distance


nodes_near_goal = t.nodes_in_radius(50, goal)
best_distance = nodes_near_goal[0].cost
best_node = nodes_near_goal[0]
for node in nodes_near_goal:
    print(node)
    if node.cost < best_distance:
        best_node = node
        best_distance = node.cost

node = best_node #t.node_list[-1]
while node.parent != None:
    plt.plot([node.parent.point.x, node.point.x], [node.parent.point.y, node.point.y], color='red')
    node = node.parent

plt.show()