import copy
import heapq
import math
import matplotlib.pyplot as plt
import numpy as np
try:
    from queue import PriorityQueue
except:
    from Queue import PriorityQueue

# car state = (x,y)
# state tuple (f,g,(x,y), [(x1,y1),(x2,y2)...])
# total cost f(n) = actual cost g(n) + heuristic cost h(n)
# obstacles = [(x,y), ...]
# min_x, max_x, min_y, max_y are the boundaries of the environment
class a_star:
    def __init__(self, min_x, max_x, min_y, max_y, \
            obstacle=[], resolution=1, robot_size=1):
    ##TODO
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.obstacles = frozenset(obstacle)
    ####

    # state: (total cost f, previous cost g, current position (x,y), \
    # previous motion id, path[(x1,y1),...])
    # start = (sx, sy)
    # end = (gx, gy)
    # sol_path = [(x1,y1),(x2,y2), ...]
    def find_path(self, start, end):
        sol_path = []
        ##TODO
        q = PriorityQueue()
        costs = {} #acts as frontier that keeps track of cost
        costs[start] = 0
        h = math.sqrt((start[0]-end[0])**2 + (start[1]-end[1])**2)
        f = costs[start] + h
        q.put((f,start))
        parent = {}
        parent[start] = None
        end_node = None

        while q:
            p = q.get()[1]
            if(p == end):
                end_node = p
                break
            for n_x in range(int(p[0]-1),int(p[0]+2)):
                for n_y in range(int(p[1]-1),int(p[1]+2)):
                    if(self.min_x <= n_x <= self.max_x and self.min_y <= n_y <= self.max_y and not (n_x == p[0] and n_y == p[1]) ):
                        if((n_x,n_y) not in self.obstacles):
                            new_p = (n_x,n_y)    # adds discovered neighbor to path
                            cost_from_start = costs[p] + 1  # current cost + 1
                            if new_p in costs: # if already seen, but now seen with better cost
                                if(cost_from_start < costs[(n_x,n_y)]):
                                    costs[(n_x,n_y)] = cost_from_start  # finds g - cost from start
                                    h = math.sqrt((new_p[0]-end[0])**2 + (new_p[1]-end[1])**2)    # finds heuristic - euclidian distance
                                    f = costs[(n_x,n_y)] + h
                                    parent[new_p] = p
                                    q.put((f, new_p))
                            else:   # never seen before
                                costs[(n_x,n_y)] = cost_from_start  # finds g - cost from start
                                h = math.sqrt((new_p[0]-end[0])**2 + (new_p[1]-end[1])**2)    # finds heuristic - euclidian distance
                                f = costs[(n_x,n_y)] + h
                                parent[new_p] = p
                                q.put((f, new_p))
        backtrack = end_node
        while backtrack is not None:
            sol_path.append(backtrack)
            backtrack = parent[backtrack]
        sol_path.reverse()
        return sol_path


def main():
    print(__file__ + " start!!")

    grid_size = 1  # [m]
    robot_size = 1.0  # [m]

    # ORIGINAL VALUES
    sx, sy = -10, -10
    gx, gy = 10, 10

    # TEST VALUES
    # sx, sy = -10, -10
    # gx, gy = 9, -10

    obstacle = []
    for i in range(30):
        obstacle.append((i-15, -15))
        obstacle.append((i-14, 15))
        obstacle.append((-15, i-14))
        obstacle.append((15, i-15))

    for i in range(3):
        obstacle.append((0,i))
        obstacle.append((0,-i))

    plt.plot(sx, sy, "xr")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")

    simple_a_star = a_star(-15, 15, -15, 15, obstacle=obstacle, \
        resolution=grid_size, robot_size=robot_size)
    path = simple_a_star.find_path((sx,sy), (gx,gy))
    print (path)

    rx, ry = [], []
    for node in path:
        rx.append(node[0])
        ry.append(node[1])

    ## ADDED FOR DEBUGGING
    ox, oy = [], []
    for o in obstacle:
        ox.append(o[0])
        oy.append(o[1])
    plt.plot(ox, oy, "xb")

    # to plot frontier (all visited) need to plot keys from cost dict
    ####

    plt.plot(rx, ry, "-r")

    plt.show()


if __name__ == '__main__':
    main()
