"""
Written by Tianqi Liu, 2020 Feb.

It finds the optimal path for a car using Hybrid A* and bicycle model.
"""

# import copy
import heapq
from math import sqrt, radians, degrees, cos, sin, tan
import matplotlib.pyplot as plt
# import numpy as np
try:
    from queue import PriorityQueue
except:
    from Queue import PriorityQueue
# from heapq import heappush, heappop


#possible steering controls
possible_str = {
    'l': -10,
    'l+': -50,
    'r+': +50,
    'r': +10,
    's': 0#,

    # 'l1': -20,
    # 'l1+': -30,
    # 'r1+': +20,
    # 'r1': +30,
    # 'l2': -40,
    # 'l2+': -60,
    # 'r2+': +40,
    # 'r2': +40
}

#possible speed controls
possible_sp = {
    'f': 1,
    'b': -1
}


# total cost f(n) = actual cost g(n) + heuristic cost h(n)
class hybrid_a_star:
    def __init__(self, min_x, max_x, min_y, max_y, \
            obstacle=[], resolution=1, vehicle_length=2):
        ##TODO
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.obstacles = frozenset(obstacle)
        self.vehicle_length = vehicle_length
        ###


    """
    For each node n, we need to store:
    (discret_x, discret_y, heading angle theta),
    (continuous x, continuous y, heading angle theta)
    cost g, f,
    path [(continuous x, continuous y, continuous theta),...]

    start: discret (x, y, theta)
    end: discret (x, y, theta)
    sol_path = [(x1,y1,theta1),(x2,y2,theta2), ...]
    """
    def find_path(self, start, end):
        sol_path = []
        ##TODO
        frontier = PriorityQueue()
        frontier.put((0,start))
        # frontier = []
        # heappush(frontier,(0,(start,)))
        g = {}
        g[start] = 0
        str_keys = sorted(possible_str.keys())
        sp_keys = sorted(possible_sp.keys())

        parent = {}
        parent[start] = None
        end_node = None

        while frontier:
            curr = frontier.get()[1]
            # curr = heappop(frontier)[1]

            x = curr[0]
            y = curr[1]
            theta = curr[2]
            l = self.vehicle_length
            curr_discrete = (round(x), round(y), theta)

            if(curr_discrete == end):
                end_node = curr
                break

            for angle in str_keys:
                for vel in sp_keys:
                    v = possible_sp[vel]
                    delta = possible_str[angle]
                    x_new = x + v*cos(radians(theta))
                    y_new = y + v*sin(radians(theta))
                    theta_new = round(theta + degrees((v/float(l))*tan(radians(delta))))
                    continous = (x_new, y_new, theta_new)
                    x_d = round(x_new)
                    y_d = round(y_new)
                    discrete = (x_d,y_d,theta_new) # get discrete cell for above continoues VALUES

                    if(self.min_x <= x_new <= self.max_x and self.min_y <= y_new <= self.max_y and not (x_d == curr_discrete[0] and y_d == curr_discrete[1]) ):
                        if((x_d,y_d) not in self.obstacles):
                            if discrete in g: # if already seen, but now seen with better cost
                                if(v == 1):
                                    cost_from_start = g[curr_discrete] + 20*abs(delta)/180.0 + 1
                                else:
                                    cost_from_start = g[curr_discrete] + 20*abs(delta)/180.0 + 5
                                # cost_from_start = g[curr_discrete] + 0.9-(v*0.1) + (1.5+(abs(delta)/float(180))) # TAKE into account steering difference
                                if(cost_from_start < g[discrete]):
                                    g[discrete] = cost_from_start  # finds g - cost from start
                                    h = sqrt(pow((x_d-end[0]),2) + pow((y_d-end[1]),2))   # finds heuristic - euclidian distance
                                    f = g[discrete] + h
                                    parent[continous] = curr
                                    frontier.put((f, continous))
                                    # heappush(frontier,(f, new_continous))

                            else:   # never seen before
                                if(v == 1):
                                    cost_from_start = g[curr_discrete] + 20*abs(delta)/180.0 + 1
                                else:
                                    cost_from_start = g[curr_discrete] + 20*abs(delta)/180.0 + 5
                                # cost_from_start = g[curr_discrete] + 0.9-(v*0.1) + (1.5+(abs(delta)/float(180))) # TAKE into account steering difference
                                g[discrete] = cost_from_start  # finds g - cost from start
                                h = sqrt(pow((x_d-end[0]),2) + pow((y_d-end[1]),2))    # finds heuristic - euclidian distance
                                f = g[discrete] + h
                                parent[continous] = curr
                                frontier.put((f, continous))
                                # heappush(frontier,(f, new_continous))
        backtrack = end_node
        while backtrack is not None:
            sol_path.append(backtrack)
            backtrack = parent[backtrack]
        sol_path.reverse()
        return sol_path


def main():
    print(__file__ + " start!!")

    # start and goal position
    #(x, y, theta) in meters, meters, degrees
    sx, sy, stheta= -5, -5, 0
    gx, gy, gtheta = 5, 5, 0

    #create obstacles
    obstacle = []

    for i in range(2):
        obstacle.append((0,i))
        obstacle.append((0,-i))

    ox, oy = [], []
    for (x,y) in obstacle:
        ox.append(x)
        oy.append(y)

    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "xr")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")

    hy_a_star = hybrid_a_star(-6, 6, -6, 6, obstacle=obstacle, \
        resolution=1, vehicle_length=2)
    path = hy_a_star.find_path((sx,sy,stheta), (gx,gy,gtheta))

    rx, ry = [], []
    for node in path:
        rx.append(node[0])
        ry.append(node[1])

    plt.plot(rx, ry, "-r")
    plt.show()


if __name__ == '__main__':
    main()
