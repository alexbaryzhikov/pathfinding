from   queue import PriorityQueue
from   config import *
import config as G


def astar_grid(start, goal):
    """ A* search for rectangular grid. """
    frontier = PriorityQueue()
    came_from = {}
    cost_so_far = {}
    explored = {}
    # initialize
    frontier.put((0, start))
    came_from[start] = start
    cost_so_far[start] = 0
    # search
    while True:
        while True:                     # pop queue until unexplored node
            if frontier.empty():        # there is no path
                return [] 
            current = frontier.get()[1]
            if current not in explored:
                break
        if current == goal:             # path found
            return get_path(came_from, start, goal)
        explored[current] = True        # mark node as explored
        for next_node in neighbors(current):
            if next_node in explored:   # skip all explored nodes
                continue
            new_cost = cost_so_far[current] + cost(current, next_node)
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + manhattan_dist(next_node, goal)
                frontier.put((priority, next_node))
                came_from[next_node] = current


def get_path(came_from, start, goal):
    """ Backtrack path from goal to start. """
    current = goal
    path = []
    current = came_from[current]
    while current != start:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


def neighbors(node):
    """ List of grid node neighbors. """
    res = set((node[0] + x, node[1] + y) \
        for x, y in [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)])
    res -= G.grid.walls | G.grid.outer_walls
    return res


def cost(node_from, node_to):
    """ Assumes nodes are neighbors. """
    move_cost = 0
    if (node_from[0] == node_to[0]) or (node_from[1] == node_to[1]):
        move_cost = MOVE_COST
    else:
        move_cost = MOVE_COST_DIAG
    return move_cost + G.grid.nodes[node_to]


def manhattan_dist(p1, p2):
    """ Manhattan distance between p1 and p2. """
    dx = abs(p1[0] - p2[0])
    dy = abs(p1[1] - p2[1])
    if dx > dy:
        dx, dy = dy, dx
    return dx * MOVE_COST_DIAG + (dy - dx) * MOVE_COST
