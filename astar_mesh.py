import math
import random
from   queue import PriorityQueue
from   config import *
import config as G
from   utils import *


def astar_mesh(start, goal, pos, dest):
    """ A* search for Delaunay mesh. """
    path1, path1_len = build_path(start, goal, pos, dest)
    path2, path2_len = build_path(goal, start, dest, pos)
    if path1_len < path2_len:
        path1.reverse()
        return path1
    else:
        return path2


def build_path(start, goal, pos, dest):
    """ Build path from pos to dest. """
    path = _astar(start, goal, pos, dest)
    for _ in range(10):
        smooth_path(path, dest, pos)
    # convert path nodes to world coordinates
    path = [e.data['cross'].copy() for e in path]
    # get path length
    path_len = 0
    prev = dest
    for point in path:
        path_len += distance(prev, point)
        prev = point
    if path:
        prev = path[-1]
    path_len += distance(prev, pos)
    return path, path_len


def smooth_path(path, pos, dest):
    """ Smooth path trajectory. """
    if len(path) > 1:
        path[0].data['cross'] = smooth_point(path[0], path[1].data['cross'], pos)
        if len(path) > 2:
            for i in range(1, len(path)-1):
                path[i].data['cross'] = smooth_point(path[i], \
                    path[i-1].data['cross'], path[i+1].data['cross'])
        path[-1].data['cross'] = smooth_point(path[-1], path[-2].data['cross'], dest)
    elif len(path) == 1:
        path[0].data['cross'] = smooth_point(path[0], pos, dest)


def smooth_point(edge, A, B):
    """ Move crossing point of edge as close as possible to line AB. """
    p = lines_inter(edge.org, edge.dest, A, B)
    d1 = edge.data['org_r'] + AGENT_RADIUS // 2
    d2 = edge.data['dest_r'] + AGENT_RADIUS // 2
    if distance(p, edge.org) < d1:
        p = [edge.org[0] + d1 * math.cos(edge.data['ang']), \
             edge.org[1] + d1 * math.sin(edge.data['ang'])]
    elif distance(p, edge.dest) < d2:
        p = [edge.sym.org[0] + d2 * math.cos(edge.sym.data['ang']), \
             edge.sym.org[1] + d2 * math.sin(edge.sym.data['ang'])]
    return p


def _astar(start, goal, pos, dest):
    """ A* search for Delaunay mesh. """
    frontier = PriorityQueue()
    came_from = {}
    cost_so_far = {}
    explored = {}
    # initialize
    for i in range(len(start)):
        start[i].data['cross'] = cross(pos, start[i])
        start[i].sym.data['cross'] = start[i].data['cross']
        frontier.put((0, random.random(), start[i]))
        came_from[start[i]] = start[i]
        cost_so_far[start[i]] = 0
    # search
    while True:
        while True:                     # pop queue until unexplored node
            if frontier.empty():        # there is no path
                return [] 
            current = frontier.get()[2]
            if current not in explored:
                break
        for i in range(len(goal)):
            if current == goal[i]:      # path found
                return get_path(came_from, start, goal[i])
        explored[current] = True        # mark node as explored
        for next_node in neighbors(current):
            if next_node in explored:   # skip all explored nodes
                continue
            tmp = next_node.data['cross']
            next_node.data['cross'] = cross(current.data['cross'], next_node)
            next_node.sym.data['cross'] = next_node.data['cross']
            new_cost = cost_so_far[current] + cost(current, next_node)
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                cost_so_far[next_node.sym] = new_cost
                priority = new_cost + manhattan_dist(next_node, dest)
                frontier.put((priority, random.random(), next_node))
                came_from[next_node] = current
                came_from[next_node.sym] = current
            else:
                next_node.data['cross'] = tmp
                next_node.sym.data['cross'] = tmp


def cross(P, edge):
    """ Find closest edge crossing point with respect to P. """
    d = distance(edge.org, P)
    a = ang_diff(edge.data['ang'], math.atan2(P[1] - edge.org[1], P[0] - edge.org[0]))
    dA = edge.data['org_r'] + AGENT_RADIUS
    dB = edge.data['len'] - edge.data['dest_r'] - AGENT_RADIUS
    A = [dA, 0]
    B = [dB, 0]
    P_ = [d * math.cos(a), d * math.sin(a)]
    # return closest point
    if P_[0] < A[0]:
        return [int(round(edge.org[0] + dA * math.cos(edge.data['ang']))), \
                int(round(edge.org[1] + dA * math.sin(edge.data['ang'])))]
    elif P_[0] > B[0]:
        return [int(round(edge.org[0] + dB * math.cos(edge.data['ang']))), \
                int(round(edge.org[1] + dB * math.sin(edge.data['ang'])))]
    else:
        return [int(round(edge.org[0] + P_[0] * math.cos(edge.data['ang']))), \
                int(round(edge.org[1] + P_[0] * math.sin(edge.data['ang'])))]


def get_path(came_from, start, goal):
    """ Backtrack path from goal to start. """
    current = goal
    path = []
    while not (current in start):
        path.append(current)
        current = came_from[current]
    else:
        path.append(current)
    return path


def neighbors(node):
    """ List of mesh node neighbors, excluding walls. """
    res = set(e for e in [node.onext, node.oprev, node.sym.onext, node.sym.oprev] \
                if not e.data['wall'])
    return res


def cost(node_from, node_to):
    """ Assumes nodes are neighbors. """
    return distance(node_from.data['cross'], node_to.data['cross'])


def manhattan_dist(p1, p2):
    """ Manhattan distance between p1 and p2. """
    dx = abs(p1.data['cross'][0] - p2[0])
    dy = abs(p1.data['cross'][1] - p2[1])
    return int(round(dx + dy))
