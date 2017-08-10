import pygame
from   pygame.locals import *
import math
import numpy as np
import random
from   delaunay import delaunay, left_of
from   astar_mesh import astar_mesh
from   astar_grid import astar_grid
from   utils import *
from   config import *
import config as G


class World:
    def __init__(self):
        self.obstacles = None
        self.agent = pygame.sprite.GroupSingle(Agent((100, 100)))
        self.wp_mark = pygame.sprite.GroupSingle()

    def update(self, dt):
        self.obstacles.update(dt)
        self.agent.update(dt)

    def draw(self):
        self.obstacles.draw(G.screen)
        self.wp_mark.draw(G.screen)
        self.agent.draw(G.screen)

    def create_mark(self, pos):
        self.kill_mark()
        self.wp_mark.sprite = WaypointMark(pos)

    def kill_mark(self):
        if self.wp_mark.sprite:
            self.wp_mark.sprite.kill()

    def add_obstacles(self, obstacles):
        if self.obstacles is None:
            self.obstacles = pygame.sprite.Group()
        else:
            self.obstacles.empty()
        self.obstacles.add(*obstacles)

    def generate_obstacles(self, seed=None):
        """ Generate a random set of obstacles. """
        random.seed(seed)
        res = []
        n = random.randint(int(COLLIDABLE_NUM * 0.75), int(COLLIDABLE_NUM * 1.25))
        for _ in range(n):
            r = random.randint(COLLIDABLE_R_MIN, COLLIDABLE_R_MAX)
            agent = self.agent.sprite
            overlap = True
            while overlap:
                x, y = (random.randint(r, SCREEN_W - r), random.randint(r, SCREEN_H - r))
                overlap = False
                for e in res:
                    if distance((x, y), e.rect.center) < (r + e.radius):
                        overlap = True
                        break
                if distance((x, y), agent.rect.center) < (r + AGENT_RADIUS + 5):
                    overlap = True
            res.append(Obstacle((x, y), r))
        self.add_obstacles(res)


class Agent(pygame.sprite.Sprite):
    def __init__(self, pos):
        pygame.sprite.Sprite.__init__(self)
        self.pos = pos
        self.angle = 0.0
        self.destination = pos
        self.waypoint = None
        self.move_vector = None
        # agent picture
        r = AGENT_RADIUS
        self.surface = pygame.Surface((r*2, r*2), SRCALPHA).convert_alpha()
        pygame.draw.circle(self.surface, (40, 40, 150), [r, r], r)
        pygame.draw.line(self.surface, (150, 150, 150), [r-5, r], [r+5, r])
        pygame.draw.line(self.surface, (150, 150, 150), [r, r-5], [r, r+5])
        pygame.draw.polygon(self.surface, (150, 0, 0), [[r*2, r], [r*1.5, r-5], [r*1.5, r+5]])
        self.image = self.surface
        # bounding Rect and draw Rect
        self.rect = self.surface.get_rect()
        self.rect.center = pos
        self.drawrect = Rect(0, 0, r*4, r*4)
        self.drawrect.center = pos

    def update(self, dt):
        if self.waypoint:
            self.move_to_waypoint(dt)

    def rotate(self):
        center = self.rect.center
        self.image = pygame.transform.rotate(self.surface, math.degrees(self.angle))
        self.rect = self.image.get_rect(center=center)

    def move_to_waypoint(self, dt):
        """ Agent moves to waypoint, avoiding obstacles. """
        self.update_move_vector()
        dist = distance(self.pos, self.destination)
        if dist < 3.0:
            self.pos = (float(self.destination[0]), float(self.destination[1]))
            self.rect.center = self.destination
            self.waypoint = None
            self.move_vector = None
            G.world.kill_mark()
        else:
            self.rotate()
            dx = AGENT_SPEED * self.move_vector[0] * dt
            dy = AGENT_SPEED * self.move_vector[1] * dt
            self.pos = (self.pos[0] + dx, self.pos[1] + dy)
            self.rect.center = (round(self.pos[0]), round(self.pos[1]))
        self.drawrect.center = self.rect.center

    def update_move_vector(self):
        pos = self.pos
        ang = math.atan2(self.waypoint[1] - pos[1], self.waypoint[0] - pos[0])
        self.angle = -ang
        # set allowable directions range
        a_min = -math.pi/2  # minimum allowed movement vector angle
        a_max =  math.pi/2  # maximum allowed movement vector angle
        for e in G.world.obstacles:
            e_pos = e.rect.center
            d = distance(pos, e_pos) - (AGENT_RADIUS + e.radius)
            if d < COLLIDABLE_MIN_DIST:  # if in touch with obstacle
                # get angle to obstacle relative to angle to waypoint
                e_ang = ang_diff(ang, math.atan2(e_pos[1] - pos[1], e_pos[0] - pos[0]))
                # only allow parrallel or away from obstacle movement vector
                e_amin = e_ang - math.pi/2
                e_amax = e_ang + math.pi/2
                # constrain allowable direction range
                if a_min < e_amin < a_max:
                    a_max = e_amin
                elif a_min < e_amax < a_max:
                    a_min = e_amax
                # stuck if no allowable directions left
                if a_min > e_amin and a_max < e_amax:
                    self.move_vector = [0.0, 0.0]
                    return
        # pick movement direction as close as possible to waypoint
        if a_min > 0:
            ang += a_min
        if a_max < 0:
            ang += a_max
        self.move_vector = [math.cos(ang), math.sin(ang)]


class WaypointMark(pygame.sprite.Sprite):
    def __init__(self, pos):
        pygame.sprite.Sprite.__init__(self)
        self.surface = pygame.Surface((20, 20), SRCALPHA).convert_alpha()
        pygame.draw.line(self.surface, (55, 200, 0), [0, 0], [20, 20])
        pygame.draw.line(self.surface, (55, 200, 0), [0, 20], [20, 0])
        self.image = self.surface
        self.rect = self.surface.get_rect()
        self.rect.center = pos


class Obstacle(pygame.sprite.Sprite):
    def __init__(self, pos, r):
        pygame.sprite.Sprite.__init__(self)
        self.radius = r
        self.surface = pygame.Surface((r * 2, r * 2), SRCALPHA).convert_alpha()
        pygame.draw.circle(self.surface, (50, 60, 70), [r, r], r)
        self.image = self.surface
        self.rect = self.surface.get_rect()
        self.rect.center = pos


class Path:
    def __init__(self):
        self.path = []
        self.pathfinding = True
        self.graph_type = 'mesh'  # mesh, grid

    def update(self):
        agent = G.world.agent.sprite
        # agent reached destination
        if agent.pos[0] == agent.destination[0] and agent.pos[1] == agent.destination[1]:
            if self.path:
                self.path = []
            return
        # agent is following path
        if self.path:
            d = distance(agent.rect.center, agent.waypoint)
            if d > WP_TOLERANCE or len(self.path) == 1:
                return
            del(self.path[0])
            agent.waypoint = self.path[0]
        # compute path to destination
        else:
            if self.pathfinding and self.graph_type == 'mesh':
                # get start and goal nodes
                start = G.mesh.triangle(agent.rect.center)
                goal = G.mesh.triangle(agent.destination)
                # find path
                if set(start) == set(goal):  # same triangle
                    self.path = []
                elif start and goal:
                    self.path = astar_mesh(start, goal, agent.pos, agent.destination)
            elif self.pathfinding and self.graph_type == 'grid':
                NW = GRID_NODE_WIDTH
                # get start and goal nodes
                start = (agent.rect.center[0] // NW, agent.rect.center[1] // NW)
                goal = (agent.destination[0] // NW, agent.destination[1] // NW)
                # find path
                if goal not in G.grid.walls:
                    self.path = astar_grid(start, goal)
                # convert path nodes to world coordinates
                self.path = [(node[0]*NW + NW//2, node[1]*NW + NW//2) for node in self.path]
            # append destination to path
            self.path.append(agent.destination)
            # set agent waypoint
            agent.waypoint = self.path[0]

    def draw(self):
        if self.path:
            agent = G.world.agent.sprite
            pygame.draw.line(G.screen, PATH_COLOR, agent.rect.center, self.path[0])
            for i in range(len(self.path)-1):
                pygame.draw.line(G.screen, PATH_COLOR, self.path[i], self.path[i+1])


class Mesh:
    def __init__(self):
        self.edges = None

    def update(self):
        # get obstacle center points
        points = [o.rect.center for o in G.world.obstacles]

        # add border points
        border_points = []
        for o in G.world.obstacles:
            p = o.rect.center
            # add horizontal point
            if p[0] - o.radius <= BORDER_POINTS_THRESHOLD:
                border_points.append((0, p[1]))
            elif p[0] + o.radius >= SCREEN_W - BORDER_POINTS_THRESHOLD:
                border_points.append((SCREEN_W-1, p[1]))
            # add vertical point
            if p[1] - o.radius <= BORDER_POINTS_THRESHOLD:
                border_points.append((p[0], 0))
            elif p[1] + o.radius >= SCREEN_H - BORDER_POINTS_THRESHOLD:
                border_points.append((p[0], SCREEN_H-1))

        # add grid points along borders
        grid_points = []
        for i in range(SCREEN_W//8, SCREEN_W, SCREEN_W//8):
            merge1 = False
            merge2 = False
            for bp in border_points:
                if not merge1:
                    merge1 = bp[1] == 0 and abs(bp[0] - i) < BORDER_POINTS_MERGE_DIST
                if not merge2:
                    merge2 = bp[1] == SCREEN_H-1 and abs(bp[0] - i) < BORDER_POINTS_MERGE_DIST
            if not merge1:
                border_points.append((i, 0))
            if not merge2:
                border_points.append((i, SCREEN_H-1))
        for i in range(SCREEN_H//5, SCREEN_H, SCREEN_H//5):
            merge1 = False
            merge2 = False
            for bp in border_points:
                if not merge1:
                    merge1 = bp[0] == 0 and abs(bp[1] - i) < BORDER_POINTS_MERGE_DIST
                if not merge2:
                    merge2 = bp[0] == SCREEN_W-1 and abs(bp[1] - i) < BORDER_POINTS_MERGE_DIST
            if not merge1:
                border_points.append((0, i))
            if not merge2:
                border_points.append((SCREEN_W-1, i))
        points.extend(border_points)
        points.extend(grid_points)
        points.extend([(0, 0), (SCREEN_W-1, 0), (0, SCREEN_H-1), (SCREEN_W-1, SCREEN_H-1)])

        # triangulate
        self.edges = delaunay(points)

        # get edges metadata
        for e in self.edges:
            e.data = { 'org_r' : 0, 'dest_r' : 0, 'len' : 0, 'ang' : 0, \
                       'cross' : [-1, -1], 'wall' : True }
            # get radiuses
            org_r = 0
            dest_r = 0
            for o in G.world.obstacles:
                if o.rect.center[0] == e.org[0] and o.rect.center[1] == e.org[1]:
                    org_r = o.radius
                elif o.rect.center[0] == e.dest[0] and o.rect.center[1] == e.dest[1]:
                    dest_r = o.radius
                if org_r and dest_r:
                    break
            e.data['org_r'] = org_r
            e.data['dest_r'] = dest_r
            # mark border edges
            e.data['wall'] = org_r == 0 and dest_r == 0 and self.is_border_edge(e)
            # mark walls
            if not e.data['wall']:
                d = distance(e.org, e.dest)
                e.data['len'] = d
                if  d > (org_r + dest_r + (AGENT_RADIUS + COLLIDABLE_MIN_DIST) * 2):
                    a = math.atan2(e.dest[1] - e.org[1], e.dest[0] - e.org[0])
                    e.data['ang'] = a
                else:
                    e.data['wall'] = True
            # copy metadata to symmetrical counterpart
            e.sym.data = e.data.copy()
            e.sym.data['org_r'], e.sym.data['dest_r'] = e.sym.data['dest_r'], e.sym.data['org_r']
            if e.sym.data['ang'] < 0:
                e.sym.data['ang'] += math.pi
            else:
                e.sym.data['ang'] -= math.pi


    def draw(self):
        for e in self.edges:
            if e.data['wall']:
                pygame.draw.line(G.screen, WALL_COLOR, e.org, e.dest)
            else:
                pygame.draw.line(G.screen, EDGE_COLOR, e.org, e.dest)


    def is_border_edge(self, e):
        """ Check if e is a border edge. """
        return (e.org[0] == 0 and e.dest[0] == 0) \
            or (e.org[0] == SCREEN_W-1 and e.dest[0] == SCREEN_W-1) \
            or (e.org[1] == 0 and e.dest[1] == 0) \
            or (e.org[1] == SCREEN_H-1 and e.dest[1] == SCREEN_H-1)


    def triangle(self, p):
        """ Return triangle (list of edges) containing p. """
        for e in self.edges:
            if left_of(p, e) and left_of(p, e.onext.sym) and left_of(p, e.sym.oprev):
                return [a for a in [e, e.sym, e.onext, e.onext.sym, e.sym.oprev, e.sym.oprev.sym]
                    if not a.data['wall']]
        return []


class Grid:
    def __init__(self):
        # nodes
        self.node_width = GRID_NODE_WIDTH
        scr_rect = G.screen.get_rect()
        x = int(scr_rect.width // self.node_width)
        y = int(scr_rect.height // self.node_width)
        self.nodes = np.zeros((x, y), dtype = np.int)
        # walls
        self.walls = set()
        self.outer_walls = set((a, b) for a in range(-1, x+1) for b in range(-1, y+1)) - \
                           set((a, b) for a in range(0, x) for b in range(0, y))
        # visuals
        self.drawables = [] # list of visual elements
        self.cross_size = 10
        sz = self.cross_size
        self.cross = pygame.Surface((sz * 2, sz * 2), SRCALPHA).convert_alpha()
        pygame.draw.line(self.cross, EDGE_COLOR, [0, sz], [sz * 2, sz])
        pygame.draw.line(self.cross, EDGE_COLOR, [sz, 0], [sz, sz * 2])
        self.font = pygame.font.SysFont('Ubuntu', 12)

    def update(self):

        def neighbors_star(grid, node):
            """ List of 4 node neighbors. """
            res = set((node[0] + x, node[1] + y) for x, y in [(0, -1), (-1, 0), (1, 0), (0, 1)])
            res -= grid.walls | grid.outer_walls
            return res

        self.nodes.fill(0)
        self.walls.clear()
        # update costs
        for e in G.world.obstacles:
            node_x0 = e.rect.left // self.node_width
            node_y0 = e.rect.top // self.node_width
            node_x1 = e.rect.right // self.node_width
            node_y1 = e.rect.bottom // self.node_width
            # map walls
            for x in range(node_x0, node_x1 + 1):
                for y in range(node_y0, node_y1 + 1):
                    node = (x, y)
                    # index outside of the grid or already marked
                    if node in (self.walls | self.outer_walls):
                        continue
                    node_center = (int((x + 0.5) * self.node_width), \
                                   int((y + 0.5) * self.node_width))
                    # node center is covered = wall
                    if e.radius < self.node_width//2:
                        d = distance(node_center, e.rect.center) - int(self.node_width * 0.3)
                    else:
                        d = distance(node_center, e.rect.center) - int(self.node_width * 0.1)
                    if d < e.radius:
                        self.walls.add(node)
                        self.nodes[node] = 99
        
        # expand walls
        new_walls = set()
        for node in self.walls:
            new_walls |= neighbors_star(self, node)
        for node in new_walls:
            self.nodes[node] = 99
        self.walls |= new_walls

        # redraw
        dim = self.nodes.shape
        self.drawables = []
        cross, cross_size = self.cross, self.cross_size
        for dim_x in range(dim[0]):
            for dim_y in range(dim[1]):
                x = dim_x * self.node_width
                y = dim_y * self.node_width
                self.drawables.append((cross, x - cross_size, y - cross_size))
                if self.nodes[dim_x, dim_y]:
                    text = self.font.render('#', 0, WALL_COLOR)
                    x = x + self.node_width * 0.5 - text.get_rect().center[0]
                    y = y + self.node_width * 0.5 - text.get_rect().center[1]
                    self.drawables.append((text, x, y))
            else:
                x = dim_x * self.node_width
                y = dim[1] * self.node_width
                self.drawables.append((cross, x - cross_size, y - cross_size))
        else:
            for dim_y in range(dim[1]):
                x = dim[0] * self.node_width
                y = dim_y * self.node_width
                self.drawables.append((cross, x - cross_size, y - cross_size))
            else:
                x = dim[0] * self.node_width
                y = dim[1] * self.node_width
                self.drawables.append((cross, x - cross_size, y - cross_size))

    def draw(self):
        for obj in self.drawables:
            G.screen.blit(obj[0], obj[1:])


class FPS:
    def __init__(self):
        self.font = pygame.font.SysFont('Ubuntu', 16)
        self.color = Color(255, 255, 255, 100)
        self.surface = self.font.render('9999', 0, self.color)
        self.rect = Rect(0, 0, 40, 20)
        self.rect.x = G.screen.get_rect().right - 50
        self.rect.y = 5
        self.fps_buf = np.zeros(FPS_BUF_SIZE, dtype = np.int)
        self.fps_idx = 0
    
    def update(self, n):
        self.fps_buf[self.fps_idx] = n
        fps = str(int(np.mean(self.fps_buf)))
        self.surface = self.font.render(fps, 0, self.color)
        self.fps_idx += 1
        if self.fps_idx == FPS_BUF_SIZE:
            self.fps_idx = 0

    def draw(self):
        G.screen.blit(self.surface, self.rect)
