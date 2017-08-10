import pygame
from   pygame.locals import *
import time
from   entities import *
from   config import *
import config as G


def load():
    """ Load and initialize scene. """
    pygame.init()
    pygame.display.set_caption('Go Robot')
    G.screen = pygame.display.set_mode([SCREEN_W, SCREEN_H], SCREEN_MODE)
    G.FPS = FPS()
    G.world = World()
    G.world.generate_obstacles()
    G.mesh = Mesh()
    G.mesh.update()
    G.grid = Grid()
    G.grid.update()
    G.path = Path()
    G.show_mesh = True
    G.show_grid = False
    G.show_path = True
    G.redraw_scene = True


def main():
    """ Main loop. """
    frame_min_t = 1 / MAX_FPS
    t = 0.0
    t_last = 0.0
    time.clock()  # init clock
    while events(pygame.event.get()):
        t_last = t
        t = time.clock()
        update(t - t_last)
        draw()


def events(events_queue):
    """ Events pass. """
    for event in events_queue:
        # close
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            return False
        # set destination
        elif event.type == MOUSEBUTTONDOWN:
            G.world.create_mark(event.__dict__['pos'])
            G.world.agent.sprite.destination = event.__dict__['pos']
            G.path.path = []
            G.redraw_scene = True
        # generate new obstacles
        elif event.type == KEYDOWN and event.__dict__['unicode'] == ' ':
            G.world.generate_obstacles()
            G.grid.update()
            G.mesh.update()
            G.redraw_scene = True
        # draw grid on/off
        elif event.type == KEYDOWN and event.__dict__['unicode'] == 'g':
            G.show_grid = not G.show_grid
            G.redraw_scene = True
        # draw mesh on/off
        elif event.type == KEYDOWN and event.__dict__['unicode'] == 'm':
            G.show_mesh = not G.show_mesh
            G.redraw_scene = True
        # draw path on/off
        elif event.type == KEYDOWN and event.__dict__['unicode'] == 'p':
            G.show_path = not G.show_path
            G.redraw_scene = True
        # compute path on/off
        elif event.type == KEYDOWN and event.__dict__['unicode'] == 'o':
            G.path.pathfinding = not G.path.pathfinding
            G.redraw_scene = True
        # switch graph type between 'mesh' and 'grid'
        elif event.type == KEYDOWN and event.__dict__['unicode'] == 't':
            if G.path.graph_type == 'mesh':
                G.path.graph_type = 'grid'
                G.show_mesh = False
                G.show_grid = True
            else:
                G.path.graph_type = 'mesh'
                G.show_mesh = True
                G.show_grid = False
            G.redraw_scene = True
    return True


def update(dt):
    """ Update pass. """
    G.world.update(dt)
    G.FPS.update(round(1 / dt))
    G.path.update()


def draw():
    """ Draw pass. """
    if G.redraw_scene:
        G.screen.fill(BG_COLOR)
    else:
        G.screen.fill(BG_COLOR, G.world.agent.sprite.drawrect)
        G.screen.fill(BG_COLOR, G.FPS.rect)
    G.world.draw()
    if G.show_mesh:
        G.mesh.draw()
    if G.show_grid:
        G.grid.draw()
    if G.show_path:
        G.path.draw()
    G.FPS.draw()
    if G.redraw_scene:
        pygame.display.flip()
        G.redraw_scene = False
    else:
        pygame.display.update([G.world.agent.sprite.drawrect, G.FPS.rect])


if __name__ == '__main__':
    load()
    main()
    pygame.quit()
