from time import time
import numpy as np
from numpy.ma.core import right_shift
import pygame

WIDTH = 1000
HEIGHT = 1000
CELLWIDTH = 20
RENDER_SKIP = 30
ANIMATION_LENGTH = 20

COLOR_BLACK = (0, 0, 0)
COLOR_WHITE = (255, 255, 255)
COLOR_GREY40 = (40, 40, 40)
COLOR_RED = (255, 0, 0)
COLOR_GREEN = (0, 255, 0)
COLOR_BLUE = (0, 0, 255)
COLOR_PINK = (255, 0, 255)
COLOR_PURPLE = (155, 50, 168)
COLOR_ORANGE = (255, 151, 5)

def circle(x, y, r):
    xx, yy = np.meshgrid(np.arange(x), np.arange(y))
    array = (xx-x/2+0.5)**2 + (yy-y/2+0.5)**2
    array4 = np.zeros((x, y, 4), dtype=np.uint8)
    array4[:,:,3] = ((array > r**2) * 255).astype(np.int)
    return array4

# generate 10 sprites
sprite_sizes = np.linspace(3,20,11)
sprite_list = []
for size in sprite_sizes:
    sprite_list.append(circle(20, 20, size))

def get_color(color0, color1, frac):
    color_return = [0] * 3
    xp = [0, 1]
    for i in range(3):
        fp = [color0[i], color1[i]]
        color_return[i] = np.interp(frac, xp, fp).astype(np.uint8)
    return color_return


def line_intersect(x1, y1, x2, y2, x3, y3, x4, y4):
    # returns a (x, y) tuple or None if there is no intersection
    d = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    if d:
        s = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / d
        t = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / d
    else:
        return None
    if not(0 <= s <= 1 and 0 <= t <= 1):
        return None
    x = x1 + s * (x2 - x1)
    y = y1 + s * (y2 - y1)
    return x, y


def timeit(method):
    def timed(*args, **kw):
        import time
        ts = time.time()
        result = method(*args, **kw)
        te = time.time()
        print(method.__name__, (te-ts)*1000, ' ms')
        return result
    return timed


class Renderer():
    # the renderer only handles the rendering of the path finding
    # the path finding algorithm can function without the renderer
    def __init__(self, MazeSolver, start=None, end=None):
        # todo: generate sprites
        self.run = True
        self.MazeSolver = MazeSolver
        self.start = start
        self.end = end
        self.interaction_mode = 'none'
        pygame.init()
        pygame.display.set_caption('Python Maze Generator')
        self.surface = pygame.display.set_mode((WIDTH, HEIGHT))
        self.clock = pygame.time.Clock()
        self.surface.fill(COLOR_WHITE)
        # each node is an object, stored in an numpy array
        self.node_array = np.empty(self.MazeSolver.maze.shape, dtype=object)
        for index, value in np.ndenumerate(self.MazeSolver.maze):
            y, x = index
            node = RendererNode(self.MazeSolver, self.surface, x, y)
            node.reset()
            self.node_array[y, x] = node
        self.draw_nodes_all()


    def reset_nodes(self, animation=True):
        for node in self.node_array.flat:
            node.reset(animation=animation)

        
    def get_node(self, x, y):
        return self.node_array[y,x]

    
    def draw_point(self, x=None, y=None, X=None, Y=None):
        if (x is not None) and (y is not None):
            center = ((0.5+x)*CELLWIDTH, (0.5+y)*CELLWIDTH)
        if (X is not None) and (Y is not None):
            center = (X, Y)
        radius = 3
        pygame.draw.circle(self.surface, COLOR_RED, center, radius)
    

    def draw_sprite(self, x, y):
        test = np.zeros((CELLWIDTH, CELLWIDTH))
        surf = pygame.surfarray.make_surface(test)
        self.surface.blit(surf, (x, y))
        

    def draw_nodes_all(self):
        # for node in self.nodes_all:
            # node._draw()
        for node in self.node_array.flat:
            node.draw()
        pygame.display.update()


    def draw_nodes_updated(self):
        new_rects = []
        for node in self.node_array.flat:
            if node.render:
                new_rects.append(node.draw())
            if node.animation_state > 1:
                node.render = False
        pygame.display.update(new_rects)


    def render_frame(self):
        def interaction_end():
            self.interaction_mode = 'none'
            self.X1, self.Y1 = None, None
            self.MazeSolver.reset()
            self.MazeSolver.find_path()
            print('reset')

        self.draw_nodes_updated()
        self.clock.tick(30)
        pygame.display.update()
        MazeSolver.renderer.draw_point(X=10, Y=20)
        MazeSolver.renderer.draw_sprite(20,20)
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                self.interaction_mode = 'block' if event.button == 1 else 'free'
                self.X1, self.Y1 = event.pos
            
            if event.type == pygame.MOUSEBUTTONUP:
                interaction_end()

            if event.type == pygame.MOUSEMOTION:
                X2, Y2 = event.pos
                # interpolate mouse movement
                if self.interaction_mode in ['block', 'free']:
                    self.interpolate_mouse_movement(
                        self.X1, self.Y1, X2, Y2)
                self.X1, self.Y1 = X2, Y2

            # check for closing the window
            if event.type == pygame.QUIT:
                self.run = False
                

    def interpolate_mouse_movement(self, X1, Y1, X2, Y2):
        # X1, Y1, X2, Y2: mouse movement in pixel coordinates
        # x1, y1: current cell in cell coordinates
        x_cell, y_cell = X1//CELLWIDTH, Y1//CELLWIDTH
        self.check_if_mouse_leaves_cell(X1, Y1, X2, Y2, x_cell, y_cell) 


    def check_if_mouse_leaves_cell(self, X1, Y1, X2, Y2, x_cell, y_cell):
        # X1, Y1, X2, Y2: mouse movement in pixel coordinates
        # x_cell, y_celL: current cell in cell coordinates
        # XA, YA, XB, YB: cell edges in pixel coordinates
        XA, XB = x_cell * CELLWIDTH, (x_cell+1) * CELLWIDTH
        YA, YB = y_cell * CELLWIDTH, (y_cell+1) * CELLWIDTH
        # get direction of mouse vector
        DX, DY = X2-X1, Y2-Y1
        # check if mouse vector leaves left/right
        if DX >= 0: 
            side = [XB, YA, XB, YB]
            x_cell_new = x_cell + 1
        else: 
            side = [XA, YA, XA, YB]
            x_cell_new = x_cell - 1
        result = line_intersect(X1, Y1, X2, Y2, *side)
        if result:
            if self.MazeSolver.check_cell_coords_in_bounds(x_cell_new, y_cell):
                self.apply_env_modification(x_cell_new, y_cell, self.interaction_mode)
                self.check_if_mouse_leaves_cell(X1, Y1, X2, Y2, x_cell_new, y_cell)
            return
        else:
            # check if mouse vector leaves top/bottom
            if DY >= 0:
                side = [XA, YB, XB, YB]
                y_cell_new = y_cell + 1
            else: 
                side = [XA, YA, XB, YA]
                y_cell_new = y_cell - 1
            result = line_intersect(X1, Y1, X2, Y2, *side)
            if result:
                if self.MazeSolver.check_cell_coords_in_bounds(x_cell, y_cell_new):
                    self.apply_env_modification(x_cell, y_cell_new, self.interaction_mode)
                    self.check_if_mouse_leaves_cell(X1, Y1, X2, Y2, x_cell, y_cell_new)
        return


    def apply_env_modification(self, x, y, mode):
        # x: position in cell coordinates
        # X: position in pixel coordinates
        # x, y = int(np.floor(X/CELLWIDTH)), int(np.floor(Y/CELLWIDTH))
        if mode == 'free': state_int = 0
        if mode == 'block': state_int = 1 
        self.MazeSolver.set_maze_state(x, y, state_int)


    def set_state_str(self, x, y, state_str: str, animation=True):
        color0=COLOR_BLACK
        node = self.get_node(x, y)
        if node.state_str not in ['start', 'end']:
            node._set_state(state_str, animation=animation, color0=color0)
            
    def set_state_int(self, x, y, state_int: int, animation=True):
        node = self.get_node(x, y)
        state_str = node.state_int_to_state_str(state_int)
        self.set_state_str(x, y, state_str, animation=animation)


class RendererNode():
    def __init__(self, MazeSolver, surface, x: int, y: int, state_str=None):
        self.MazeSolver = MazeSolver
        self.surface = surface
        self.x = x
        self.y = y
        self.state_str = state_str
        self.animation_state = 0
        self.color0 = COLOR_WHITE
        self.color1 = COLOR_WHITE
        self.active = False
        self.render = True
        
    def state_int_to_state_str(self, state_int):
        # 0: free, 1: blocked, 2: start, 3: end
        state_list = ['free', 'blocked', 'start', 'end']
        return state_list[state_int]
        
    def reset(self, animation=True):
        self.color0 = self.color1
        if self.active == True:
            # do not reset this time
            self.active = False
        else:
            # get initial state
            state_int = self.MazeSolver.maze[self.y, self.x]
            self._set_state(self.state_int_to_state_str(state_int), animation=animation)
            

    def _set_state(self, state_str: str, animation=True, color0=None):
        self.active = True
        self.render = True
        self.state_str = state_str
        if animation:
            self.animation_state = 0
        else:
            self.animation_state = 0.99
        if color0:
            self.color0 = color0

    def draw(self, color=None):
        self.animation_state += 1/ANIMATION_LENGTH
        color_dict = dict(free = COLOR_WHITE,
                          blocked = COLOR_BLACK,
                          start = COLOR_GREEN,
                          end = COLOR_BLUE,
                          open = COLOR_PURPLE,
                          closed = COLOR_ORANGE,
                          best_path = COLOR_GREEN,)

        self.color1 = color_dict[self.state_str]
        if not color:
            color = get_color(self.color0, self.color1, self.animation_state)
        
        
        sprite = sprite_list[int(self.animation_state*10)]
        surf = pygame.image.frombuffer(sprite.tobytes(), (20, 20), 'RGBA')
        # surf = surf.convert_alpha()
        
        # surf = pygame.surfarray.make_surface(sprite)
        # surf = surf.convert_alpha()

        # surf.fill(color)

        # surf.pixels_alpha = 
        # surf.fill(color)
        
        # return self.surface.blit(surf, (self.x*CELLWIDTH, self.y*CELLWIDTH))
        return pygame.draw.rect(self.surface, color, (self.x*CELLWIDTH, self.y*CELLWIDTH, CELLWIDTH, CELLWIDTH))


class MazeSolver():
    def __init__(self):
        self.node_counter = 0
        self.current_node = None

        # maze state_list
        # 0: free, 1: blocked, 2: start, 3: end
        self.maze = np.zeros((HEIGHT//CELLWIDTH, WIDTH//CELLWIDTH), dtype=np.uint8)  # 50*50
        # self.maze[10:40, 20] = 0
        self.start = (0, 0)  # (x, y)
        self.end = (49, 49)  # (x, y)
        self.renderer = Renderer(self, start=self.start, end=self.end)
        self.reset()
        self.x_bounds = (0, WIDTH//CELLWIDTH - 1)
        self.y_bounds = (0, HEIGHT//CELLWIDTH - 1)

    def reset(self):
        self.open = set()
        self.closed = set()
        self.add_node_to_open(self.create_node(*self.start))
        self.shortest_path=[]
        
    def set_maze_state(self, x, y, state_int):
        # state: 0=free, 1=blocked 
        self.maze[y, x] = state_int
        self.renderer.set_state_int(x, y, state_int)

    def add_node_to_open(self, node):
        self.open.add(node)
        # set animation state
        self.renderer.set_state_str(node.x, node.y, 'open')

    def add_node_to_closed(self, node):
        self.closed.add(node)
        # set animation state
        self.renderer.set_state_str(node.x, node.y, 'closed')

    def create_node(self, x, y):
        # get index
        idx = self.node_counter
        self.node_counter += 1
        g_cost = self.calculate_g_cost(x, y)
        h_cost = self.calculate_h_cost(x, y)
        return Node(x, y, idx=idx, g_cost=g_cost, h_cost=h_cost)

    def calculate_g_cost(self, x, y):
        # g_cost: current path length
        if not self.current_node:
            current_cost = 0
            dx, dy = abs(x-self.start[0]), abs(x-self.start[1])
        else:
            current_cost = self.current_node.g_cost
            dx, dy = abs(x-self.current_node.x), abs(y-self.current_node.y)
        n_nondiag = abs(dx-dy)
        n_diag = max(dx, dy) - n_nondiag
        # 14 for diagonal traversing, 10 for hor. / vert.
        return current_cost + 10*n_nondiag + 14*n_diag


    def calculate_h_cost(self, x, y):
        # h_cost: heuristic cost, distance from node to end node
        if not self.current_node:
            dx, dy = abs(x-self.end[0]), abs(x-self.end[1])
        else:
            dx, dy = abs(x-self.end[0]), abs(y-self.end[1])
        n_nondiag = abs(dx-dy)
        n_diag = max(dx, dy) - n_nondiag
        # 14 for diagonal traversing, 10 for hor. / vert.
        return 14*n_diag + 10*n_nondiag


    def get_node_fcost_min(self):
        # finds node with min(node.fcost) from self.open 
        # removes the node from self.open
        # returns the node
        lowest_object = None
        lowest_value = 1e9
        for item in self.open:
            if item.f_cost < lowest_value:
                lowest_object = item
                lowest_value = item.f_cost
            elif item.f_cost == lowest_value:
                if item.idx < lowest_object.idx:
                    lowest_object = item
                    lowest_value = item.f_cost
        self.open.remove(lowest_object)
        return lowest_object
    
    
    def check_cell_coords_in_bounds(self, x, y):
        return np.all([self.x_bounds[0] <= x <= self.x_bounds[1],
                       self.y_bounds[0] <= y <= self.y_bounds[1]])


    def get_node_neighbour_coords(self, x, y): # 0 ms
        # returns coords tuples (x, y) of traversable neighbours as list
        #   --->
        # |    x
        # v y
        coords_candidates_list = []
        for x_shift in range(-1, 2):
            for y_shift in range(-1, 2):
                if x_shift == 0 and y_shift == 0: continue
                coords_candidates_list.append((x+x_shift, y+y_shift))

        # check if coordinates are in boundaries
        check_boundaries = [False] * len(coords_candidates_list)
        for i, coords in enumerate(coords_candidates_list):
            # x_bounds = (0, 49)  # todo : dynamic
            # y_bounds = (0, 49)
            if self.check_cell_coords_in_bounds(coords[0], coords[1]):
                check_boundaries[i] = True
        zipped_list = zip(coords_candidates_list, check_boundaries)
        coords_candidates_list = [value for value, check in zipped_list if check]

        # check if coordinates are traversable
        check_traversable = [False] * len(coords_candidates_list)
        for i, coords in enumerate(coords_candidates_list):
            if self.maze[coords[1], coords[0]] == 0:
                check_traversable[i] = True
        zipped_list = zip(coords_candidates_list, check_traversable)
        coords_candidates_list = [value for value, check in zipped_list if check]

        return coords_candidates_list


    def find_node_in_closed(self, x, y):
        for node in self.closed:
            if node.x == x:
                if node.y == y:
                    return node
        return None


    def find_node_in_open(self, x, y):
        for node in self.open:
            if node.x == x:
                if node.y == y:
                    return node
        return None
    
    def export_shortest_path(self):
        run = True
        current_node = self.current_node
        x_coords = []
        y_coords = []
        while run:
            current_node = current_node.parent
            x_coords.append(current_node.x)
            y_coords.append(current_node.y)
            if current_node.x == self.start[0]:
                if current_node.y == self.start[1]:
                    run = False
        return_list = list(zip(x_coords, y_coords))
        return_list.reverse()
        self.shortest_path = return_list

            
    def apply_shortest_path(self):
        if len(self.shortest_path) > 0:
            segment = self.shortest_path.pop()
            x, y = segment
            self.renderer.set_state_str(x, y, 'best_path')
            return False
        else: # when path is completely applied
            self.renderer.reset_nodes()
            return True
        

    # @timeit
    def astar_step(self):  # 0ms
        # solving path finding
        self.current_node = self.get_node_fcost_min()
        self.add_node_to_closed(self.current_node)

        if (self.current_node.x, self.current_node.y) == self.end:
            print('path has been found')
            self.export_shortest_path()
            return True

        # get neighbours of current_node
        neighbour_coords = self.get_node_neighbour_coords(self.current_node.x, self.current_node.y)

        for coords in neighbour_coords:  # for each neighbour
            # STEP 1
            # check if node with x,y is in closed -> if yes, skip
            if self.find_node_in_closed(*coords):
                continue

            # calculate new g_cost for neighbour node
            new_g_cost = self.calculate_g_cost(*coords)

            # check if node with x,y is in open -> if yes, use
            node_in_open = self.find_node_in_open(*coords)

            if node_in_open:  # node is in open
                if new_g_cost < node_in_open.g_cost:
                    # execute below
                    node_in_open._set_g_cost(new_g_cost)
                    neighbour_node = node_in_open
                else:
                    continue  # continue to next node
            else:  # node is not in open
                neighbour_node = self.create_node(*coords)

            # update g_cost of neighbour
            neighbour_node._set_g_cost(new_g_cost)
            # set parent of neighbour to current
            neighbour_node._set_parent(self.current_node)

            if not node_in_open:  # add node to open
                self.add_node_to_open(neighbour_node)
        return False
    
    def find_path(self):
        self.frame_counter = 0
        path_found = False
        path_applied = False
        while self.renderer.run:
            self.frame_counter += 1
            self.renderer.render_frame()
            for _ in range(RENDER_SKIP):
                if path_found == False:
                    path_found = self.astar_step()
            for _ in range(2):
                if path_applied == False and path_found:
                    path_applied = self.apply_shortest_path()


class Node():
    def __init__(self, x, y, idx=None, parent=None, g_cost=None, h_cost=None):
        self.x = x
        self.y = y
        self.idx = idx
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = None
        if (g_cost is not None) and (h_cost is not None):
            self._calculate_f_cost()
        self.parent = parent
        # g_cost: distance from starting node (not perfect)
        # h_cost: distance from end ndoe (perfect)
        # f_cost: g_cost + h_cost

    def _set_idx(self, idx):
        self.idx = idx

    def _set_parent(self, parent):
        self.parent = parent

    def _set_g_cost(self, g_cost):
        self.g_cost = g_cost
        if self.h_cost:
            self._calculate_f_cost()

    def _set_h_cost(self, h_cost):
        self.h_cost = h_cost
        if self.g_cost:
            self._calculate_f_cost()

    def _calculate_f_cost(self):
        self.f_cost = self.g_cost + self.h_cost


MazeSolver = MazeSolver()
MazeSolver.find_path()


# todo: press r for reset