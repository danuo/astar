from enum import Enum
from typing import TYPE_CHECKING, Optional, Tuple

if TYPE_CHECKING:
    from code.astar_implementation import MazeSolver

import numpy as np
import pygame


class ArrayDict:
    def __init__(self):
        self.data = dict()

    def __getitem__(self, x) -> "RendererNode":
        return self.data[x]

    def __setitem__(self, x, val: "RendererNode"):
        self.data[x] = val

    def __iter__(self):
        return iter(self.data.values())


WIDTH = 1000
HEIGHT = 1000
CELLWIDTH = 20
RENDER_SKIP = 30
ANIMATION_LENGTH = 20


COLOR_BLACK = (0, 0, 0)
COLOR_GREY40 = (40, 40, 40)
COLOR_WHITE = (255, 255, 255)
COLOR_LBLUE = (60, 63, 201)
COLOR_DBLUE = (18, 0, 120)
COLOR_PURP0 = (199, 44, 186)
COLOR_PURP1 = (157, 1, 145)
COLOR_PINK = (253, 58, 105)


class NodeStates(Enum):
    FREE = 0
    BLOCKED = 1
    START = 2
    END = 3
    OPEN = 4
    CLOSED = 5
    BESTPATH = 6


starting_colors = {
    NodeStates.FREE: COLOR_LBLUE,
    NodeStates.BLOCKED: COLOR_BLACK,
    NodeStates.START: COLOR_WHITE,
    NodeStates.END: COLOR_WHITE,
    NodeStates.OPEN: COLOR_WHITE,
    NodeStates.CLOSED: COLOR_PURP0,
    NodeStates.BESTPATH: COLOR_WHITE,
}


ending_colors = {
    NodeStates.FREE: COLOR_WHITE,
    NodeStates.BLOCKED: COLOR_GREY40,
    NodeStates.START: COLOR_PINK,
    NodeStates.END: COLOR_PINK,
    NodeStates.OPEN: COLOR_DBLUE,
    NodeStates.CLOSED: COLOR_PURP1,
    NodeStates.BESTPATH: COLOR_PINK,
}


def get_color(state: NodeStates):
    color0 = COLOR_WHITE
    color1 = COLOR_BLACK
    if state in starting_colors:
        color0 = starting_colors[state]
    if state in ending_colors:
        color1 = ending_colors[state]
    return color0, color1


def interpolate_color(color0, color1, frac: float) -> pygame.Color:
    color_return = [0] * 3
    xp = [0, 1]
    for i in range(3):
        fp = [color0[i], color1[i]]
        color_return[i] = int(np.interp(frac, xp, fp))
    return pygame.Color(*color_return)


def line_intersect(x1, y1, x2, y2, x3, y3, x4, y4):
    # returns (x, y) tuple if there is an intersection
    # returns None in case of no intersection
    d = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    if d:
        s = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / d
        t = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / d
    else:
        return None
    if not (0 <= s <= 1 and 0 <= t <= 1):
        return None
    x = x1 + s * (x2 - x1)
    y = y1 + s * (y2 - y1)
    return x, y


def state_int_to_state_str(state_int):
    # 0: free, 1: blocked, 2: start, 3: end
    state_list = ["free", "blocked", "start", "end"]
    return state_list[state_int]


class Renderer:
    # the renderer only handles the rendering of the path finding
    # the path finding algorithm can function without the renderer
    def __init__(self, MazeSolver: "MazeSolver", start=None, end=None, export_frames=None):
        self.run = True
        self.maze_solver = MazeSolver
        self.start = start
        self.end = end
        self.interaction_mode = "none"
        pygame.init()
        pygame.display.set_caption("Python Maze Generator")
        self.surface = pygame.display.set_mode((WIDTH, HEIGHT))
        self.clock = pygame.time.Clock()
        self.surface.fill(COLOR_WHITE)
        self.frame_counter = 0
        self.export_frames = export_frames

        # each node is stored in an 2-d numpy array for simple referencing
        # self.node_array: NDArray[RendererNode] = np.empty(self.maze_solver.maze.shape, dtype=object)
        self.node_array = ArrayDict()
        for index, value in np.ndenumerate(self.maze_solver.maze):
            y, x = index
            node = RendererNode(self.maze_solver, self.surface, x, y)
            node.reset()
            self.node_array[y, x] = node
        self.draw_nodes_all()

    def reset_nodes(self, animation=True):
        for node in self.node_array:
            node.reset(animation=animation)

    def get_node(self, x, y) -> "RendererNode":
        return self.node_array[y, x]

    def draw_nodes_all(self):
        # draw every node
        for node in self.node_array:
            node.draw()
        pygame.display.update()

    def draw_nodes_updated(self):
        # draw selection of nodes (faster)
        # only nodes that need to be updated are drawn
        new_rectangles = []
        node: RendererNode
        for node in self.node_array:
            if node.render:
                new_rectangles.append(node.draw())
            if node.animation_progress > 1:
                node.render = False
        pygame.display.update(new_rectangles)

    def render_frame(self):
        def interaction_end():
            self.interaction_mode = "none"
            self.X1, self.Y1 = None, None
            self.maze_solver.reset()
            self.maze_solver.find_path()
            print("reset")

        self.draw_nodes_updated()
        self.clock.tick(30)

        # export frame as jpg
        if self.export_frames:
            from PIL import Image

            self.surface = pygame.display.get_surface()
            image3d = np.ndarray((WIDTH, HEIGHT, 3), np.uint8)
            pygame.pixelcopy.surface_to_array(image3d, self.surface)
            image3dT = np.transpose(image3d, axes=[1, 0, 2])
            im = Image.fromarray(image3dT)  # monochromatic image
            imrgb = im.convert("RGB")  # color image
            filename = str(self.frame_counter).zfill(5) + ".jpg"
            imrgb.save(filename)
            self.frame_counter += 1

        # process user input
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                self.interaction_mode = "block" if event.button == 1 else "free"
                self.X1, self.Y1 = event.pos

            if event.type == pygame.MOUSEBUTTONUP:
                interaction_end()

            if event.type == pygame.MOUSEMOTION:
                X2, Y2 = event.pos
                # interpolate mouse movement to effect every node on mouse
                # movement trajectory
                if self.interaction_mode in ["block", "free"]:
                    self.interpolate_mouse_movement(self.X1, self.Y1, X2, Y2)
                self.X1, self.Y1 = X2, Y2

            # check for closing the window
            if event.type == pygame.QUIT:
                self.run = False

    def interpolate_mouse_movement(self, X1, Y1, X2, Y2):
        # X1, Y1, X2, Y2: mouse movement in pixel coordinates
        # x1, y1: current cell in cell coordinates
        x_cell, y_cell = X1 // CELLWIDTH, Y1 // CELLWIDTH
        self.check_if_mouse_leaves_cell(X1, Y1, X2, Y2, x_cell, y_cell)

    def check_if_mouse_leaves_cell(self, X1, Y1, X2, Y2, x_cell, y_cell):
        # X1, Y1, X2, Y2: mouse movement in pixel coordinates
        # x_cell, y_celL: current cell in cell coordinates
        # XA, YA, XB, YB: cell edges in pixel coordinates
        XA, XB = x_cell * CELLWIDTH, (x_cell + 1) * CELLWIDTH
        YA, YB = y_cell * CELLWIDTH, (y_cell + 1) * CELLWIDTH
        # get direction of mouse vector
        DX, DY = X2 - X1, Y2 - Y1
        # check if mouse vector leaves left/right
        if DX >= 0:
            side = [XB, YA, XB, YB]
            x_cell_new = x_cell + 1
        else:
            side = [XA, YA, XA, YB]
            x_cell_new = x_cell - 1
        result = line_intersect(X1, Y1, X2, Y2, *side)
        if result:
            if self.maze_solver.check_cell_coords_in_bounds(x_cell_new, y_cell):
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
                if self.maze_solver.check_cell_coords_in_bounds(x_cell, y_cell_new):
                    self.apply_env_modification(x_cell, y_cell_new, self.interaction_mode)
                    self.check_if_mouse_leaves_cell(X1, Y1, X2, Y2, x_cell, y_cell_new)
        return

    def apply_env_modification(self, x, y, mode):
        # x: position in cell coordinates
        # X: position in pixel coordinates
        # x, y = int(np.floor(X/CELLWIDTH)), int(np.floor(Y/CELLWIDTH))
        if mode == "free":
            state = NodeStates.FREE
        else:  # mode == "block"
            state = NodeStates.BLOCKED
        self.maze_solver.set_maze_state(x, y, state.value)
        self.set_renderer_state(x, y, state)

    def set_renderer_state(self, x, y, state: NodeStates, animation=True):
        color0, color1 = get_color(state)
        node = self.get_node(x, y)
        if node.state not in (NodeStates.START, NodeStates.END):
            node.set_state(state, with_animation=animation, color0=color0, color1=color1)


class RendererNode:
    def __init__(self, MazeSolver: "MazeSolver", surface, x: int, y: int, state: NodeStates = NodeStates.FREE):
        self.maze_solver = MazeSolver
        self.surface = surface
        self.x = x
        self.y = y
        self.state: NodeStates = state
        self.animation_progress = 0
        self.color0 = COLOR_WHITE
        self.color1 = COLOR_WHITE
        self.active = False
        self.render = True

    def reset(self, animation=True):
        self.color0 = self.color1
        if self.active is True:
            # do not reset this time
            self.active = False
        else:
            # get initial state
            state_int = self.maze_solver.maze[self.y, self.x]
            state = NodeStates(state_int)
            color0, color1 = get_color(state)
            self.color1 = color1
            self.set_state(state, with_animation=animation)

    def set_state(
        self,
        state: NodeStates,
        with_animation: bool = True,
        color0: Optional[Tuple[int, int, int]] = None,
        color1: Optional[Tuple[int, int, int]] = None,
    ):
        assert isinstance(state, NodeStates)
        self.active = True
        self.render = True
        self.state: NodeStates = state
        if with_animation:
            self.animation_progress = 0
        else:
            self.animation_progress = 0.99
        if color0:
            self.color0 = color0
        if color1:
            self.color1 = color1

    def draw(self, color=None):
        self.animation_progress += 1 / ANIMATION_LENGTH
        if not color:
            color = interpolate_color(self.color0, self.color1, self.animation_progress)
        return pygame.draw.rect(self.surface, color, (self.x * CELLWIDTH, self.y * CELLWIDTH, CELLWIDTH, CELLWIDTH))
