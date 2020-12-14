from code.renderer import NodeStates, Renderer
from typing import Optional

import numpy as np

# define constants for rendering
WIDTH = 1000
HEIGHT = 1000
CELLWIDTH = 20
RENDER_SKIP = 30
ANIMATION_LENGTH = 20


class MazeSolver:
    def __init__(self, export_frames):
        # for unique node index
        self.node_counter = 0
        self.current_node = None

        # maze state_list
        # 0: free, 1: blocked, 2: start, 3: end
        self.maze = np.zeros((HEIGHT // CELLWIDTH, WIDTH // CELLWIDTH), dtype=np.uint8)  # 50*50
        self.start_coords = (0, 0)  # (x, y)
        self.end_coords = (49, 49)  # (x, y)
        self.x_bounds = (0, WIDTH // CELLWIDTH - 1)
        self.y_bounds = (0, HEIGHT // CELLWIDTH - 1)
        self.renderer = Renderer(self, start=self.start_coords, end=self.end_coords, export_frames=export_frames)
        self.reset()

    def reset(self):
        self.open: set["MazeSolverNode"] = set()
        self.closed: set["MazeSolverNode"] = set()
        self.add_node_to_open(self.create_node(*self.start_coords))
        self.shortest_path = []

    def set_maze_state(self, x, y, state_int):
        # state: 0=free, 1=blocked
        self.maze[y, x] = state_int

    def add_node_to_open(self, node):
        self.open.add(node)
        self.renderer.set_renderer_state(node.x, node.y, NodeStates.OPEN)

    def add_node_to_closed(self, node):
        self.closed.add(node)
        self.renderer.set_renderer_state(node.x, node.y, NodeStates.CLOSED)

    def create_node(self, x, y) -> "MazeSolverNode":
        # get index
        idx = self.node_counter
        self.node_counter += 1
        g_cost = self.calculate_g_cost(x, y)
        h_cost = self.calculate_h_cost(x, y)
        return MazeSolverNode(x, y, idx=idx, g_cost=g_cost, h_cost=h_cost)

    def calculate_g_cost(self, x, y):
        # g_cost: current path length
        if not self.current_node:
            current_cost = 0
            dx, dy = abs(x - self.start_coords[0]), abs(x - self.start_coords[1])
        else:
            current_cost = self.current_node.g_cost
            dx, dy = abs(x - self.current_node.x), abs(y - self.current_node.y)
        n_nondiag = abs(dx - dy)
        n_diag = max(dx, dy) - n_nondiag
        # 14 for diagonal traversing, 10 for hor. / vert.
        return current_cost + 10 * n_nondiag + 14 * n_diag

    def calculate_h_cost(self, x, y):
        # h_cost: heuristic cost, distance from node to end node
        if not self.current_node:
            dx, dy = abs(x - self.end_coords[0]), abs(x - self.end_coords[1])
        else:
            dx, dy = abs(x - self.end_coords[0]), abs(y - self.end_coords[1])
        n_nondiag = abs(dx - dy)
        n_diag = max(dx, dy) - n_nondiag
        # 14 for diagonal traversing, 10 for hor. / vert.
        return 14 * n_diag + 10 * n_nondiag

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
        return np.all([self.x_bounds[0] <= x <= self.x_bounds[1], self.y_bounds[0] <= y <= self.y_bounds[1]])

    def get_node_neighbour_coords(self, x, y):  # 0 ms
        # returns coords tuples (x, y) of traversable neighbours as list
        coords_candidates_list = []
        for x_shift in range(-1, 2):
            for y_shift in range(-1, 2):
                if x_shift == 0 and y_shift == 0:
                    continue
                coords_candidates_list.append((x + x_shift, y + y_shift))

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

    def find_node_in_closed(self, x, y) -> Optional["MazeSolverNode"]:
        for node in self.closed:
            if node.x == x:
                if node.y == y:
                    return node
        return None

    def find_node_in_open(self, x, y) -> Optional["MazeSolverNode"]:
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
            if current_node.x == self.start_coords[0]:
                if current_node.y == self.start_coords[1]:
                    run = False
        return_list = list(zip(x_coords, y_coords))
        return_list.reverse()
        self.shortest_path = return_list

    def apply_shortest_path(self):
        self.renderer.set_renderer_state(*self.end_coords, NodeStates.END)
        if len(self.shortest_path) > 0:
            segment = self.shortest_path.pop()
            x, y = segment
            self.renderer.set_renderer_state(x, y, NodeStates.BESTPATH)
            return False
        else:  # when path is completely applied
            self.renderer.reset_nodes()
            return True

    def astar_step(self):  # 0ms
        # solving path finding
        self.current_node = self.get_node_fcost_min()
        self.add_node_to_closed(self.current_node)

        if (self.current_node.x, self.current_node.y) == self.end_coords:
            print("path has been found")
            self.export_shortest_path()
            return True

        # get neighbours of current_node
        neighbour_coords = self.get_node_neighbour_coords(self.current_node.x, self.current_node.y)

        for coords in neighbour_coords:  # for each neighbour
            # if node (x,y) is in closed -> skip
            if self.find_node_in_closed(*coords):
                continue

            # calculate new g_cost for neighbour node
            new_g_cost = self.calculate_g_cost(*coords)

            # find node (x,y) in open
            node_in_open = self.find_node_in_open(*coords)
            # node is in open
            if node_in_open:  # node is in open
                if new_g_cost < node_in_open.g_cost:
                    # execute below
                    node_in_open._set_g_cost(new_g_cost)
                    neighbour_node = node_in_open
                else:
                    continue  # continue to next node
            # node is not in open
            else:
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
                if path_found is False:
                    path_found = self.astar_step()
            for _ in range(2):
                if path_applied is False and path_found:
                    path_applied = self.apply_shortest_path()


class MazeSolverNode:
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
        # g_cost: distance from starting node
        # h_cost: distance from end ndoe
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
