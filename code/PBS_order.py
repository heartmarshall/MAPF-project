import copy
import math
import matplotlib.pyplot as plt
import numpy as np
from heapq import heappop, heappush, heapify
from random import randint
import time
from IPython.display import HTML
from PIL import Image, ImageDraw, ImageOps
from IPython.display import Image as Img
from IPython.display import display
from sys import float_info
from map_reading import read_map_from_movingai_file

EPS = float_info.epsilon


class Map:
    def __init__(self):
        self._width = 0
        self._height = 0
        self._cells = []
        self.tmp_obst = []
    

    def read_from_string(self, cell_str, width, height):
        self._width = width
        self._height = height
        self._cells = [[0 for _ in range(width)] for _ in range(height)]
        cell_lines = cell_str.split("\n")
        i = 0
        j = 0
        for l in cell_lines:
            if len(l) != 0:
                j = 0
                for c in l:
                    if c == '.':
                        self._cells[i][j] = 0
                    elif c == '#' or c == 'T' or c == '@':
                        self._cells[i][j] = 1
                    else:
                        continue
                    j += 1
                if j != width:
                    raise Exception("Size Error. Map width = ", j, ", but must be", width )
                
                i += 1

        if i != height:
            raise Exception("Size Error. Map height = ", i, ", but must be", height )
    

    def set_grid_cells(self, width, height, grid_cells):
        self._width = width
        self._height = height
        self._cells = grid_cells


    def in_bounds(self, i, j):
        return (0 <= j < self._width) and (0 <= i < self._height)
    
    def traversable(self, i, j):
        return not self._cells[i][j]


    def get_neighbors(self, i, j):
        neighbors = []
        delta = [[0, 1], [1, 0], [0, -1], [-1, 0]]

        for d in delta:
            if self.in_bounds(i + d[0], j + d[1]) and self.traversable(i + d[0], j + d[1]):
                neighbors.append((i + d[0], j + d[1]))
                
        return neighbors

    def get_size(self):
        return (self._height, self._width)

    def add_obstacle(self, i, j):
        self._cells[i][j] = 1
        self.tmp_obst.append((i, j))
    
    def clear_tmp(self):
        for i, j in self.tmp_obst:
            self._cells[i][j] = 0
        self.tmp_obst = []



class Node:

    def __init__(self, i, j, g = 0, h = 0, f = None, parent = None):
        self.i = i
        self.j = j
        self.g = g
        self.h = h
        if f is None:
            self.f = self.g + h
        else:
            self.f = f        
        self.parent = parent

        
    
    def __eq__(self, other):
        return (self.i == other.i) and (self.j == other.j) and (self.g == other.g)
    
    def __hash__(self):
        ij = self.i, self.j
        return hash(ij)


    def __lt__(self, other): 
        return (self.f < other.f) or (self.f == other.f and self.h < other.h)
        
        
        
class SearchTree: #Non list-based implementation of the search tree
        
    def __init__(self):
        self._open = []
        self._closed = set()
        
    def open_is_empty(self):
        return len(self._open) == 0

    def add_to_open(self, item):
        heappush(self._open, item)
        return   
        

    def get_best_node_from_open(self):
        node = heappop(self._open)
        while node in self._closed:
            node = heappop(self._open)
        return node
    
    def add_to_closed(self, item):
        self._closed.add(item)
        
    def was_expanded(self, item):
        return item in self._closed
    
    @property
    def OPEN(self):
        return self._open
    
    @property
    def CLOSED(self):
        return self._closed



def manhattan_distance(i1, j1, i2, j2):
    '''
    Returns a manhattan distance between two cells

    Parameters
    ----------
    i1, j1 : int, int
        Position of first cell on the grid map
    i2, j2 : int, int
        Position of second cell on the grid map

    Returns
    -------
    int
        Manhattan distance between two cells

    '''
    return abs(i1 - i2) + abs(j1 - j2)
    
    
    
    
def compute_cost(i1, j1, i2, j2):
    '''
    Computes cost of simple moves

    Parameters
    ----------
    i1, j1 : int, int
        Position of first cell on the grid map
    i2, j2 : int, int
        Position of second cell on the grid map

    Returns
    -------
    int
        Cost of the move between two neighbouring cells

    '''

    d = abs(i1 - i2) + abs(j1 - j2)
    if d == 0:  # wait
        return 1
    elif d == 1:  # cardinal move
        return 1
    else:
        raise Exception('Trying to compute the cost of non-supported move!')
        
        
        
        
class CATable:
    '''
    Class, which implements collision avoidance table for effective checking collisions with dynamic obstacles
    '''
    def __init__(self, dyn_obst_traj):       
        self.pos_time_table = dict()
        self.max_time_table = dict()
        
        for obst_id, obstacle in enumerate(dyn_obst_traj):
            for t, (i, j) in enumerate(obstacle):
                self.pos_time_table[(i, j, t)] = obst_id
            
            self.max_time_table[obstacle[-1]] = len(obstacle) - 1 
            
    def __check_pos_at_time(self, i, j, t):
        '''
        Checks, that cell (i, j) is occupied at moment t
        
        Parameters
        ----------
        i, j: int, int
            Position of cell on the grid map
        t : int
             Time moment to check
            
        Returns
        -------
        bool
            False, if cell is occupied at time moment t
            True, if not occupied at time moment t
        '''
        
        return not (i, j, t + 1) in self.pos_time_table.keys()
                
    def __check_rev_move(self, i1, j1, i2, j2, t_start):
        '''
        Checks, that the given move does not result in edge collision
        
        Parameters
        ----------
        i1, j1 : int, int
            Start cell of the move
        i2, j2 : int, int
            Target cell of the move
        t_start : int
             Time when the move starts
            
        Returns
        -------
        bool        
            True if the given move does not result in the edge collision
            False if the given move does results in the edge collision
        '''
        
        return not ((i1, j1, t_start + 1) in self.pos_time_table.keys() \
                    and (i2, j2, t_start) in self.pos_time_table.keys() \
                    and self.pos_time_table[(i1, j1, t_start + 1)] == self.pos_time_table[(i2, j2, t_start)])

    def check_move(self, i1, j1, i2, j2, t_start):
        '''
        Checks if the move between (i1, j1) and (i2, j2) 
        at moment (t_start -> t_start+1) leads 
        to the collision with a dynamic obstacle.

        Parameters
        ----------
        i1, j1 : int, int
            Start cell of the move
        i2, j2 : int, int
            Target cell of the move
        t_start : int
             Time step when the move starts
            
        Returns
        -------
        bool
            Is the move valid (true) or will it lead to a collision (false)
        '''
        
        
        return self.__check_pos_at_time(i2, j2, t_start) \
                and self.__check_rev_move(i1, j1, i2, j2, t_start)
                
                
                
def get_neighbors_wrt_time(i, j, t, grid_map, ca_table: CATable):
    '''
    Returns a list of neighbouring cells as (i, j) tuples. 
    
    Should return such neighbours, that result
    from non-colliding actions (cardinal moves and wait) w.r.t.
    the given time step.
    
    I.e. the succesor cell should be free at the next time step,
    as well at the move should not result in the edge collision.
    
    Parameters
    ----------
    i, j : int
        Cell coordinates
    grid_map : Map
        An additional domain information (such as grid map).
    ca_table : CATable
        Collision avoidance table

    Returns
    -------
    neighbours : list[tuple[int, int]]
        List of neighbours grid map (i, j) coordinates
    '''

    neighbors = grid_map.get_neighbors(i, j)
    neighbors.append((i, j))
    result = []

    for i2, j2 in neighbors:
        if ca_table.check_move(i, j, i2, j2, t):
            result.append((i2, j2))

    return result
    
    
    
    
def astar_timesteps(grid_map, ca_table, start_i, start_j, goal_i, goal_j, heuristic_func = None, search_tree = None):
    '''
    Runs A* search algorithm without re-expansion on dynamic obstacles domain.

    Parameters
    ----------
    grid_map : Map
        An additional domain information (such as grid map).
    ca_table : CATable
        Collision avoidance table
    start_i, start_j : int, int
        Start cell
    goal_i, goal_j  : int, int
        Goal cell
    heuristic_func : function
        Heuristic function
    search_tree : type 
        Search tree data structure

    Returns
    -------
    path_found : bool
        Path was found or not.  
    last_node : Node
        The last node in path. None if path was not found.
    steps : int
        The number of search steps
    noodes_created : int
        The number of nodes, which were created and stored during the search process (size of the resultant search tree)
    open : iterable object
        Iterable collection of OPEN nodes
    expanded : iterable object
        Iterable collection of the expanded nodes
    '''

    ast = search_tree()
    steps = 0
    nodes_created = 0
    
    goal = (goal_i, goal_j)
    start = Node(start_i, start_j, 0, heuristic_func(start_i, start_j, *goal), parent=None)
    ast.add_to_open(start)
    
    while not ast.open_is_empty():
        steps += 1
        best_node = ast.get_best_node_from_open()
        best = (best_node.i, best_node.j)
        ast.add_to_closed(best_node)
        
        if (best == goal):
            return True, best_node, steps, nodes_created, ast.OPEN, ast.CLOSED
        
        for succ in get_neighbors_wrt_time(*best, best_node.g, grid_map, ca_table):
            new_node = Node(*succ, best_node.g + compute_cost(*best, *succ), heuristic_func(*succ, *goal), parent=best_node)
            nodes_created += 1
            if not ast.was_expanded(new_node):
                ast.add_to_open(new_node)
        if steps > 500:
            break
    
    return False, None, steps, nodes_created, ast.OPEN, ast.CLOSED
    
    
    
    
def make_path(goal):
    '''
    Creates a path by tracing parent pointers from the goal node to the start node
    It also returns path's length.
    '''

    length = goal.g
    current = goal
    path = []
    while current.parent:
        path.append(current)
        current = current.parent
    path.append(current)
    return path[::-1], length
    
    
    
    
def draw(grid_map, dyn_obst_traj, path, output_filename = 'animated_trajectories'):
    '''
    Auxiliary function that visualizes the environment.
    
    The function assumes that nodes_opened/nodes_expanded
    are iterable collestions of search nodes
    '''

    k = 30
    quality = 6
    height, width = grid_map.get_size()
    h_im = height * k
    w_im = width * k
    pathlen = len(path)
    
    step = 0
    images = []
    agent_color = randint(0, 255), randint(0, 255), randint(0, 255)
              
    while step < pathlen:
        for n in range(0, quality):
            im = Image.new('RGB', (w_im, h_im), color = 'white')
            draw = ImageDraw.Draw(im)
            
            # draw static obstacles
            for i in range(height):
                for j in range(width):
                    if(not grid_map.traversable(i, j)):
                        draw.rectangle((j * k, i * k, (j + 1) * k - 1, (i + 1) * k - 1), fill=( 70, 80, 80 ))
                   
            
            #draw agent
            curr_node = path[step]
            next_node = path[min(pathlen - 1, step + min(n, 1))]

            di = n * (next_node.i - curr_node.i) / quality
            dj = n * (next_node.j - curr_node.j) / quality

            draw.ellipse((float(curr_node.j + dj + 0.2) * k, 
                          float(curr_node.i + di + 0.2) * k, 
                          float(curr_node.j + dj + 0.8) * k - 1, 
                          float(curr_node.i + di + 0.8) * k - 1), 
                          fill=agent_color, width=0)
            
            # draw dynamic obstacles 
            for i in range(len(dyn_obst_traj)):
                curr_pos = dyn_obst_traj[i][min(len(dyn_obst_traj[i]) - 1, step)]
                next_pos = dyn_obst_traj[i][min(len(dyn_obst_traj[i]) - 1, step + min(n, 1))]
                
                di = n * (next_pos[0] - curr_pos[0]) / quality
                dj = n * (next_pos[1] - curr_pos[1]) / quality
            
                draw.rounded_rectangle((float(curr_pos[1] + dj + 0.2) * k, 
                              float(curr_pos[0] + di + 0.2) * k, 
                              float(curr_pos[1] + dj + 0.8) * k - 1, 
                              float(curr_pos[0] + di + 0.8) * k - 1), 
                              fill=(50, 50, 50), width=0, radius=k * 0.2)
            im = ImageOps.expand(im, border=2, fill='black')
            images.append(im)
        step += 1
    images[0].save('./'+output_filename+'.png', save_all=True, append_images=images[1:], optimize=False, duration=500/quality, loop=0)
    
    
    
    
def generate_colors():
    colors = []
    for i in (0, 128, 255):
        for j in (128, 255, 0):
            for k in (0, 255, 128):
                triple = (i, j, k)
                colors.append(triple)
    return colors
    
    
def draw(grid_map, paths, pathlen, output_filename = 'animated_trajectories'):
    '''
    Auxiliary function that visualizes the environment.
    
    The function assumes that nodes_opened/nodes_expanded
    are iterable collestions of search nodes
    '''

    k = 30
    quality = 6
    height, width = grid_map.get_size()
    h_im = height * k
    w_im = width * k
    
    step = 0
    images = []

    colors = generate_colors()
    while step < pathlen:
        for n in range(0, quality):
            im = Image.new('RGB', (w_im, h_im), color = 'white')
            draw = ImageDraw.Draw(im)
            
            # draw static obstacles
            for i in range(height):
                for j in range(width):
                    if(not grid_map.traversable(i, j)):
                        draw.rectangle((j * k, i * k, (j + 1) * k - 1, (i + 1) * k - 1), fill=( 70, 80, 80 ))
                   
            
            # draw dynamic obstacles 
            for i in range(len(paths)):
                color = (0, 0, 0)
                if len(colors) >= len(paths):
                    color = colors[i]

                curr_pos = paths[i][min(len(paths[i]) - 1, step)]
                next_pos = paths[i][min(len(paths[i]) - 1, step + min(n, 1))]
                
                di = n * (next_pos[0] - curr_pos[0]) / quality
                dj = n * (next_pos[1] - curr_pos[1]) / quality
            
                draw.rounded_rectangle((float(curr_pos[1] + dj + 0.2) * k, 
                              float(curr_pos[0] + di + 0.2) * k, 
                              float(curr_pos[1] + dj + 0.8) * k - 1, 
                              float(curr_pos[0] + di + 0.8) * k - 1), 
                              fill=color, width=0, radius=k * 0.2)
            im = ImageOps.expand(im, border=2, fill='black')
            images.append(im)
        step += 1
    images[0].save('./'+output_filename+'.png', save_all=True, append_images=images[1:], optimize=False, duration=500/quality, loop=0)
    


def list_pair_from_list_nodes(path):
    p_path = []
    for node in path:
        pair = (node.i, node.j)
        p_path.append(pair)
    return p_path


def run_with_order(traj_prev, id, starts, goals, search_func, map: Map, *args):
    
    ca_table = CATable(traj_prev)

    start = Node(*starts[id])
    goal = Node(*goals[id])
    result = search_func(map, ca_table, start.i, start.j, goal.i, goal.j, *args)
    if result[0]:
        goal = result[1]
        path = make_path(goal)[0]
        p_path = list_pair_from_list_nodes(path)
    else:
        return False, None
    return True, p_path


def PBS(starts, goals, search_func, map: Map, *args):
    order = [i for i in range(len(starts))]

    traj = []
    for index, agent in enumerate(order):

        for k in range(len(order)):
            i, j = goals[order[k]]
            if (i, j) != goals[agent]:
                map.add_obstacle(i, j)

        res, p_path = run_with_order(traj, agent, starts, goals, search_func, map, *args)
        map.clear_tmp()
        if res:
            traj.append(p_path)

    lengths = []
    for i in traj:
        lengths.append(len(i))
    return traj, max(lengths)