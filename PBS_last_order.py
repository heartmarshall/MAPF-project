import nbimporter

from heapq import heappop, heappush, heapify
from map import Map




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


class SearchTree: 
        
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

class CATable:
    '''
    Class, which implements collision avoidance table for effective checking collisions with dynamic obstacles
    '''
    def __init__(self, dyn_obst_traj, obst_stopped):       
        self.pos_time_table = dict()
        self.max_time_table = dict()
        
        self.obst_stopped = obst_stopped
        
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
        for obst in self.obst_stopped:
            if obst[0][0] == i and obst[0][1] == j:
                if t + 1 >= obst[1]:
                    return False
                else:
                    break 
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


def get_neighbors_wrt_time(i, j, t, grid_map: Map, ca_table: CATable):
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
    if ca_table.check_move(i, j, i, j, t):
        result.append((i,j))
    return result
      
    
def astar_timesteps(grid_map, ca_table, start_i, start_j, goal_i, goal_j, 
                    heuristic_func=manhattan_distance, search_tree=SearchTree, 
                    lower_time_limit=0):
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
        
        if (best == goal) and (steps >= lower_time_limit):
            return True, best_node, steps, nodes_created, ast.OPEN, ast.CLOSED
        
        for succ in get_neighbors_wrt_time(*best, best_node.g, grid_map, ca_table):
            new_node = Node(*succ, best_node.g + grid_map.compute_cost(*best, *succ), 
                            (lower_time_limit - best_node.g) * (lower_time_limit > best_node.g) +
                            heuristic_func(*succ, *goal), parent=best_node)
            nodes_created += 1
            if not ast.was_expanded(new_node):
                ast.add_to_open(new_node)
        if steps > 1000:
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


def list_pair_from_list_nodes(path):
    p_path = []
    for i, node in enumerate(path):
        pair = (node.i, node.j)
        p_path.append(pair)
    return p_path


def run_with_order(traj_prev, traj_stopped, start, goal, search_func, map: Map, lower_time_limit=0, **kwargs):
    
    ca_table = CATable(traj_prev, traj_stopped)
    
    result = search_func(map, ca_table, start[0], start[1], goal[0], goal[1], 
                         lower_time_limit=lower_time_limit, **kwargs)
    
    if result[0]:
        goal = result[1]
        path = make_path(goal)[0]
        p_path = list_pair_from_list_nodes(path)
        steps = result[2]
        if steps > lower_time_limit:
            lower_time_limit = steps
        return True, p_path, steps, lower_time_limit
    else:
        return False, None, 0, lower_time_limit


def FixedOrderSearch(starts, goals, map:Map,*, order=None, search_func=astar_timesteps, **kwargs):
    traj = []
    traj_stopped = []
    if order == None:
        order = list(range(len(starts)))
    
    lower_time_limit = 0
    for agent in range(len(starts)):
        res, p_path, steps, lower_time_limit = run_with_order(traj, 
                                                              traj_stopped,
                                                              starts[order[agent]], 
                                                              goals[order[agent]], 
                                                              search_func=search_func, 
                                                              map=map, 
                                                              lower_time_limit=lower_time_limit, **kwargs)
        map.clear_tmp()
        if res:
            traj.append(p_path)
            traj_stopped.append((goals[order[agent]], steps))
        else:
            pass
            # print(f"agent {agent} has failed")
    
    lengths = []
    for i in traj:
        lengths.append(len(i))
    return traj, max(lengths)