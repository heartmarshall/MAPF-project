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
            raise Exception("Size Error. Map height = ", i, ", but must be", height)
        
        return self
        

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
    
    @classmethod
    def read_map_from_movingai_file(path):
        map_file = open(path)
        map_file.readline()
        line = map_file.readline()
        height = int(line.split()[1])
        line = map_file.readline()
        width = int(line.split()[1])
        line = map_file.readline()


        cells = [[0 for _ in range(width)] for _ in range(height)]

        for i, l in enumerate(map_file):
            for j, c in enumerate(l):
                if j == width:
                    break
                
                if c == '.':
                    cells[i][j] = 0
                elif c == "#" or c == "@" or c == "T":
                    cells[i][j] = 1
                else:
                    continue

            if(i == height):
                break

        return Map(width, height, cells)
    
    
    def compute_cost(self, i1, j1, i2, j2):
        d = abs(i1 - i2) + abs(j1 - j2)
        if d == 0:  # wait
            return 1
        elif d == 1:  # cardinal move
            return 1
        else:
            raise Exception('Trying to compute the cost of non-supported move!')