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

    return (width, height, cells)
