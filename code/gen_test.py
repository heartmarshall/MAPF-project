import numpy as np 
import random
import map

def gen_tasks(task_map: map.Map, num_of_tasks, mode="sides"):
    width, height = task_map.get_size()
    avalable_points = []
    if mode == "sides":
        for i in range(1, width - 1):
            if task_map.traversable(i, 0):
                avalable_points.append((i, 0))
                
            if task_map.traversable(i, height - 1):
                avalable_points.append((i,height - 1))    
        
        for j in range(1, height - 1):
            if task_map.traversable(0, j):
                avalable_points.append((0,j))    
            
            if task_map.traversable(width - 1, j):
                avalable_points.append((width - 1, j))
    elif mode == "all":
        for i in range(1, width - 1):
            for j in range(1, height - 1):
                if task_map.traversable(i,j):
                    avalable_points.append((i,j))  
    else:
        return None, None  
    # random.seed(100)
    starts = random.sample(avalable_points, num_of_tasks)
    finishes = random.sample(avalable_points, num_of_tasks)
    return starts, finishes