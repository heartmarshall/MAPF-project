import map
import gen_test
import nbimporter
import matplotlib.pyplot as plt
from random import randint
from tqdm import tqdm
from IPython.display import HTML
from PIL import Image, ImageDraw, ImageOps
from IPython.display import Image as Img
from IPython.display import display

def generate_colors():
    colors = []
    for i in (0, 128, 255):
        for j in (128, 255, 0):
            for k in (0, 255, 128):
                triple = (i, j, k)
                colors.append(triple)
    return colors

def draw(grid_map: map.Map, paths, pathlen, output_filename = 'animated_trajectories'):
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
    for step in tqdm(range(pathlen)):
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
    images[0].save('./'+output_filename+'.png', save_all=True, append_images=images[1:], optimize=False, duration=500/quality, loop=0)
    