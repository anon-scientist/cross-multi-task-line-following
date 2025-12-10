"""
This file needs to be executed in the project root directory and creates
* the bitmap with 150 tracks, defined by colored lines, for the ground plane in Gazebo
* a file tracks_defs.txt which contains info about the 150 tracks: 
    colors of the left, right and centered line
    lamps that should be turned on in the first and 2nd half of the track
    starting y position  of the robot on the ground plane
                

Robot starting y position for each track is computed as:
  plt         = edge length of quadratic ground plane (currently 100m)
  nr_task_y   = number of tasks for a single y position (currently 75)
  i           = track number

  Robot starting position for  track number y  = ((plt / nr_task_y) * (i+1)) - ((plt / nr_task_y) / 3)
"""
    
from PIL import Image, ImageDraw
from itertools import permutations
from itertools import product
import numpy as np ;

# Farben definieren (RGB)
color_array = [
    (255, 0, 0, "red"),     # Rot
    (0, 255, 0, "green"),     # Gruen
    (0, 0, 255, "blue"),     # Blau
    (0,255,255, "cyan"),     # Tuerkis
#    (255, 255, 0,"yellow"),   # Gelb
    (255, 0, 255,"magenta"),   # Magenta
    (0, 0, 0,"black")        # Black
]


kombis = product(color_array, repeat=3)


color_combinations = [
    kombi for kombi in kombis
    if len(set(kombi)) > 1                     # mind. 2 verschiedene Farben
    and not (kombi[0] == kombi[1] or kombi[1] == kombi[2])  # keine gleichen nebeneinander
]

# Bildkonfiguration
start_spacing = 0
stripe_height = 20
stripe_height_middle = 4
group_spacing = 22
group_height = 2 * stripe_height + stripe_height_middle

exp_nr = count_exp = 150


total_height =  exp_nr * (group_height + group_spacing) + start_spacing

# Bild erstellen
image = Image.new("RGB", (total_height, int(total_height/2)), "white")
draw = ImageDraw.Draw(image)

print(total_height, "x", int(total_height/2))

# Linien zeichnen
y = total_height/2 - group_spacing

f=open('track_defs.txt','w',encoding="latin1") ;
#f.write(str(count_exp)+" \n") ;

first_half = color_combinations[:exp_nr // 2]
second_half = color_combinations[exp_nr // 2:]
index = 0 ;
for color in first_half :
    # Oben
    y -= stripe_height
    draw.rectangle([(0, y), (total_height/2, y + stripe_height)], fill=color[0][0:3])

    # Mitte
    y -= stripe_height_middle
    draw.rectangle([(0, y), (total_height/2, y + stripe_height_middle)], fill=color[1][0:3])

    # Unten
    y -= stripe_height
    draw.rectangle([(0, y), (total_height/2, y + stripe_height)], fill=color[2][0:3])
    y -= group_spacing  # Abstand zur naechsten Gruppe

    starty  = ((100. / 75.) * (index+1)) - ((100 / 75.) / 3)

    rnd = np.random.randint(0,6,size=[2]) ;
    f.write(f"{color[0][3]} {color[1][3]} {color[2][3]} - {rnd[0]} {rnd[1]} - 0 {starty}\n" ) ;
    index += 1 ;
    
index = 0 ;
if(len(second_half) <=0):
    print("empty")
else:
    y = total_height/2 - group_spacing
    for color in second_half :
        # Oben
        y -= stripe_height
        draw.rectangle([(total_height/2, y), (total_height, y + stripe_height)], fill=color[0][0:3])

        # Mitte
        y -= stripe_height_middle
        draw.rectangle([(total_height/2, y), (total_height, y + stripe_height_middle)], fill=color[1][0:3])

        # Unten
        y -= stripe_height 
        draw.rectangle([(total_height/2, y), (total_height, y + stripe_height)], fill=color[2][0:3])
        y-= group_spacing  # Abstand zur naechsten Gruppe
        rnd = np.random.randint(0,6,size=[2]) ;

        starty  = ((100. / 75.) * (index+1)) - ((100 / 75.) / 3)
        f.write(f"{color[0][3]} {color[1][3]} {color[2][3]} - {rnd[0]} {rnd[1]} - 50 {starty}\n" ) ;
        index += 1 ;

f.close() ;   
print("Max Experimente 150")
print("Experiment Anzahl: ", count_exp)

# Bild speichern
image.save("./simulation/gazebo/models/colored_ground_plane/tracks/lines.png")
print("Bild gespeichert als 'lines.png'")


