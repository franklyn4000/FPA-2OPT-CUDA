import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import math
import pandas as pd

def read_fittest_from_file(file_path):

    with open(file_path, 'r') as f:
        lines = f.readlines()

    # Create a generator for every three lines
    path_points = zip(*(iter(lines),) * 3)

    # Create path list where every point is a tuple (X, Y, Z)
    path = [(float(x.strip()), float(y.strip()), float(z.strip())) for x, y, z in path_points]

    return path

def load_array_from_csv(filename):
    z_2d_array_loaded = np.genfromtxt(filename, delimiter=",")


    y_len, x_len = z_2d_array_loaded.shape
    y = np.repeat(np.arange(y_len), x_len)
    x = np.tile(np.arange(x_len), y_len)
    z = z_2d_array_loaded.flatten()

    plt.figure(figsize=(10, 7))
    plt.scatter(x, y, c=z, cmap='jet')
    plt.colorbar(label='Height')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('2D Top View of the Heightmap')
    plt.show()

    return z_2d_array_loaded


def visualize_3D(loaded, fittest_paths):
    theta = math.radians(160)

    camera = dict(
        up=dict(x=0, y=0, z=1),
        center=dict(x=0, y=0, z=0),
        eye=dict(x=math.cos(theta)*1.7, y=math.sin(theta)*1.7, z=1.5*1.7)
    )

    lighting = dict(ambient=0.45,  # Ambient light intensity ranging from 0 to 1
                      diffuse=0.72,  # Diffuse light intensity ranging from 0 to 1
                      specular=0.21,  # Specular light intensity ranging from 0 to 2
                      roughness=0.65,  # Roughness of the surface ranging from 0 to 1
                      fresnel=0.2)

    surface = go.Surface(z=loaded, colorscale='Blues', lighting = lighting, cmin=1000,
            cmax=4000)

    data = [surface]

    # Create a Scatter3D plot for each path
    for fittest_path in fittest_paths:
        fittest_path_x = [float(point[0]) for point in fittest_path]
        fittest_path_y = [float(point[1]) for point in fittest_path]
        fittest_path_z = [float(point[2]) for point in fittest_path]

        scatter = go.Scatter3d(x=fittest_path_x, y=fittest_path_y, z=fittest_path_z, mode='lines')
        data.append(scatter)

    fig = go.Figure(data=data)

    fig.update_layout(scene=dict(xaxis_title='X', yaxis_title='Y', zaxis_title='Z', camera=camera,aspectmode='manual',
            aspectratio=dict(x=2.2, y=2.2, z=1)),
                      width=1800, margin=dict(r=20, l=10, b=10, t=10),
                      paper_bgcolor='rgba(0,0,0,0)',
                      plot_bgcolor='rgba(0,0,0,0)')
    fig.show()


loaded = load_array_from_csv('switzerland.csv')
fittest_path_cpu = read_fittest_from_file('fittest4.csv')
fittest_path_gpu = read_fittest_from_file('fittest5.csv')
visualize_3D(loaded, [fittest_path_cpu, fittest_path_gpu])
