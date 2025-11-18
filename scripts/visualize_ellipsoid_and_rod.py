#!/usr/bin/env python3
"""
Visualize ellipsoid mesh (.msh) and rod (.msh or .geo) together in 3D
Usage:
    python visualize_ellipsoid_and_rod.py tumor_ellipsoid.msh ellipsoid_rod.msh
    python visualize_ellipsoid_and_rod.py tumor_ellipsoid.msh ellipsoid_rod.geo

Dependencies:
    pip install meshio matplotlib
"""
import sys
import meshio
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re

def read_geo_points(geo_file):
    points = []
    with open(geo_file, 'r') as f:
        for line in f:
            m = re.match(r'Point\(\d+\) = \{([\d\.eE\-]+), ([\d\.eE\-]+), ([\d\.eE\-]+),', line)
            if m:
                x, y, z = float(m.group(1)), float(m.group(2)), float(m.group(3))
                points.append([x, y, z])
    return points

def read_rod_file(rod_file):
    """Read rod points from either .geo or .msh file"""
    if rod_file.endswith('.geo'):
        return read_geo_points(rod_file)
    else:
        # Read .msh file
        rod_mesh = meshio.read(rod_file)
        return rod_mesh.points.tolist()

def plot_mesh_and_rod(meshfile, rodfile):
    mesh = meshio.read(meshfile)
    rod_points = read_rod_file(rodfile)
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    # Plot mesh nodes
    ax.scatter(mesh.points[:,0], mesh.points[:,1], mesh.points[:,2], s=10, c='b', label='Ellipsoid nodes', alpha=0.5)
    # Plot rod points and lines
    if rod_points:
        rod_points_arr = list(zip(*rod_points))
        ax.plot(rod_points_arr[0], rod_points_arr[1], rod_points_arr[2], c='r', lw=2, label='Nerve rod')
        ax.scatter(rod_points_arr[0], rod_points_arr[1], rod_points_arr[2], c='r', s=20)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Ellipsoid Mesh and Nerve Rod Visualization')
    ax.legend()
    plt.tight_layout()
    plt.show()

def main():
    if len(sys.argv) < 3:
        print('Usage: python visualize_ellipsoid_and_rod.py tumor_ellipsoid.msh ellipsoid_rod.msh')
        print('   or: python visualize_ellipsoid_and_rod.py tumor_ellipsoid.msh ellipsoid_rod.geo')
        sys.exit(1)
    plot_mesh_and_rod(sys.argv[1], sys.argv[2])

if __name__ == '__main__':
    main()
