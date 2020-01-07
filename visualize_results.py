import os
import open3d as o3d
import pdb
import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt

def createMesh(points, traingles):
	num_points = points.shape[0]
	num_triangles = triangles.shape[0]
	with open('test.obj', 'w') as f:
		for i in range(num_points):
			f.write('v ' + str(points[i][0]) + ' ' + str(points[i][1]) + ' ' + str(points[i][2]) + '\n')
		for i in range(num_triangles):
			f.write('f ' + str(triangles[i][0]) + ' ' + str(triangles[i][1]) + ' ' + str(triangles[i][2]) + '\n')


fileDir = 'build/Output/'
files = os.listdir(fileDir)

sort_dict = {}
new_files = []

num_files = 0
for file in files:
	if os.path.isdir(fileDir + file):
		continue
	if not file.split('.')[1] == 'pcd':
		continue
	file_num = int((file[5:]).split('.')[0])
	sort_dict[file_num] = file
	print(file)
	num_files += 1

for i in range(num_files):
	new_files.append(sort_dict[i])

files = new_files.copy()
del(new_files)


print(files)
# visualize files:

# mesh = o3d.io.read_triangle_mesh(fileDir + files[0])
# o3d.visualization.draw_geometries([mesh])
camera_parameters = o3d.camera.PinholeCameraParameters()
camera_parameters.extrinsic = np.array([[1,0,0,1],
                                           [0,1,0,0],
                                           [0,0,1,2],
                                           [0,0,0,1]])
viewer = o3d.visualization.Visualizer()
viewer.create_window()

control = viewer.get_view_control()
control.convert_from_pinhole_camera_parameters(camera_parameters)

for i in range(num_files):
	pcd = o3d.io.read_point_cloud(fileDir + files[i])
	viewer.add_geometry(pcd)
	viewer.run()
	viewer.capture_screen_image("build/Output/images/%04d.jpg" % i)