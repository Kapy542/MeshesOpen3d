# -*- coding: utf-8 -*-
"""
Created on Sat Jan 13 12:58:40 2024

@author: kapyla
"""

import numpy as np
import open3d as o3d

cube_pts = np.array([[-1,-1,0], [1,-1,0], [1,1,0], [-1,1,0], [-1,-1,2], [1,-1,2], [1,1,2], [-1,1,2]]).astype(float)
pyramid_pts = np.array([[-1,-1,0], [1,-1,0], [1,1,0], [-1,1,0], [0,0,1]]).astype(float)

cube_triangles = np.array([[0,1,2], [2,3,0], [4,5,6], [6,7,4], [4,7,6]])
pyramid_triangles = np.array([[0,1,4], [1,2,4], [2,3,4], [3,0,4], [0,1,2], [2,3,0]])

#%% Create pointclouds
cube_pcd = o3d.geometry.PointCloud()
cube_pcd.points = o3d.utility.Vector3dVector(cube_pts)
o3d.visualization.draw_geometries([cube_pcd])

pyramid_pcd = o3d.geometry.PointCloud()
pyramid_pcd.points = o3d.utility.Vector3dVector(pyramid_pts)
o3d.visualization.draw_geometries([pyramid_pcd])

#%% Create meshes
cube_mesh = o3d.geometry.TriangleMesh()
pyramid_mesh = o3d.geometry.TriangleMesh()

pyramid_mesh.has_triangle_material_ids()
pyramid_mesh.has_triangles()
pyramid_mesh.has_vertices()
pyramid_mesh.is_empty()
pyramid_mesh.is_vertex_manifold()

cube_mesh.vertices = o3d.cpu.pybind.utility.Vector3dVector( cube_pts )
cube_mesh.triangles = o3d.cpu.pybind.utility.Vector3iVector( cube_triangles )

pyramid_mesh.vertices = o3d.cpu.pybind.utility.Vector3dVector( pyramid_pts )
pyramid_mesh.triangles = o3d.cpu.pybind.utility.Vector3iVector( pyramid_triangles )

o3d.visualization.draw_geometries([cube_mesh])
o3d.visualization.draw_geometries([pyramid_mesh])
