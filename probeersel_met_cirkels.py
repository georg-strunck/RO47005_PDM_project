# -*- coding: utf-8 -*-
"""
Created on Wed Jan 11 13:10:11 2023

@author: Tibbe Lukkassen
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np


def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

vec1 = [1,1,1]
vec2 = [2,2,2]

ax1 = 1
ay1 = 1
bx1 = 2
by1 = 2
ax2 = 2
ay2 = 2
bx2 = 5
by2 = 1

R = 1

le1 = np.sqrt((bx1-ax1)**2 + (by1-ay1)**2) 
v1x = (bx1-ax1)/le1
v1y = (by1-ay1)/le1

le2 = np.sqrt((bx2-ax2)**2 + (by2-ay2)**2) #length of A2-B2 segment
v2x = (bx2-ax2) / le2
v2y = (by2-ay2) / le2

px1= ax1 + v1y*R
py1= ay1 - v1x*R
px2= ax2 + v2y*R
py2= ay2 - v2x*R

den = v1x*v2y - v2x*v1y
k1 = (v2y*(px2-px1) - v2x*(py2-py1)) / den
k2 = (v1y*(px2-px1) - v1x*(py2-py1)) / den
cx = px1 + k1*v1x   
cy = py1 + k1*v1y 

tx1 = ax1 + k1*v1x
ty1 = ay1 + k1*v1y

fig, ax = plt.subplots()
circle = plt.Circle((cx,cy),R, fill = False)

ax.add_patch(circle)
plt.plot([ax1, bx1, bx2], [ay1, ay2, by2])
plt.plot(cx,cy, 'ro')
plt.plot(tx1,ty1, 'ro')
plt.show()