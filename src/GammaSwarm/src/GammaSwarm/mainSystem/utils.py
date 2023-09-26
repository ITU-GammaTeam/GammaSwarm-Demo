import math
import numpy as np
import sys
from numpy.linalg import norm, solve
from State import *
from itertools import tee
from rtree import index
import uuid
import plotly as py
from plotly import graph_objs as go
import os
import yaml


def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    temp = np.arctan2(y, x)
    phi = temp /(math.pi/180)
    return rho, phi


def pol12cart(rho, phi):
    x = rho * np.cos(math.radians(phi))
    y = rho * np.sin(math.radians(phi))
    return x, y


def distP(p1, p2):
    distance_square = math.pow(p2.x - p1.x, 2) + math.pow(p1.y - p2.y, 2)
    return math.sqrt(distance_square)


def dist(x1, y1, x2, y2):
    distance_square = math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2)
    return math.sqrt(distance_square)


def dist_of_two_pt(p1, p2):
    return dist(p1[0], p1[1], p2[0], p2[1])


def dist3d(x1, y1, z1, x2, y2, z2):
    distance_square = math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2) + math.pow(z2 - z1, 2)
    return math.sqrt(distance_square)


def principalAngle(angle):
    if angle > 180:
        angle = angle - 360
    elif angle < -180:
        angle = angle + 360
    return angle


def line_intersect(p1, p2, p3, p4):
    """ returns a (x, y) tuple or None if there is no intersection """
    Ax1 = p1.x
    Ay1 = p1.y
    Ax2 = p2.x
    Ay2 = p2.y
    Bx1 = p3.x
    By1 = p3.y
    Bx2 = p4.x
    By2 = p4.y

    d = (By2 - By1) * (Ax2 - Ax1) - (Bx2 - Bx1) * (Ay2 - Ay1)
    if d:
        uA = ((Bx2 - Bx1) * (Ay1 - By1) - (By2 - By1) * (Ax1 - Bx1)) / d
        uB = ((Ax2 - Ax1) * (Ay1 - By1) - (Ay2 - Ay1) * (Ax1 - Bx1)) / d
    else:
        return False

    if not (0 <= uA <= 1 and 0 <= uB <= 1):
        return False

    return True

def rotate_points(points,rotation_angle,center=[0,0],clockwise=False,around="z"):
    if around == "z":
        idx1,idx2= 0,1
    if around == "y":
        idx1,idx2 = 0,2
    if around == "x":
        idx1,idx2 = 1,2

    rotated_points = []
    for i in points:
        r,phi = cart2pol(i[idx1]-center[idx1],i[idx2]-center[idx2])
        if clockwise:
            x,y = pol12cart(r,phi-rotation_angle)
        else:
            x,y = pol12cart(r,phi+rotation_angle)
        x+=center[idx1]
        y+=center[idx2]
        if around == "z":
            rotated_points.append([x,y,i[2]])
        if around == "y":
            rotated_points.append([x,i[1],y])
        if around == "x":
            rotated_points.append([i[0],x,y])
    if len(rotated_points)==1:
        rotated_points = rotated_points[0]

    return rotated_points

##########################     TRAJECTORY GENERATION UTILS   ###############################################
def normalize(v):
  norm = np.linalg.norm(v)
  assert norm > 0
  return v / norm


def polyder(t, k = 0, order = 10):
    if k == 'all':
        terms = np.array([polyder(t,k,order) for k in range(1,5)])
    else:
        terms = np.zeros(order)
        coeffs = np.polyder([1]*order,k)[::-1]
        pows = t**np.arange(0,order-k,1)
        terms[k:] = coeffs*pows
    return terms

def Hessian(T,order = 10,opt = 4):
    n = len(T)
    Q = np.zeros((order*n,order*n))
    for k in range(n):
        m = np.arange(0,opt,1)
        for i in range(order):
            for j in range(order):
                if i >= opt and j >= opt:
                    pow = i+j-2*opt+1
                    Q[order*k+i,order*k+j] = 2*np.prod((i-m)*(j-m))*T[k]**pow/pow
    return Q

def Circle_waypoints(n,Tmax = 2*np.pi):
    t = np.linspace(0,Tmax, n)
    x = 1+0.5*np.cos(t)
    y = 1+0.5*np.sin(t)
    z = 1+0*t
    return np.stack((x, y, z), axis=-1)

def Helix_waypoints(n,Tmax = 2*np.pi):

    t = np.linspace(0, Tmax, n)
    x = 1+0.5*np.cos(t)
    y = 1+0.5*np.sin(t)
    z = t/Tmax*2

    return np.stack((x, y, z), axis=-1)


def position_to_numpy(list):
    output_list = []

    for element in list:
        arr = [element.x,element.y,element.z]
        output_list.append(arr)
    return np.array(output_list)

############################################### END OF TRAJECTORY UTILS  ###############################################
