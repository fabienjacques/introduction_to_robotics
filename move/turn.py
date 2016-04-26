import itertools
import time
import numpy
import pypot.dynamixel
import math

from kinematic.ik import computeIK
from kinematic.dk import computeDK

def moveUp135(a, z, p, coords, dxl_io):
    ids1 = [11, 12, 13]
    ids2 = [21, 22, 23]
    ids3 = [31, 32, 33]
    ids4 = [41, 42, 43]
    ids5 = [51, 52, 53]
    ids6 = [61, 62, 63]

    currentTime = time.time()
    beginingTime = currentTime

    x = (1 - math.cos(a))*165
    y = - math.sin(a)*165

    while (currentTime - beginingTime < p):

        coords1 = [coords[0][0]+x*(currentTime-beginingTime)*(1/p), coords[0][1]+y*(currentTime-beginingTime)*(1/p), coords[0][2]+math.sin((currentTime - beginingTime)*(1/p)*math.pi)*z]
        coords2 = [coords[1][0]-x*(currentTime-beginingTime)*(1/p), coords[1][1]-y*(currentTime-beginingTime)*(1/p), coords[1][2]]
        coords3 = [coords[2][0]+x*(currentTime-beginingTime)*(1/p), coords[2][1]+y*(currentTime-beginingTime)*(1/p), coords[2][2]+math.sin((currentTime - beginingTime)*(1/p)*math.pi)*z]
        coords4 = [coords[3][0]-x*(currentTime-beginingTime)*(1/p), coords[3][1]-y*(currentTime-beginingTime)*(1/p), coords[3][2]]
        coords5 = [coords[4][0]+x*(currentTime-beginingTime)*(1/p), coords[4][1]+y*(currentTime-beginingTime)*(1/p), coords[4][2]+math.sin((currentTime - beginingTime)*(1/p)*math.pi)*z]
        coords6 = [coords[5][0]-x*(currentTime-beginingTime)*(1/p), coords[5][1]-y*(currentTime-beginingTime)*(1/p), coords[5][2]]

        
        angles1 = computeIK(coords1[0], coords1[1], coords1[2])
        angles2 = computeIK(coords2[0], coords2[1], coords2[2])
        angles3 = computeIK(coords3[0], coords3[1], coords3[2])
        angles4 = computeIK(coords4[0], coords4[1], coords4[2])
        angles5 = computeIK(coords5[0], coords5[1], coords5[2])
        angles6 = computeIK(coords6[0], coords6[1], coords6[2])                
                                    
        currentTime = time.time()
                        
        pos1 = dict(zip(ids1, angles1))
        pos2 = dict(zip(ids2, angles2))
        pos3 = dict(zip(ids3, angles3))
        pos4 = dict(zip(ids4, angles4))
        pos5 = dict(zip(ids5, angles5))
        pos6 = dict(zip(ids6, angles6))

        dxl_io.set_goal_position(pos1)
        dxl_io.set_goal_position(pos2)
        dxl_io.set_goal_position(pos3)
        dxl_io.set_goal_position(pos4)
        dxl_io.set_goal_position(pos5)
        dxl_io.set_goal_position(pos6)
                        
        time.sleep(0.01)

    return [coords1, coords2, coords3, coords4, coords5, coords6]

def moveUp246(a, z, p, coords, dxl_io):
    ids1 = [11, 12, 13]
    ids2 = [21, 22, 23]
    ids3 = [31, 32, 33]
    ids4 = [41, 42, 43]
    ids5 = [51, 52, 53]
    ids6 = [61, 62, 63]

    currentTime = time.time()
    beginingTime = currentTime

    x = (1 - math.cos(a))*165
    y = - math.sin(a)*165

    while (currentTime - beginingTime < p):        

        coords1 = [coords[0][0]-x*(currentTime-beginingTime)*(1/p), coords[0][1]-y*(currentTime-beginingTime)*(1/p), coords[0][2]]
        coords2 = [coords[1][0]+x*(currentTime-beginingTime)*(1/p), coords[1][1]+y*(currentTime-beginingTime)*(1/p), coords[1][2]+math.sin((currentTime - beginingTime)*(1/p)*math.pi)*z]
        coords3 = [coords[2][0]-x*(currentTime-beginingTime)*(1/p), coords[2][1]-y*(currentTime-beginingTime)*(1/p), coords[2][2]]
        coords4 = [coords[3][0]+x*(currentTime-beginingTime)*(1/p), coords[3][1]+y*(currentTime-beginingTime)*(1/p), coords[3][2]+math.sin((currentTime - beginingTime)*(1/p)*math.pi)*z]
        coords5 = [coords[4][0]-x*(currentTime-beginingTime)*(1/p), coords[4][1]-y*(currentTime-beginingTime)*(1/p), coords[4][2]]
        coords6 = [coords[5][0]+x*(currentTime-beginingTime)*(1/p), coords[5][1]+y*(currentTime-beginingTime)*(1/p), coords[5][2]+math.sin((currentTime - beginingTime)*(1/p)*math.pi)*z]

        
        angles1 = computeIK(coords1[0], coords1[1], coords1[2])
        angles2 = computeIK(coords2[0], coords2[1], coords2[2])
        angles3 = computeIK(coords3[0], coords3[1], coords3[2])
        angles4 = computeIK(coords4[0], coords4[1], coords4[2])
        angles5 = computeIK(coords5[0], coords5[1], coords5[2])
        angles6 = computeIK(coords6[0], coords6[1], coords6[2])                
                                    
        currentTime = time.time()
                        
        pos1 = dict(zip(ids1, angles1))
        pos2 = dict(zip(ids2, angles2))
        pos3 = dict(zip(ids3, angles3))
        pos4 = dict(zip(ids4, angles4))
        pos5 = dict(zip(ids5, angles5))
        pos6 = dict(zip(ids6, angles6))

        dxl_io.set_goal_position(pos1)
        dxl_io.set_goal_position(pos2)
        dxl_io.set_goal_position(pos3)
        dxl_io.set_goal_position(pos4)
        dxl_io.set_goal_position(pos5)
        dxl_io.set_goal_position(pos6)
                        
        time.sleep(0.01)

    return [coords1, coords2, coords3, coords4, coords5, coords6]

def turn(a, z, p, coords, dxl_io):
    coords = moveUp135(a, z, p, coords, dxl_io)
    coords = moveUp246(a, z, p, coords, dxl_io)
    return coords