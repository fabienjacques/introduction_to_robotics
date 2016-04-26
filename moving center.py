import itertools
import time
import numpy
import pypot.dynamixel
import math

from kinematic.ik import computeIK
from kinematic.dk import computeDK


def move(x, y, p, coords):
    ids1 = [11, 12, 13]
    ids2 = [21, 22, 23]
    ids3 = [31, 32, 33]
    ids4 = [41, 42, 43]
    ids5 = [51, 52, 53]
    ids6 = [61, 62, 63]

    currentTime = time.time()
    beginingTime = currentTime

    while (currentTime - beginingTime < p):

        coords1 = [coords[0][0]+x*(currentTime-beginingTime)*(1/p), coords[0][1]+y*(currentTime-beginingTime)*(1/p), coords[0][2]]
        coords2 = [coords[1][0]-y*(currentTime-beginingTime)*(1/p), coords[1][1]+x*(currentTime-beginingTime)*(1/p), coords[1][2]]
        coords3 = [coords[2][0]-y*(currentTime-beginingTime)*(1/p), coords[2][1]+x*(currentTime-beginingTime)*(1/p), coords[2][2]]
        coords4 = [coords[3][0]-x*(currentTime-beginingTime)*(1/p), coords[3][1]-y*(currentTime-beginingTime)*(1/p), coords[3][2]]
        coords5 = [coords[4][0]+y*(currentTime-beginingTime)*(1/p), coords[4][1]-x*(currentTime-beginingTime)*(1/p), coords[4][2]]
        coords6 = [coords[5][0]+y*(currentTime-beginingTime)*(1/p), coords[5][1]-x*(currentTime-beginingTime)*(1/p), coords[5][2]]

        
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

def danse(coords, p):
    while(True):
        coords = move(30, 0, p, coords)
        coords = move(-30, 0, p, coords)
            
        coords = move(0, 30, p, coords)
        coords = move(0, -30, p, coords)
            
        coords = move(-30, 0, p, coords)
        coords = move(30, 0, p, coords)
            
        coords = move(0, -30, p, coords)
        coords = move(0, 30, p, coords)
            
            
        coords = move(30, 30, p, coords)
        coords = move(-30, -30, p, coords)
            
        coords = move(30, -30, p, coords)
        coords = move(-30, 30, p, coords)
            
        coords = move(-30, -30, p, coords)
        coords = move(30, 30, p, coords)
            
        coords = move(-30, 30, p, coords)
        coords = move(30, -30, p, coords)

def circle(coords, p, q, r):
    coords = move(-r, 0, p, coords)

    while(True):
        for i in range(q):
            x = math.cos( (2*math.pi*i)/q ) - math.cos( (2*math.pi*(i+1))/q )
            y = math.sin( (2*math.pi*i)/q ) - math.sin( (2*math.pi*(i+1))/q )
            coords = move(x*r, y*r, p, coords)
            print(math.cos( (2*math.pi*i)/q )*30, math.sin( (2*math.pi*i)/q )*30)


if __name__ == '__main__':

    # we first open the Dynamixel serial port
    with pypot.dynamixel.DxlIO('/dev/ttyUSB0', baudrate=1000000) as dxl_io:

        coords1 = [100, 0, -100]
        coords2 = [70, 65, -100]
        coords3 = [70, -65, -100]
        coords4 = [100, 0, -100]
        coords5 = [70, 65, -100]
        coords6 = [70, -65, -100]

        ids1 = [11, 12, 13]
        ids2 = [21, 22, 23]
        ids3 = [31, 32, 33]
        ids4 = [41, 42, 43]
        ids5 = [51, 52, 53]
        ids6 = [61, 62, 63]
        
      
        # we power on the motors
        dxl_io.enable_torque(ids1 + ids2 + ids3 + ids4 + ids5 + ids6)

        # we create a python dictionnary: {id0 : position0, id1 : position1...} 
        angles1 = computeIK(coords1[0], coords1[1], coords1[2])
        angles2 = computeIK(coords2[0], coords2[1], coords2[2])
        angles3 = computeIK(coords3[0], coords3[1], coords3[2])
        angles4 = computeIK(coords4[0], coords4[1], coords4[2])
        angles5 = computeIK(coords5[0], coords5[1], coords5[2])
        angles6 = computeIK(coords6[0], coords6[1], coords6[2])
        
        pos1 = dict(zip(ids1, angles1))
        pos2 = dict(zip(ids2, angles2))
        pos3 = dict(zip(ids3, angles3))
        pos4 = dict(zip(ids4, angles4))
        pos5 = dict(zip(ids5, angles5))
        pos6 = dict(zip(ids6, angles6))


        # we send these new positions
        dxl_io.set_goal_position(pos1)
        dxl_io.set_goal_position(pos2)
        dxl_io.set_goal_position(pos3)
        dxl_io.set_goal_position(pos4)
        dxl_io.set_goal_position(pos5)
        dxl_io.set_goal_position(pos6)
        
        time.sleep(2)  # we wait for 1s


        coords = [coords1, coords2, coords3, coords4, coords5, coords6]

        circle(coords, 0.02, 100, 75)       
            

        time.sleep(3)  # we wait for 1s       

        
    

        # we power off the motors
        dxl_io.disable_torque(ids1 + ids2 + ids3 + ids4 + ids5 + ids6)
        time.sleep(1)  # we wait for 1s
