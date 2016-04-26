import pygame
import itertools
import time
import numpy
import pypot.dynamixel
import math

from kinematic.ik import computeIK
from kinematic.dk import computeDK
from move.walk import walk
from move.center import center
from move.center import dance
from move.turn import turn

with pypot.dynamixel.DxlIO('/dev/ttyUSB0', baudrate=1000000) as dxl_io:

    lastX = 0
    lastY = 0
    X = 0
    Y = 0

    #---------ROBOT---------

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

    currentTime = time.time()
    beginingTime = currentTime

    coords = [coords1, coords2, coords3, coords4, coords5, coords6]

    pygame.init()
            
    #---------GAMEPAD---------

    #Loop until the user clicks the close button.
    done = False

    # Initialize the joysticks
    pygame.joystick.init()

    while done==False:
        # EVENT PROCESSING STEP
        for event in pygame.event.get(): # User did something
            if event.type == pygame.QUIT: # If user clicked close
                done=True # Flag that we are done so we exit this loop
            

        # Get count of joysticks
        joystick_count = pygame.joystick.get_count()

        
        # For each joystick:
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
        
            # Get the name from the OS for the controller/joystick
            name = joystick.get_name()
            
            
            # Usually axis run in pairs, up/down for one, and left/right for
            # the other.
            axes = joystick.get_numaxes()
            axis = []
            for i in range( axes ):
                axis.append( joystick.get_axis( i ) )
                
            buttons = joystick.get_numbuttons()
            button = []
            for i in range(buttons):
                button.append(joystick.get_button(i))
                
            # Hat switch. All or nothing for direction, not like joysticks.
            # Value comes back in an array.
            hats = joystick.get_numhats()
            hat = []
            for i in range( hats ):
                hat.append(joystick.get_hat( i ))

    #---------MOVING---------

        #End program
        if button[9]:
            break
        #Dances, with speed 1, 2, 3 or 4
        elif button[0]:
            coords = dance(coords, 1, dxl_io)
        elif button[1]:
            coords = dance(coords, 0.5, dxl_io)
        elif button[2]:
            coords = dance(coords, 0.2, dxl_io)
        elif button[3]:
            coords = dance(coords, 0.1, dxl_io)
        #Walking
        elif axis[0] != 0 or axis[1] != 0:
            coords = walk(-15*axis[0], 15*axis[1], 10, 0.2, coords, dxl_io)
        #Moving center
        elif axis[2] != 0 or axis[4] != 0:
            X = -axis[2]
            Y = -axis[4]
            coords = center(-30*(X - lastX), 30*(Y - lastY), 0.05, coords, dxl_io)
            lastX = X
            lastY = Y
            
        elif (axis[2] == 0 and axis[4] == 0) and (lastX != 0 or lastY != 0):
            X = 0
            Y = 0
            coords = center(-30*(X - lastX), 30*(Y - lastY), 0.05, coords, dxl_io)
            lastX = X
            lastY = Y
        #Reinitialising position
        elif button[8]:
            coords1 = [100, 0, -100]
            coords2 = [70, 65, -100]
            coords3 = [70, -65, -100]
            coords4 = [100, 0, -100]
            coords5 = [70, 65, -100]
            coords6 = [70, -65, -100]

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
        #Turning    
        elif axis[3] != 0:
            coords = turn((math.pi/12) * axis[3], 20, 0.2, coords, dxl_io)
            coords = [coords1, coords2, coords3, coords4, coords5, coords6]
            


        print("\n ---------------------\n")
        time.sleep(0.01)
        
        
    # we power off the motors
    dxl_io.disable_torque(ids1 + ids2 + ids3 + ids4 + ids5 + ids6)
    time.sleep(1)  # we wait for 1s
