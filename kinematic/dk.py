import itertools
import time
import numpy
import pypot.dynamixel
import math

constL1 = 51
constL2 = 63.7
constL3 = 93
# Angle to match the theory with reality for theta 2 (measures of the triangle are 22.5, 60.7, 63.7). => Angle =  -20.69
theta2Correction = -20.69
# Same goes for theta 3 : +90 - 20.69 - a. Where a = asin(8.2/93) = 5.06
theta3Correction = 90 + theta2Correction - 5.06


# Computes the direct kinematics of a leg in the leg's frame
# Given the angles (theta1, theta2, theta3) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the destination point (x, y, z)
def computeDK(theta1, theta2, theta3, l1=constL1, l2=constL2,l3=constL3) :
    theta1 = theta1 * math.pi / 180.0
    theta2 = (theta2 - theta2Correction) * math.pi / 180.0
    theta3 = -(theta3 - theta3Correction) * math.pi / 180.0

    planContribution = l1 + l2*math.cos(theta2) + l3*math.cos(theta2 + theta3)

    x = math.cos(theta1) * planContribution
    y = math.sin(theta1) * planContribution
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3))

    return [x, y, z]