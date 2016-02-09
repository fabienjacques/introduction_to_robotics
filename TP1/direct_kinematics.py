#!/usr/bin/python
from math import *

def leg_dk(theta1, theta2, theta3, l1, l2, l3):
    
    alpha = -20.69
    beta = 5.06
    
    theta2 = -theta2    
    theta2b = theta2
    theta2 = theta2 + alpha
    theta3 = theta3 - 90.0 + beta

    theta1 = radians(theta1)
    theta2 = radians(theta2)
    theta3 = radians(theta3)
    theta2b = radians(theta2b)

    x1 = l1*cos(theta1)
    y1= l1*sin(theta1)
    z1= 0.0

    x2=x1 + l2*cos(theta2)*cos(theta1)
    y2=y1+ l2*cos(theta2)*sin(theta1)
    z2= z1 +l2*sin(theta2)

    x3=x2 + l3*cos(theta2b+theta3)*cos(theta1)
    y3=y2 + l3*cos(theta2b+theta3)*sin(theta1)
    z3= z2 + l3*sin(theta2b+theta3)
    
    return x3,y3,z3

print(leg_dk(0.0, 0.0, 0.0, 51.0, 63.7, 93.0))
