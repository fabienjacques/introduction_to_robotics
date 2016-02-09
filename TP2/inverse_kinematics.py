#!/usr/bin/python
from math import *

def leg_ik(P3x, P3y, P3z, L1, L2, L3):

    alpha = 20.69
    beta = 5.06

    theta1 = atan2(P3y, P3x)

    d13 = sqrt( (P3x*P3x) + (P3y*P3y) ) - L1
    d = sqrt( (d13*d13) + (P3z*P3z) )

    a = atan2(P3z, d13)
    b = alKashi(L2, d, L3)

    theta2 = a + b
    theta3 = alKashi(L2, L3, d) + pi
    
    theta1 = degrees(theta1)
    theta2 = -(degrees(theta2) + alpha)
    theta3 = degrees(theta3) + 90 - alpha - beta

    while theta1 > 180:
        theta1 -= 360

    while theta1 <= -180:
        theta1 += 360

    while theta2 > 180:
        theta2 -= 360

    while theta2 <= -180:
        theta2 += 360

    while theta3 > 180:
        theta3 -= 360

    while theta3 <= -180:
        theta3 += 360
    
    return theta1, theta2, theta3

def alKashi(a, b, c):
    return acos( ( (a*a) + (b*b) - (c*c) ) / (2*a*b))

print(leg_ik(203.23, 0.0, -14.30, 51.0, 63.7, 93.0))
