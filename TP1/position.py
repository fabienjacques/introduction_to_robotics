#!/usr/bin/python

#Compute the position of P1, P2 and P3 according to the angles theta1, theta2, theta3, alpha, beta and the lengths L1, L2 and L3

import math

theta1 = 1
theta2 = 1
theta3 = 1
L1 = 23
L2 = 45
L3 = 12
alpha = 2
beta = 1.5

def P1x():
    return L1 * math.cos(theta1)

def P1y():
    return L1 * math.sin(theta1)

def P1z():
    return 0

def P2x():
    return P1x() + math.cos(theta2 - alpha) * L2 * math.cos(theta2 - alpha)

def P2y():
    return P1y() + math.sin(theta1) * L2 * math.cos(theta2 - alpha)

def P2z():
    return P1z() + L2 * math.sin(theta2 - alpha)

def planContribution():
    return L1 + L2 * math.cos(theta2 - alpha) + L3 * math.cos(theta2 - alpha + theta3 - (math.pi / 2) + beta)

def P3x():
    return planContribution() * math.cos(theta1)

def P3y():
    return planContribution() * math.sin(theta1)

def P3z():
    return L2 * math.sin(theta2 - alpha) + L3 * math.sin(theta2 - alpha + theta3 - (math.pi / 2) + beta)


print("P1x = ", P1x())
print("P1y = ", P1y())
print("P1z = ", P1z())
print("P2x = ", P2x())
print("P2y = ", P2y())
print("P2z = ", P2z())
print("P3x = ", P3x())
print("P3y = ", P3y())
print("P3z = ", P3z())
