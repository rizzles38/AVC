#!/usr/bin/python
import math

wheelbase = 33
print("Enter x1: ")
x1 = raw_input()
x1 = float(x1)
print("Enter y1: ")
y1 = raw_input()
y1 = float(y1)

radius =  (x1*x1 + y1*y1 )/ (2 * x1)
radius = -radius

steer_angle = math.atan(wheelbase/radius)
print("radius: " + str(radius) + " steering angle: " + str(steer_angle))
