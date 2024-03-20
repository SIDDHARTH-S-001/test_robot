from math import sin, cos

vx_robot, vy_robot, w_robot, theta = 0.0, 0.0, 0.0, 0.0 # Robot Movement Parameters
w_right, w_left = 0.0, 0.0                # Wheel Movement Parameters
r = 0.2286                            # Wheel Radius
l = 0.3048                            # Wheel Seperation
turning_radius = 0.0

vx_robot = (r/2)*cos(theta)*w_right + (r/2)*cos(theta)*w_left
vy_robot = (r/2)*sin(theta)*w_right + (r/2)*sin(theta)*w_left
w_robot  = (r/l)*w_right - (r/l)*w_left 

turning_radius = (l/2)*((w_right + w_left)/(w_right - w_left))

