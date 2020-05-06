#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 30 20:07:37 2019
@author: ryan
This script is property of Ryan Wilkins and it is intended solely for use by Full Blue Racing.
If you would like to request permission to use this intellectual property, please contact ryan.wilkins@fullblueracing.co.uk
"""

# This script will allow you to approximate roll-centre migration, camber changes and scrub radius
# throughout a roll sweep.
# This model is based on front view only.
# Rotation (roll) is clockwise in the model.
# Caveat is that this is not infinitely accurate because the roll centre is dynamic but is only taken at discrete points
# The centre of the contact patch is assumed constant, on the wheel centreline at zero camber.
# i.e. equivalent to halving the track.

import math #Import math library;
import matplotlib.pyplot as plt; # Import matplotlib for plotting graphs
import numpy as np

# Define key parameters. All length parameters / coordinates in mm.
RHS_Upr_OB_pickup = [528.53,368]; #RHS Upper OB pickup point
RHS_Lwr_OB_pickup = [564.85,140]; #RHS Lower OB pickup point
RHS_Upr_IB_pickup = [308.4991052451646, 329.1028894753635]; #RHS Upper IB pickup point
RHS_Lwr_IB_pickup = [208.93360212033411, 127.54636728580483]; #RHS Lower IB pickup point
Track = 1200; #Total track width


Damper_IB = [189.7,569.3];
Damper_OB = [494.2,157.5];


# Define the maximum amount of expected bump/rebound and the program will iterate up to this.
Sweep_param = 30 #Initial counting parameter for the sweep is set to zero.
Movement_step = 0.1; #Add a small amount of roll with each iteration. The smaller this is, the more accurate it will be but will take longer to run.

RHS_Upr_IB_pickup = [308.4991052451646, 329.1028894753635]; #RHS Upper IB pickup point
RHS_Lwr_IB_pickup = [208.93360212033411, 127.54636728580483]; #RHS Lower IB pickup point

def Find_length(IB_point,OB_point):
    return math.sqrt((OB_point[0] - IB_point[0])**2 + (OB_point[1] - IB_point[1])**2)

def Find_OB_damper(OB_point,IB_point,d_parallel,d_perpendicular):
    OB_point = np.array([OB_point[0] , OB_point[1]])
    IB_point = np.array([IB_point[0] , IB_point[1]])
    wb_vector = OB_point - IB_point; # Make vector along wishbone
    wb_vector = wb_vector/(np.linalg.norm(wb_vector))#Normalise vector along wishbone
    parallel_vector = wb_vector * d_parallel
    P = OB_point - parallel_vector
    perp_vector = np.array([parallel_vector[1],-parallel_vector[0]]) # Make a vector perpendicular to the wb vector. This is the cross product with the k-direction
    perp_vector = perp_vector/(np.linalg.norm(perp_vector)) #Normalise vector normal to wishbone
    
    #If the vector perpendicular to the wishbone points down, flip it around. The vector points up because the damper mount is above the wishbone
    if perp_vector[1] < 0:
        perp_vector = - perp_vector
    else:
        pass
    
    Damper_point = P + (perp_vector * d_perpendicular)
    
    return [Damper_point[0], Damper_point[1]]


#Function to Move the COG and IB pickup points in bump or rebound.
def Move_IB_point(Point,step):
    #Move the point upwards by the specified step.
    Point[1] += step;
    
    return Point

#Function to rotate the COG and IB pickup points about the roll centre.
def Rotate_IB_point(Point,Roll_Centre,step):
    # Find the position of the IB point relative to the roll centre in polar coordinates.
    r = math.sqrt(((Point[0] - Roll_Centre[0])**2) + ((Point[1] - Roll_Centre[1])**2));
    
    Point_to_RC = [Point[0] - Roll_Centre[0],Point[1] - Roll_Centre[1]]
    if Point_to_RC[0] == 0 and Point_to_RC[1] > 0: 
        theta = math.pi/2;
    elif Point_to_RC[0] == 0 and Point_to_RC[1] < 0:
        theta = 1.5*math.pi;
    elif Point_to_RC == [0,0]:
        return Point
    elif Point_to_RC[0] > 0 and  Point_to_RC[1] >= 0:
        theta = math.atan(Point_to_RC[1]/Point_to_RC[0]);
    elif Point_to_RC[0] < 0 and Point_to_RC[1] >= 0:
        theta  = math.pi - math.atan(abs(Point_to_RC[1]/Point_to_RC[0]));
    elif Point_to_RC[0] < 0 and Point_to_RC[1] <= 0:
        theta  = math.atan(abs(Point_to_RC[1]/Point_to_RC[0])) - math.pi;
    elif Point_to_RC[0] > 0 and Point_to_RC[1] <= 0:
        theta  = -math.atan(abs(Point_to_RC[1]/Point_to_RC[0]));
    
    # Rotate by the specified step (the variable passed into the function should be Roll_step, as defined above)
    step = step*math.pi/180 # convert the step to radians
    theta -= step;
    
    # Convert back to cartesian coordinates adding the polar to the roll centre, which was previously the "origin"
    Point[0] = (r * math.cos(theta)) + Roll_Centre[0];
    Point[1] = (r * math.sin(theta)) + Roll_Centre[1];
    
    return Point

#Function to find the new position of an OB pickup point, given the new IB pickup points and old OB points.
def Find_OB(OB_point,Track,IB_point,WB_rad):
    
    # Find the radius of the cirlce passing through the centre of contact patch and OB point.
    r0 = math.sqrt(((OB_point[0] - Track/2)**2) + (OB_point[1]**2));

    # The radius of the second circle is equal to the length of the relevant wishbone in front view.
    r1 = WB_rad;
    
    # Define the centre points of each of these circles
    x0 = Track/2;
    y0 = 0;
    x1 = IB_point[0];
    y1 = IB_point[1];

    # As shown by hand, linearise the equations to give y in terms of x.
    # These next lines of code give the gradient and constant that define this line.
    m = (x0-x1)/(y1-y0);
    constant = ((r0**2 + x1**2 + y1**2) - (r1**2 + x0**2 + y0**2))/(2*(y1 - y0));
    
    """ Using y = mx + c and the inital equation defining one of the circles, we have a line that passes
        through the points where the circles intersect. Thus, using this sustitution will give a quadratic
        that can be solved to find intersection points. Below, we find the relevant coefficients to solve
        using the quadratic equation and this then finds both x solutions """
    
    a = 1 + m**2;
    b = (2*constant*m) - (2*x0) - (2*y0*m);
    c = (x0**2) + (constant**2) + (y0**2) - (2*y0*constant) - (r0**2);

    x_sol1 = (-b + (math.sqrt((b**2) - (4*a*c))))/(2*a);
    x_sol2 = (-b - (math.sqrt((b**2) - (4*a*c))))/(2*a);
    
    # Using y = mx + c we can find the corresponding y coordinates.
    y_sol1 = (m*x_sol1) + constant;
    y_sol2 = (m*x_sol2) + constant;
    
    """ As we are iterating for each small rotation, intuitively we would expect that the closest point to
        the previous OB point is the correct solution. Whilst not the most robust way to do this, it is
        the simplest to code and should not cause any issues. """
    
    # Start by finding the absollute distance between each solution and the old OB point.
    
    r_sol1 = math.sqrt(((x_sol1 - OB_point[0])**2) + ((y_sol1 - OB_point[1])**2));
    r_sol2 = math.sqrt(((x_sol2 - OB_point[0])**2) + ((y_sol2 - OB_point[1])**2));
    
    # Return the new OB point (i.e. the point colosest to the old one)
    
    if r_sol1 < r_sol2:
        New_OB_point = [x_sol1,y_sol1];
        
    else:
        New_OB_point = [x_sol2,y_sol2];
    
    return New_OB_point

"""Find the necessary parameters for the calcultions"""

UWB_length = Find_length(RHS_Upr_OB_pickup , RHS_Upr_IB_pickup)
LWB_length = Find_length(RHS_Lwr_OB_pickup , RHS_Lwr_IB_pickup)
damper_bsl_length = Find_length(Damper_OB , Damper_IB)

# Find point P in baseline position and use this to find parallel and perpendicular distance of damper mount to the wishbone
damper_vector = np.array([Damper_OB[0] - RHS_Lwr_OB_pickup[0], Damper_OB[1] - RHS_Lwr_OB_pickup[1]] )
wb_vector = np.array([RHS_Lwr_IB_pickup[0] - RHS_Lwr_OB_pickup[0], RHS_Lwr_IB_pickup[1] - RHS_Lwr_OB_pickup[1]])
d_parallel = np.dot(wb_vector,damper_vector)/np.dot(wb_vector,wb_vector)
P_vector = wb_vector * d_parallel
P = [RHS_Lwr_OB_pickup[0] + P_vector[0] , RHS_Lwr_OB_pickup[1] + P_vector[1]]
perpendicular_vector = [Damper_OB[0] - P[0] , Damper_OB[1] - P[1]]
d_perpendicular = math.sqrt(perpendicular_vector[0]**2 + perpendicular_vector[1]**2)
d_parallel = math.sqrt(P_vector[0]**2 + P_vector[1]**2)


#Make a list for the damper length vs displacement from baseline
damper_lengths = []

#Define counting variable
count = -Sweep_param

"""Move the chassis down by to start the bump sweep. This is the same as moving the wheel upwards"""
RHS_Upr_IB_pickup = Move_IB_point(RHS_Upr_IB_pickup,-Sweep_param);
RHS_Lwr_IB_pickup = Move_IB_point(RHS_Lwr_IB_pickup,-Sweep_param);
Damper_IB = Move_IB_point(Damper_IB , -Sweep_param);
  
# From the new IB pickup points, find the new positions of the OB points.
RHS_Upr_OB_pickup = Find_OB(RHS_Upr_OB_pickup,Track,RHS_Upr_IB_pickup,UWB_length);
RHS_Lwr_OB_pickup = Find_OB(RHS_Lwr_OB_pickup,Track,RHS_Lwr_IB_pickup,LWB_length);
Damper_OB = Find_OB_damper(RHS_Lwr_OB_pickup, RHS_Lwr_IB_pickup, d_parallel, d_perpendicular)

#Find new damper length  
damper_len = Find_length(Damper_OB , Damper_IB)

#Find damper extension and add to list
Damper_extension = damper_len - damper_bsl_length

damper_lengths.append((count,Damper_extension))

"""This is the main loop of the code"""

#Initially, we move the geometry into the maximum Bump condition
while count <= Sweep_param:

    # Rotate the COG and IB pickup points (same angle of rotation because it's a solid body)
    RHS_Upr_IB_pickup = Move_IB_point(RHS_Upr_IB_pickup , Movement_step);
    RHS_Lwr_IB_pickup = Move_IB_point(RHS_Lwr_IB_pickup , Movement_step);
    Damper_IB = Move_IB_point(Damper_IB , Movement_step);
  
    # From the new IB pickup points, find the new positions of the OB points.
    RHS_Upr_OB_pickup = Find_OB(RHS_Upr_OB_pickup , Track,RHS_Upr_IB_pickup , UWB_length);
    RHS_Lwr_OB_pickup = Find_OB(RHS_Lwr_OB_pickup , Track,RHS_Lwr_IB_pickup , LWB_length);    
    Damper_OB = Find_OB_damper(RHS_Lwr_OB_pickup, RHS_Lwr_IB_pickup, d_parallel, d_perpendicular)
    
    #Find new damper length  
    damper_len = Find_length(Damper_OB , Damper_IB)

    #Find damper extension and add to list
    Damper_extension = damper_len - damper_bsl_length
    damper_lengths.append((count,Damper_extension))

    count += Movement_step;
    
plt.plot(*zip(*damper_lengths))
plt.xlabel("Vertical Wheel Movement")
plt.ylabel("Damper Extension (Compression positive)")
print(damper_lengths[-1][1])
#Assume motion ratio is linear and find MR by taking first and last index of the list of damper lengths
motion_ratio = (damper_lengths[-1][1] - damper_lengths[0][1]) / (damper_lengths[-1][0] - damper_lengths[0][0])

print("Motion ratio is " + str(round(motion_ratio,3)))