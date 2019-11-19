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
import matplotlib.patches as mpatches;

# Define key parameters. All length parameters / coordinates in mm.
RHS_Upr_OB_pickup = [528.53,355]; #RHS Upper OB pickup point
RHS_Lwr_OB_pickup = [564.85,140]; #RHS Lower OB pickup point
LHS_Upr_OB_pickup = [-RHS_Upr_OB_pickup[0],RHS_Upr_OB_pickup[1]]; #LHS Upper OB pickup point
LHS_Lwr_OB_pickup = [-RHS_Lwr_OB_pickup[0],RHS_Lwr_OB_pickup[1]]; #LHS Lower OB pickup point
static_camber = -1.5; #In degrees
RHS_Upr_IB_pickup = [305, 295]; #RHS Upper IB pickup point
RHS_Lwr_IB_pickup = [212.71, 113.52]; #RHS Lower IB pickup point
LHS_Upr_IB_pickup = [0,0]; #Create empty coordinate set for LHS Upr IB pickup point.
LHS_Lwr_IB_pickup = [0,0]; #Create empty coordinate set for LHS Lwr IB pickup point.
Track = 1200; #Total track width
Sprung_COG = [0,276.3666667]; #Static sprung mass COG. This will move as the car rolls in the simulation.
Roll_centre = [0,0] #This will not be used at the given value, but will be calculated later.
Roll_step = 0.001; #Add a small amount of roll with each iteration. The smaller this is, the more accurate it will be but will take longer to run.
RHS_scrub_rad = 0; # This will be calculated later so the value is largely irrelevant.
LHS_scrub_rad = 0; # This will be calculated later so the value is largely irrelevant.
#Tyre_rad = 256.5; #Tyre radius in mm, will be used to find contact patch centre. Current tyre is 256.5 mm rad.


Damper_IB = [0,0];
Damper_OB = [0,0];

Damper_length = math.sqrt((Damper_IB[0] - Damper_OB[0])**2 + (Damper_IB[1] - Damper_OB[1])**2)

dv_max = 30 #maximum tyre displacement in mm


"""Artificially added"""
UWB_length = math.sqrt((RHS_Upr_OB_pickup[0] - RHS_Upr_IB_pickup[0])**2 + (RHS_Upr_OB_pickup[1] - RHS_Upr_IB_pickup[1])**2);
LWB_length = math.sqrt((RHS_Lwr_OB_pickup[0] - RHS_Lwr_IB_pickup[0])**2 + (RHS_Lwr_OB_pickup[1] - RHS_Lwr_IB_pickup[1])**2);
UWB_angle = (math.atan((RHS_Upr_OB_pickup[1] - RHS_Upr_IB_pickup[1])/(RHS_Upr_OB_pickup[0] - RHS_Upr_IB_pickup[0])))*180/math.pi;
LWB_angle = (math.atan((RHS_Lwr_OB_pickup[1] - RHS_Lwr_IB_pickup[1])/(RHS_Lwr_OB_pickup[0] - RHS_Lwr_IB_pickup[0])))*180/math.pi;

LHS_Upr_IB_pickup[0] = - RHS_Upr_IB_pickup[0];
LHS_Upr_IB_pickup[1] = RHS_Upr_IB_pickup[1];
LHS_Lwr_IB_pickup[0] = - RHS_Lwr_IB_pickup[0];
LHS_Lwr_IB_pickup[1] = RHS_Lwr_IB_pickup[1];


print(RHS_Upr_IB_pickup);
print(RHS_Lwr_IB_pickup);

#Function that finds the instantaneous centre. Abstraction allows for calling this function over and over again.(Keeps code clean)
def Find_IC(Upr_OB,Upr_IB,Lwr_OB,Lwr_IB):
    # Find Gradients and appropriate constant to turn pickup points into lines in cartesian space.
    m1 = (Upr_OB[1]-Upr_IB[1])/(Upr_OB[0]-Upr_IB[0]);
    m2 = (Lwr_OB[1]-Lwr_IB[1])/(Lwr_OB[0]-Lwr_IB[0]);
    c1 = Upr_OB[1] - (m1*Upr_OB[0]);
    c2 = Lwr_OB[1] - (m2*Lwr_OB[0]);
    
    #Find  where these lines interest to find the IC.
    X_ic = (1/(m1-m2))*(c2-c1);
    Y_ic = (1/(m1-m2))*((m1*c2) - (m2*c1));
    
    IC = [X_ic,Y_ic];
    
    return IC

#Function that finds the roll centre from the track and instantaneous centres on each side of the car.
def Find_RC(IC_RHS,IC_LHS,Track):
    # Set the x coordinate of the contact patch to be half of the track.
    Contact_patch_RHS = [Track/2,0];
    Contact_patch_LHS = [-Track/2,0];
    """contact patch position needs to be modified to account for camber"""
    # Find Gradients and appropriate constant to find the lines between contact patch and IC on each side.
    m1 = (IC_RHS[1]-Contact_patch_RHS[1])/(IC_RHS[0]-Contact_patch_RHS[0]);
    m2 = (IC_LHS[1]-Contact_patch_LHS[1])/(IC_LHS[0]-Contact_patch_LHS[0]);
    c1 = IC_RHS[1] - (m1*IC_RHS[0]);
    c2 = IC_LHS[1] - (m2*IC_LHS[0]);

    #Find  where these lines interest to find the RC.
    X_rc = (1/(m1-m2))*(c2-c1);
    Y_rc = (1/(m1-m2))*((m1*c2) - (m2*c1));
    
    RC = [X_rc,Y_rc];
    
    return RC

#Function to find the scrub radius, given the OB pickup points and track of the car.
def Find_scrub_rad(Upr_OB,Lwr_OB,Track):
    
    #Define a line in cartesian space for the front view of the kingpin axis
    if Upr_OB[0]-Lwr_OB[0] != 0:
        m = (Upr_OB[1]-Lwr_OB[1])/(Upr_OB[0]-Lwr_OB[0]);
        c = Upr_OB[1] - (m*Upr_OB[0]);
    
        # Find where this intersects the ground plane
        x0 = -c/m;
    else:
        x0 = Upr_OB[0]
    # Calculate the scrub radius
    scrub = (Track/2) - abs(x0);
    
    return scrub

#Function that finds the kingpin angle, given the OB pickup points.
def Find_kingpin_angle(Upr_OB_point,Lwr_OB_point):
    
    # Find the x and y distances between the OB points. Using trigonoetry, this gives us the kingpin angle.
    dx = -(abs(Lwr_OB_point[0]) - abs(Upr_OB_point[0]));
    dy = Upr_OB_point[1] - Lwr_OB_point[1];
    
    # The kingpin angle is the arctan(dx/dy)
    if dy != 0: 
        angle = math.atan(dx/dy);
    else:
        angle = 0
    angle = angle * 180 / math.pi
    
    return angle

#Function that finds the camber, given the kingpin angle and constant difference in kingpin angle and camber.
def Find_camber(camber_to_kingpin,kingpin):
    
    # We can calculate kingpin from OB points and camber to kingpin is fixed so we can find camber.
    camber = camber_to_kingpin + kingpin;
    
    return camber

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
    

def Find_OB_damper(OB_point,Track,IB_point,WB_rad):
    
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

# Create lists to hold the roll centre coordinates, roll angles, scrub radii and wheel camber
Roll_centres = [];

# Output the baseline roll centre
RHS_IC = Find_IC(RHS_Upr_OB_pickup,RHS_Upr_IB_pickup,RHS_Lwr_OB_pickup,RHS_Lwr_IB_pickup);
LHS_IC = Find_IC(LHS_Upr_OB_pickup,LHS_Upr_IB_pickup,LHS_Lwr_OB_pickup,LHS_Lwr_IB_pickup);
Roll_centre = Find_RC(RHS_IC, LHS_IC, Track);
print("Baseline roll centre height is: " + str(round(Roll_centre[1],2)) + " mm.");


"""This is the bump/rebound section of the script"""

# Create lists to hold the roll centre coordinates, roll angles, scrub radii and wheel camber
Roll_centres = [];
COG_migration = [];
RHS_Scrub_radii = [];
LHS_Scrub_radii = [];
LHS_cambers = [];
RHS_cambers = [];
RC_migration = [];

# Set initial camber to be equal to the value of static camber defined earlier.
LHS_camber = static_camber;
RHS_camber = static_camber;

# Find initial kingpin angles.
RHS_kingpin_angle = Find_kingpin_angle(RHS_Upr_OB_pickup,RHS_Lwr_OB_pickup);
LHS_kingpin_angle = Find_kingpin_angle(LHS_Upr_OB_pickup,LHS_Lwr_OB_pickup);

# The angle between camber and kingpin is fixed so this allows us to find camber.
camber_to_kingpin = RHS_camber - RHS_kingpin_angle;

"""This is the main loop of the code"""
Sweep_param = 0; # sweep parameter.
Movement_step = dv_max/1000;

#Initially, we move the geometry into the maximum Bump condition
while Sweep_param <= dv_max:
    # This iterative loop rolls the car up to the expected maximum and saves the coordinates in a list
    # of tuples that are later used to plot a graph of roll migration.
    # Find the instantaneous centres
    RHS_IC = Find_IC(RHS_Upr_OB_pickup,RHS_Upr_IB_pickup,RHS_Lwr_OB_pickup,RHS_Lwr_IB_pickup);
    LHS_IC = Find_IC(LHS_Upr_OB_pickup,LHS_Upr_IB_pickup,LHS_Lwr_OB_pickup,LHS_Lwr_IB_pickup);
    
    # Find the roll centre, scrub radius and kingpin angle.
    Roll_centre = Find_RC(RHS_IC, LHS_IC, Track);  
    
    # Rotate the COG and IB pickup points (same angle of rotation because it's a solid body)
    Sprung_COG = Move_IB_point(Sprung_COG,Movement_step);
    RHS_Upr_IB_pickup = Move_IB_point(RHS_Upr_IB_pickup,-Movement_step);
    RHS_Lwr_IB_pickup = Move_IB_point(RHS_Lwr_IB_pickup,-Movement_step);
    LHS_Upr_IB_pickup = Move_IB_point(LHS_Upr_IB_pickup,-Movement_step);
    LHS_Lwr_IB_pickup = Move_IB_point(LHS_Lwr_IB_pickup,-Movement_step);
    Damper_IB = Move_IB_point(Damper_IB,-Movement_step);
    
    # From the new IB pickup points, find the new positions of the OB points.
    RHS_Upr_OB_pickup = Find_OB(RHS_Upr_OB_pickup,Track,RHS_Upr_IB_pickup,UWB_length);
    RHS_Lwr_OB_pickup = Find_OB(RHS_Lwr_OB_pickup,Track,RHS_Lwr_IB_pickup,LWB_length);
    LHS_Upr_OB_pickup = Find_OB(LHS_Upr_OB_pickup,-Track,LHS_Upr_IB_pickup,UWB_length);
    LHS_Lwr_OB_pickup = Find_OB(LHS_Lwr_OB_pickup,-Track,LHS_Lwr_IB_pickup,LWB_length);
    Damper_OB = Find_OB_damper()
    
    Sweep_param += Movement_step;
    
# Create wishbones, uprights, connection to contact patch in max bump
RHS_Upr_WB_bump = [RHS_Upr_OB_pickup,RHS_Upr_IB_pickup];
RHS_Lwr_WB_bump = [RHS_Lwr_OB_pickup,RHS_Lwr_IB_pickup];
RHS_upright_bump = [RHS_Lwr_OB_pickup, RHS_Upr_OB_pickup]; 

RHS_wheel_bump = [RHS_Lwr_OB_pickup, [Track/2,0]]; 

#Plot max bump geometry
plt.plot(*zip(*RHS_Upr_WB_bump), marker='x', color='r');
plt.plot(*zip(*RHS_Lwr_WB_bump), marker='x', color='r');
plt.plot(*zip(*RHS_upright_bump), color='r');
plt.plot(*zip(*RHS_wheel_bump), color='r');


""" After moving the COG to its lowest expected point, we run through the whole sweep
    from maximum bump to maximum rebound."""

Sweep_param = 0; # sweep parameter.
Sweep_size = dv_max; #Define the total size of the sweep.

while Sweep_param < Sweep_size:
    # This iterative loop rolls the car up to the expected maximum and saves the coordinates in a list
    # of tuples that are later used to plot a graph of roll migration.
    # Find the instantaneous centres
    RHS_IC = Find_IC(RHS_Upr_OB_pickup,RHS_Upr_IB_pickup,RHS_Lwr_OB_pickup,RHS_Lwr_IB_pickup);
    LHS_IC = Find_IC(LHS_Upr_OB_pickup,LHS_Upr_IB_pickup,LHS_Lwr_OB_pickup,LHS_Lwr_IB_pickup);
    
    # Find the roll centre, scrub radius and kingpin angle.
    Roll_centre = Find_RC(RHS_IC, LHS_IC, Track);  
    
    # Rotate the COG and IB pickup points (same angle of rotation because it's a solid body)
    Sprung_COG = Move_IB_point(Sprung_COG,Movement_step);
    RHS_Upr_IB_pickup = Move_IB_point(RHS_Upr_IB_pickup,Movement_step);
    RHS_Lwr_IB_pickup = Move_IB_point(RHS_Lwr_IB_pickup,Movement_step);
    LHS_Upr_IB_pickup = Move_IB_point(LHS_Upr_IB_pickup,Movement_step);
    LHS_Lwr_IB_pickup = Move_IB_point(LHS_Lwr_IB_pickup,Movement_step);
    Damper_IB = Move_IB_point(Damper_IB,Movement_step);
    
    # From the new IB pickup points, find the new positions of the OB points.
    RHS_Upr_OB_pickup = Find_OB(RHS_Upr_OB_pickup,Track,RHS_Upr_IB_pickup,UWB_length);
    RHS_Lwr_OB_pickup = Find_OB(RHS_Lwr_OB_pickup,Track,RHS_Lwr_IB_pickup,LWB_length);
    LHS_Upr_OB_pickup = Find_OB(LHS_Upr_OB_pickup,-Track,LHS_Upr_IB_pickup,UWB_length);
    LHS_Lwr_OB_pickup = Find_OB(LHS_Lwr_OB_pickup,-Track,LHS_Lwr_IB_pickup,LWB_length);
    
    Sweep_param += Movement_step;

# Append final values to the appropriate lists in order to plot graphs later
Roll_centres.append((Roll_centre[0],Roll_centre[1]));

# Create wishbones, uprights, connection to contact patch
RHS_Upr_WB_reb = [RHS_Upr_OB_pickup,RHS_Upr_IB_pickup];
RHS_Lwr_WB_reb = [RHS_Lwr_OB_pickup,RHS_Lwr_IB_pickup];
RHS_upright_reb = [RHS_Lwr_OB_pickup, RHS_Upr_OB_pickup];

RHS_wheel_reb = [RHS_Lwr_OB_pickup, [Track/2,0]]; 

#Plot Max rebound geometry
plt.plot(*zip(*RHS_Upr_WB_reb), marker='x', color='y');
plt.plot(*zip(*RHS_Lwr_WB_reb), marker='x', color='y');
plt.plot(*zip(*RHS_upright_reb), color='y');
plt.plot(*zip(*RHS_wheel_reb), color='y');
blue = mpatches.Patch(color='blue', label='Baseline');
red = mpatches.Patch(color='red', label='Maximum bump');
yellow = mpatches.Patch(color='yellow', label='Maximum rebound');
plt.legend(handles=[blue,red,yellow]);
plt.xlim(75,650);
plt.ylim(0,400);
plt.xlabel("x-coordinate");
plt.ylabel("y-coordinate");
plt.title("Geometry changes in bump");
plt.show();
