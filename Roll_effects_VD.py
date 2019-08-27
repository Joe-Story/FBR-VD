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
# Rotation (roll) is clockwise in the model
# Caveat is that this is not infinitely accurate because the roll centre is dynamic but is only taken at discrete points
# Another assumption is that there is 0% cross weight (i.e. COG is on the centreline when static)
# The centre of the contact patch is assumed constant, on the wheel centreline at zero camber.

import math #Import math library;
import matplotlib.pyplot as plt; # Import matplotlib for plotting graphs

# Define key parameters. All length parameters / coordinates in mm.
RHS_Upr_OB_pickup = [538.53,351]; #RHS Upper OB pickup point
RHS_Lwr_OB_pickup = [564.85,171.5]; #RHS Lower OB pickup point
LHS_Upr_OB_pickup = [-RHS_Upr_OB_pickup[0],RHS_Upr_OB_pickup[1]]; #LHS Upper OB pickup point
LHS_Lwr_OB_pickup = [-RHS_Lwr_OB_pickup[0],RHS_Lwr_OB_pickup[1]]; #LHS Lower OB pickup point
static_camber = 0; #In degrees
UWB_length = 234.254625; #Upper wishbone length in front view
LWB_length = 418.9412262; #Lower wishbone length in front view
UWB_angle = 0.6408328324; #Upper wishbone angle to horizontal. Anti-clockwise positive
LWB_angle = -7.910191806; #Lower wishbone angle to horizontal. Anti-clockwise positive
RHS_Upr_IB_pickup = [0,0]; #Create empty coordinate set for RHS Upr IB pickup point.
RHS_Lwr_IB_pickup = [0,0]; #Create empty coordinate set for RHS Lwr IB pickup point.
LHS_Upr_IB_pickup = [0,0]; #Create empty coordinate set for LHS Upr IB pickup point.
LHS_Lwr_IB_pickup = [0,0]; #Create empty coordinate set for LHS Lwr IB pickup point.
Track = 1200; #Total track width
Sprung_COG = [0,276.3666667]; #Static sprung mass COG. This will move as the car rolls in the simulation.
Roll_centre = [0,0] #This will not be used at the given value, but will be calculated later.
Roll_step = 0.001; #Add a small amount of roll with each iteration. The smaller this is, the more accurate it will be but will take longer to run.
RHS_scrub_rad = 0; # This will be calculated later so the value is largely irrelevant.
LHS_scrub_rad = 0; # This will be calculated later so the value is largely irrelevant.

# Define the maximum amount of expected roll and the program will iterate up to this.
Applied_roll = 3; #Roll artificially applied to the car
Roll = 0; #Initial roll

# Convert degrees to radians for math library.
UWB_angle = UWB_angle*math.pi/180;
LWB_angle = LWB_angle*math.pi/180;

# Calculate IB pickup points from existing defined geometry
RHS_Upr_IB_pickup[0] = RHS_Upr_OB_pickup[0] - (UWB_length*math.cos(UWB_angle));
RHS_Upr_IB_pickup[1] = RHS_Upr_OB_pickup[1] - (UWB_length*math.sin(UWB_angle));
RHS_Lwr_IB_pickup[0] = RHS_Lwr_OB_pickup[0] - (LWB_length*math.cos(LWB_angle));
RHS_Lwr_IB_pickup[1] = RHS_Lwr_OB_pickup[1] - (LWB_length*math.sin(LWB_angle));


LHS_Upr_IB_pickup[0] = - RHS_Upr_IB_pickup[0];
LHS_Upr_IB_pickup[1] = RHS_Upr_IB_pickup[1];
LHS_Lwr_IB_pickup[0] = - RHS_Lwr_IB_pickup[0];
LHS_Lwr_IB_pickup[1] = RHS_Lwr_IB_pickup[1];

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
    """Needs Checking"""
    # Find the x and y distances between the OB points. Using trigonoetry, this gives us the kingpin angle.
    dx = abs(Lwr_OB_point[0]) - abs(Upr_OB_point[0]);
    dy = Upr_OB_point[1] - Lwr_OB_point[1];
    
    
    
    # The kingpin angle is the arctan(dx/dy)
    if dy != 0: 
        angle = math.atan(dx/dy);
    else:
        angle = math.pi/2;
    angle = angle * 180 / math.pi
    
    return angle

#Function that finds the camber, given the kingpin angle and constant difference in kingpin angle and camber.
def Find_camber(camber_to_kingpin,kingpin):
    
    # We can calculate kingpin from OB points and camber to kingpin is fixed so we can find camber.
    camber = camber_to_kingpin + kingpin;
    
    return camber
    

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
def Rotate_OB(OB_point,Track,IB_point,WB_rad):
    
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
        the simplest to code. With more time, this approach may be changed in the future. """
    
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
Roll_angles = [];
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

# Output the baseline roll centre
RHS_IC = Find_IC(RHS_Upr_OB_pickup,RHS_Upr_IB_pickup,RHS_Lwr_OB_pickup,RHS_Lwr_IB_pickup);
LHS_IC = Find_IC(LHS_Upr_OB_pickup,LHS_Upr_IB_pickup,LHS_Lwr_OB_pickup,LHS_Lwr_IB_pickup);
Roll_centre = Find_RC(RHS_IC, LHS_IC, Track);
print("Baseline roll centre height is: " + str(round(Roll_centre[1],2)) + " mm.");


"""This is the main loop of the code"""
while Roll <= Applied_roll:
    # This iterative loop rolls the car up to the expected maximum and saves the coordinates in a list
    # of tuples that are later used to plot a graph of roll migration.
    # Find the instantaneous centres
    RHS_IC = Find_IC(RHS_Upr_OB_pickup,RHS_Upr_IB_pickup,RHS_Lwr_OB_pickup,RHS_Lwr_IB_pickup);
    LHS_IC = Find_IC(LHS_Upr_OB_pickup,LHS_Upr_IB_pickup,LHS_Lwr_OB_pickup,LHS_Lwr_IB_pickup);
    
    # Find the roll centre, scrub radius and kingpin angle.
    Roll_centre = Find_RC(RHS_IC, LHS_IC, Track);
    RHS_scrub_rad = Find_scrub_rad(RHS_Upr_OB_pickup,RHS_Lwr_OB_pickup,Track);
    LHS_scrub_rad = Find_scrub_rad(LHS_Upr_OB_pickup,LHS_Lwr_OB_pickup,Track);
    RHS_kingpin_angle = Find_kingpin_angle(RHS_Upr_OB_pickup,RHS_Lwr_OB_pickup);
    LHS_kingpin_angle = Find_kingpin_angle(LHS_Upr_OB_pickup,LHS_Lwr_OB_pickup);
    
    
    #From the kingpin angles, we can find camber.
    RHS_camber = Find_camber(camber_to_kingpin,RHS_kingpin_angle);
    LHS_camber = Find_camber(camber_to_kingpin,LHS_kingpin_angle);
    
    # Append current values to the appropriate lists in order to plot graphs later
    Roll_centres.append((Roll_centre[0],Roll_centre[1]));
    RC_migration.append((Roll_centre[0] - Roll_centres[0][0],Roll_centre[1] - Roll_centres[0][1]));
    Roll_angles.append(Roll);
    RHS_Scrub_radii.append(RHS_scrub_rad);
    LHS_Scrub_radii.append(LHS_scrub_rad);
    RHS_cambers.append(RHS_camber);
    LHS_cambers.append(LHS_camber);
    
    # Rotate the COG and IB pickup points (same angle of rotation because it's a solid body)
    Sprung_COG = Rotate_IB_point(Sprung_COG,Roll_centre,Roll_step);
    RHS_Upr_IB_pickup = Rotate_IB_point(RHS_Upr_IB_pickup,Roll_centre,Roll_step);
    RHS_Lwr_IB_pickup = Rotate_IB_point(RHS_Lwr_IB_pickup,Roll_centre,Roll_step);
    LHS_Upr_IB_pickup = Rotate_IB_point(LHS_Upr_IB_pickup,Roll_centre,Roll_step);
    LHS_Lwr_IB_pickup = Rotate_IB_point(LHS_Lwr_IB_pickup,Roll_centre,Roll_step);
    
    # From the new IB pickup points, find the new positions of the OB points.
    RHS_Upr_OB_pickup = Rotate_OB(RHS_Upr_OB_pickup,Track,RHS_Upr_IB_pickup,UWB_length);
    RHS_Lwr_OB_pickup = Rotate_OB(RHS_Lwr_OB_pickup,Track,RHS_Lwr_IB_pickup,LWB_length);
    LHS_Upr_OB_pickup = Rotate_OB(LHS_Upr_OB_pickup,-Track,LHS_Upr_IB_pickup,UWB_length);
    LHS_Lwr_OB_pickup = Rotate_OB(LHS_Lwr_OB_pickup,-Track,LHS_Lwr_IB_pickup,LWB_length);
    
    Roll += Roll_step;




# Plot a graph of roll centres in absolute space;
plt.scatter(*zip(*Roll_centres));
plt.grid();
plt.xlabel("x-coordinate");
plt.ylabel("y-coordinate");
plt.title("Absolute roll centre position");
plt.show();

# Plot a graph of roll_centre migration
plt.scatter(*zip(*RC_migration));
plt.grid();
plt.xlabel("x migration");
plt.ylabel("y migration");
plt.title("Roll centre migration");
plt.show();

#Plot graphs for scrub radius vs roll
plt.scatter(Roll_angles,LHS_Scrub_radii);
plt.grid();
plt.xlabel("Applied roll angle");
plt.ylabel("LHS scrub radius");
plt.title("LHS scrub radius vs roll");
plt.show();

plt.scatter(Roll_angles,RHS_Scrub_radii);
plt.grid();
plt.xlabel("Applied roll angle");
plt.ylabel("LHS scrub radius");
plt.title("RHS scrub radius vs roll");
plt.show();

#Plot graphs for camber vs roll
plt.scatter(Roll_angles,RHS_cambers);
plt.grid();
plt.xlabel("Applied roll angle");
plt.ylabel("RHS camber angle");
plt.title("RHS camber angle vs roll");
plt.show();

plt.scatter(Roll_angles,LHS_cambers);
plt.grid();
plt.xlabel("Applied roll angle");
plt.ylabel("LHS camber angle");
plt.title("LHS camber angle vs roll");
plt.show();
    
