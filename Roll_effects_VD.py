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
RHS_Upr_OB_pickup = [890,200]; #RHS Upper OB pickup point
RHS_Lwr_OB_pickup = [890,150]; #RHS Lower OB pickup point
LHS_Upr_OB_pickup = [-RHS_Upr_OB_pickup[0],RHS_Upr_OB_pickup[1]]; #LHS Upper OB pickup point
LHS_Lwr_OB_pickup = [-RHS_Lwr_OB_pickup[0],RHS_Lwr_OB_pickup[1]]; #LHS Lower OB pickup point
static_camber = 0; #In degrees
UWB_length = 160; #Upper wishbone length in front view
LWB_length = 160; #Lower wishbone length in front view
UWB_angle = 0; #Upper wishbone angle to horizontal. Anti-clockwise positive
LWB_angle = 0; #Lower wishbone angle to horizontal. Anti-clockwise positive
RHS_Upr_IB_pickup = [0,0]; #Create empty coordinate set for RHS Upr IB pickup point.
RHS_Lwr_IB_pickup = [0,0]; #Create empty coordinate set for RHS Lwr IB pickup point.
LHS_Upr_IB_pickup = [0,0]; #Create empty coordinate set for LHS Upr IB pickup point.
LHS_Lwr_IB_pickup = [0,0]; #Create empty coordinate set for LHS Lwr IB pickup point.
Track = 1200; #Total track width
Sprung_COG = [0,276.3666667]; #Static sprung mass COG. This will move as the car rolls in the simulation.
Roll_centre = [0,0] #This will not be used at the given value, but will be calculated later.
Roll_step = 0.05; #Add a small amount of roll with each iteration. The smaller this is, the more accurate it will be but will take longer to run.
RHS_scrub_rad = 0; # This will be calculated later so the value is largely irrelevant.
LHS_scrub_rad = 0; # This will be calculated later so the value is largely irrelevant.

# Define the maximum amount of expected roll and the program will iterate up to this.
Applied_roll = 2; #Roll artificially applied to the car
Roll = 0; #Initial roll

# Convert degrees to radians for math library.
static_camber = static_camber*math.pi/180;
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

#Function that finds the roll centre. Abstraction allows for calling this function over and over again.(Keeps code clean)
def Find_IC(Upr_OB,Upr_IB,Lwr_OB,Lwr_IB):
    # Find Gradients and appropriate constant to turn pickup points into lines in cartesian space.
    m1 = (Upr_OB[1]-Upr_IB[1])/(Upr_OB[0]-Upr_IB[0]);
    m2 = (Lwr_OB[1]-Lwr_IB[1])/(Lwr_OB[0]-Lwr_IB[0]);
    c1 = Upr_OB[1] - (m1*Upr_OB[0]);
    c2 = Lwr_OB[1] - (m2*Lwr_OB[0]);
    
    #Find  where these lines interest to find the IC.
    X_ic = (1/(m1+m2))*((m2*c1)-(m1*c2));
    Y_ic = (1/(m1+m2))*(c1-c2);
    
    IC = [X_ic,Y_ic];
    return IC

def Find_RC(IC_RHS,IC_LHS,Track):
    # Set the x coordinate of the contact patch to be half of the track.
    Contact_patch_RHS = [Track/2,0];
    Contact_patch_LHS = [-Track/2,0];
    
    # Find Gradients and appropriate constant to find the lines between contact patch and IC on each side.
    m1 = (IC_RHS[1]-Contact_patch_RHS[1])/(IC_RHS[0]-Contact_patch_RHS[0]);
    m2 = (IC_LHS[1]-Contact_patch_LHS[1])/(IC_LHS[0]-Contact_patch_LHS[0]);
    c1 = IC_RHS[1] - (m1*IC_RHS[0]);
    c2 = IC_LHS[1] - (m2*IC_LHS[0]);

    #Find  where these lines interest to find the RC.
    X_rc = (1/(m1+m2))*((m2*c1)-(m1*c2));
    Y_rc = (1/(m1+m2))*(c1-c2);
    
    RC = [X_rc,Y_rc];
    return RC

def Find_scrub_rad(Upr_OB,Lwr_OB,Track):
    
    #Define a line in cartesian space for the front view of the kingpin axis
    m = (Upr_OB[1]-Lwr_OB[1])/(Upr_OB[0]-Lwr_OB[0]);
    c = Upr_OB[1] - (m*Upr_OB[0]);
    
    # Find where this intersects the ground plane
    x0 = -c/m;
    
    # Calculate the scrub radius
    scrub = (Track/2) - x0;
    
    return scrub

def Rotate_IB_point(COG,Roll_Centre,step):
    # Find the position of the COG relative to the roll centre in polar coordinates.
    r = math.sqrt(((COG[0] - Roll_Centre[0])**2) + ((COG[1] - Roll_Centre[1])**2));
    theta = math.atan((COG[1] - Roll_Centre[1])/(COG[0] - Roll_Centre[0]));
    # Rotate by the specified step (the variable passed into the function should be Roll_step, as defined above)
    step = step*math.pi/180 # convert the step to radians
    theta -= step;
    
    # Convert back to cartesian coordinates adding the polar to the roll centre, which was previously the "origin"
    COG[0] = (r * math.cos(theta)) + Roll_Centre[0];
    COG[1] = (r * math.sin(theta)) + Roll_Centre[1];
    
    return COG

# Create lists to hold the roll centre coordinates, roll angles, scrub radii and wheel camber
Roll_centres = [];
Roll_angles = [];
RHS_Scrub_radii = [];
LHS_Scrub_radii = [];
LHS_camber = [];
RHS_camber = [];

while Roll <= Applied_roll:
    # This iterative loop rolls the car up to the expected maximum and saves the coordinates in a list
    # of tuples that are later used to plot a graph of roll migration.
    
    # Find the instantaneous centres
    RHS_IC = Find_IC(RHS_Upr_OB_pickup,RHS_Upr_IB_pickup,RHS_Lwr_OB_pickup,RHS_Lwr_IB_pickup);
    LHS_IC = Find_IC((LHS_Upr_OB_pickup,LHS_Upr_IB_pickup,LHS_Lwr_OB_pickup,LHS_Lwr_IB_pickup));
    
    # Find the roll centre and scrub radius
    Roll_centre = Find_RC(RHS_IC, LHS_IC, Track);
    RHS_scrub_rad = Find_scrub_rad(RHS_Upr_OB_pickup,RHS_Lwr_OB_pickup,Track);
    LHS_scrub_rad = Find_scrub_rad(LHS_Upr_OB_pickup,LHS_Lwr_OB_pickup,Track);
    
    
    "Note to self: work out camber and make a function for this"
    
    # Append current values to the appropriate lists in order to plot graphs later
    Roll_centres.append((Roll_centre[0],Roll_centre[1]));
    Roll_angles.append(Roll);
    RHS_Scrub_radii.append(RHS_scrub_rad);
    LHS_Scrub_radii.append(LHS_scrub_rad);
    "Add appending to list for camber here"
    
    # Rotate the COG and IB pickup points (same angle of rotation because it's a solid body)
    Sprung_COG = Rotate_IB_point(Sprung_COG,Roll_centre,Roll_step);
    RHS_Upr_IB_pickup = Rotate_IB_point(RHS_Upr_IB_pickup,Roll_centre,Roll_step);
    RHS_Lwr_IB_pickup = Rotate_IB_point(RHS_Lwr_IB_pickup,Roll_centre,Roll_step);
    LHS_Upr_IB_pickup = Rotate_IB_point(LHS_Upr_IB_pickup,Roll_centre,Roll_step);
    LHS_Lwr_IB_pickup = Rotate_IB_point(LHS_Lwr_IB_pickup,Roll_centre,Roll_step);
    
    """Find OB points by making a circle (radius = wishbone length) for each IB point. The points where the
    distance bewteen them is equal to the distance between OB points initially gives the solution, as they
    are rigidly connected via the upright.
    
    Camber change can be calculated from this"""
    
    
    Roll += Roll_step;
    
   #Plot the appropriate graphs 