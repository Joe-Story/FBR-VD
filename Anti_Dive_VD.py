# -*- coding: utf-8 -*-
"""
Created on Thu Aug 22 21:40:04 2019

@author: Joseph Story

This script is the property of Joseph Story and it is intended solely for use by Full Blue Racing.
If you would like to request permission to use this intellectual property, please contact joe.story@fullblueracing.co.uk
"""

# This script will allow you to approximate the percentage of anti-dive of the front
# suspension
# This model is based on the side view only.

import math #Import math library;

#Define key parameters
Contact_Patch = [0,0] #Contact patch is the origin of the x, y coordinate system
COM = [700,500]
LHS_Lwr_Front_IB = [-300,400]
LHS_Lwr_Angle_deg = 5
LHS_Lwr_Spread = 300
LHS_Upr_Front_IB = [-300,650]
LHS_Upr_Angle_deg = -5
LHS_Upr_Spread = 300
LHS_Upr_Mid_Perp_Offset = 75 #Offset of the mid IB perpendicular to the line between the Front and Back IBs
LHS_Upr_Mid_Para_Offset = 150 #Offset of the mid IB parallel to the line between the Front and Back IBs

#Convert all angles to radians
LHS_Lwr_Angle_rad = math.radians(LHS_Lwr_Angle_deg)
LHS_Upr_Angle_rad = math.radians(LHS_Upr_Angle_deg)

#Calculate all other points
LHS_Lwr_Back_IB = [(LHS_Lwr_Front_IB[0]+LHS_Lwr_Spread*(math.cos(LHS_Lwr_Angle_rad))),(LHS_Lwr_Front_IB[1]+LHS_Lwr_Spread*(math.sin(LHS_Lwr_Angle_rad)))]
LHS_Upr_Back_IB = [(LHS_Upr_Front_IB[0]+LHS_Upr_Spread*(math.cos(LHS_Upr_Angle_rad))),(LHS_Upr_Front_IB[1]+LHS_Upr_Spread*(math.sin(LHS_Upr_Angle_rad)))]
LHS_Upr_Mid_IB = [(LHS_Upr_Front_IB[0]+(LHS_Upr_Mid_Para_Offset*math.cos(LHS_Upr_Angle_rad))-(LHS_Upr_Mid_Perp_Offset*math.sin(LHS_Upr_Angle_rad))),(LHS_Upr_Front_IB[1]+(LHS_Upr_Mid_Para_Offset*math.sin(LHS_Upr_Angle_rad))+(LHS_Upr_Mid_Perp_Offset*math.cos(LHS_Upr_Angle_rad)))]

#Perform anti-dive calculations
m_Upr = (LHS_Upr_Back_IB[1]-LHS_Upr_Front_IB[1])/(LHS_Upr_Back_IB[0]-LHS_Upr_Front_IB[0])
m_Lwr = (LHS_Lwr_Back_IB[1]-LHS_Lwr_Front_IB[1])/(LHS_Lwr_Back_IB[0]-LHS_Lwr_Front_IB[0])
c_Upr = LHS_Upr_Mid_IB[1]-(m_Upr*LHS_Upr_Mid_IB[0])
c_Lwr = LHS_Lwr_Front_IB[1]-(m_Lwr*LHS_Lwr_Front_IB[0])

intersect = [(1/(m_Upr-m_Lwr))*(c_Lwr-c_Upr),(1/(m_Upr-m_Lwr))*((m_Upr*c_Lwr) - (m_Lwr*c_Upr))]

m_resultant = (intersect[1]/intersect[0])

perc_anti_dive = ((m_resultant*COM[0])/COM[1])*100

print ("The percentage of anti-dive is:", perc_anti_dive, "%")