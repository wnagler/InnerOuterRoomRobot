# -*- coding: utf-8 -*-
"""
Created on Tue Nov 24 20:06:50 2020

Program: Pioneer 3-DX bot program doing:
    a) finding the middle of a room
    b) leaving it and mapping the outside world
    c) tracking and finding a beacon
    d) returning to the middle of the initial room and shutting down

    Name: IMAT5121_6_P3DX_Final

# simRemoteApi.start(19999)

@author: naglerw
"""

#Loading libraries
try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import numpy as np
import random

print ('Libraries loaded')

#Program Control parameters

class Robot:
    """Create a Robot object """
    def __init__(self, state="Off"):
        self.state=state
        
    def change_state(self, state):
        self.state=state
        
    def get_state(self):
        return self.state
    
    def run(self):
            if p3dx.get_state() == "Off":
                p3dx.change_state("find_middle")
                print(p3dx.get_state())
    
            if p3dx.get_state()=="find_middle":
                p3dx.change_state("find_beacon")
                print(p3dx.get_state())
    
            if p3dx.get_state()=="find_beacon":
                p3dx.change_state("go_home")
                print(p3dx.get_state())
    
            if p3dx.get_state()=="go_home":
                p3dx.change_state("test")
                print(p3dx.get_state())

## Load V-Rep Library, close existing connection and create new sim
     
sim.simxFinish(-1);   # just in case, close all opened connections         
clientID=sim.simxStart('127.0.0.1',19998,True,True,5000,5);   # Connect to CoppeliaSim

if (clientID!=1):           
    #If connection successful
    print ('Connected to remote API server')
    
    #initialise the P3DX
    p3dx = Robot()


    #Main Program logic
    if p3dx.get_state() == "Off":
        p3dx.change_state("find_middle")
    
    print(p3dx.get_state())
    
    if p3dx.get_state()=="find_middle":
        p3dx.change_state("find_beacon")
        
    print(p3dx.get_state())
    
    if p3dx.get_state()=="find_beacon":
        p3dx.change_state("go_home")
    
    print(p3dx.get_state())
    
    if p3dx.get_state()=="go_home":
        p3dx.change_state("wnd")
    
    print(p3dx.get_state())
    
    # Stop the simulation
    sim.simxFinish(clientID);
     
# End of program    
else:
    # Connection Failed
    print('Failed connecting to remote API server')



