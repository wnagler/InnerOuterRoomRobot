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

########################          Importing libraries           ######################
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
from math import cos
from math import sin
import math
import matplotlib.pyplot as plt
from statistics import mean

print ('Libraries loaded')

############## Simulation Parameters ###############
ALL_OBJECTIVES = ["Find Middle","Leave Room","Wall Follow","Find Beacon", "Go Home","End Program"]          #To keep track of prime directive

THRESHOLD_BEACON = 3          #Max distance that the beacon can be detected, set standard to 7
MIN_DISTANCE_BEACON=0.5       #Min distance to stany away from the Beacon (to take robot dimension into account)
MIN_DISTANCE_ROOM=0.05        #Tolerance with regards to moving to the middle of the room
MIN_FRONT_DISTANCE=0.3        #Min distance accepted with regards to forward distance to the wall
MIN_SIDE_DISTANCE=1           #Min distance accepted with regards to side distance to the wall
ANTI_COLLISION_DISTANCE=1.5   #Distance at which the anti-collision becomes active
SETPOINT = 2;                 #The desired distance from the wall the robot must follow in the wall follow algorithm
ENDTIME = 2750;               #Time the wall following logic should run
DRIVE_SPEED=3                 #Speed at which the robot drives, note that PID tuning must be done again if changed! 

###Testing turning interval in inner room
#ANGLES=[0, math.pi/16, math.pi/8, 3*math.pi/16, math.pi/4, 5*math.pi/16, 3*math.pi/8, 7*math.pi/16]
ANGLES=[0, math.pi/8, math.pi/4, 3*math.pi/8, math.pi/2, 5*math.pi/8, 3*math.pi/4, 7*math.pi/8]
#ANGLES=[0, math.pi/4, math.pi/2, 3*math.pi/4, math.pi, 5*math.pi/4, 3*math.pi/2, 7*math.pi/4]

############### BEGIN - Creating a Robot Class containing all robot functions ##############

class Robot():

    def __init__(self):
        # Internal parameters of the robot
        self.objective=ALL_OBJECTIVES[0]
        
        self.antiCollisionDistance=ANTI_COLLISION_DISTANCE         #2         #Distance at which anti-collision starts working
        self.minFrontDistance = MIN_FRONT_DISTANCE #0.2          #If the bot is this close to the wall it can no longer turn normally and risks getting stuck 
        
        self.turnKp = 3.75;                 #Proportional Gain turn PID Controller
        self.turnKi = 0.006;                #Integral Gain turn PID Controller
        self.turnKd = 6;                    #Derivative Gain turn PID Controller
        
        self.Kp = 1;                       #Proportional Gain WallFollow
        self.Ki = 0;                        #Integral Gain WallFollow
        self.Kd = 35;                      #Derivative Gain WallFollow
        
        # Setup P3DX
        res, self.robot = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking)
        
        # initialise position
        self.position=[]        #Array that will hold the x,y and Phi of the robot
        res, self.fullPosition = sim.simxGetObjectPosition(clientID, self.robot , -1 , sim.simx_opmode_blocking)
        res, self.eulerAngles = sim.simxGetObjectOrientation(clientID, self.robot, -1, sim.simx_opmode_blocking)
        self.position.append(self.fullPosition[0])
        self.position.append(self.fullPosition[1])
        self.position.append(self.eulerAngles[2])
        
        # Setup Motors
        res, self.leftMotor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)
        res, self.rightMotor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking)

        # Setup Sonars
        self.sonarHandles = []
        
        text = 'Pioneer_p3dx_ultrasonicSensor'

        for i in range(1,17):
            sonarText = text + str(i)
            res, tempHandle = sim.simxGetObjectHandle(clientID, sonarText,sim.simx_opmode_blocking)
            self.sonarHandles.append(tempHandle)

        for sonar in self.sonarHandles:
            res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,sonar,sim.simx_opmode_streaming)
        
        # Start Beacon
        res, self.distHandle = sim.simxGetDistanceHandle(clientID, 'beaconDistance', sim.simx_opmode_blocking)
        res, distance = sim.simxReadDistance(clientID, self.distHandle, sim.simx_opmode_streaming)
        res, self.beaconHandle = sim.simxGetObjectHandle(clientID, 'beacon',sim.simx_opmode_blocking)
        
    def getObjective(self):
        return self.objective
    
    def setObjective(self, objective):
        self.objective = objective      

    def getDistanceReading(self, sonarID):
        # Get distance reading from a sensor
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,self.sonarHandles[sonarID],sim.simx_opmode_buffer)

        if detectionState == 1:
            # return magnitude of detectedPoint
            return math.sqrt(sum(i**2 for i in detectedPoint))
        else:
            # return another value that we know cannot be true and handle it (use a large number so that if you do 'distance < reading' it will work)
            return 9999
     
    def getPosition(self):
        # Get the position from the robot vs the World frame (-1 value sees to that)
        res, self.fullPosition=sim.simxGetObjectPosition(clientID, self.robot , -1 , sim.simx_opmode_blocking)
        # This is getting the alignment of the robot vs the World axis (-1 value sees to that)
        # It is the 3rd or z-axis which we need to find orientation on the xy-plane.
        res, self.eulerAngles = sim.simxGetObjectOrientation(clientID, self.robot, -1, sim.simx_opmode_blocking)
        self.position[0] = self.fullPosition[0]
        self.position[1] = self.fullPosition[1]
        self.position[2] = self.eulerAngles[2]
                
        return self.position
    
    def stop(self):
        res = sim.simxSetJointTargetVelocity(clientID, self.leftMotor, 0, sim.simx_opmode_blocking)
        res = sim.simxSetJointTargetVelocity(clientID, self.rightMotor, 0, sim.simx_opmode_blocking)

    def turn(self, turnVelocity):
        # turnVelocity < 0 = turn left
        # turnVelocity > 0 = turn right
        res = sim.simxSetJointTargetVelocity(clientID, self.leftMotor, turnVelocity, sim.simx_opmode_blocking)
        res = sim.simxSetJointTargetVelocity(clientID, self.rightMotor, -turnVelocity, sim.simx_opmode_blocking)

    def turnArc(self, leftMotorVelocity, rightMotorVelocity):
        res = sim.simxSetJointTargetVelocity(clientID, self.leftMotor, leftMotorVelocity, sim.simx_opmode_blocking)
        res = sim.simxSetJointTargetVelocity(clientID, self.rightMotor, rightMotorVelocity, sim.simx_opmode_blocking)

    def move(self, velocity):
        # velocity < 0 = reverse
        # velocity > 0 = forward
        res = sim.simxSetJointTargetVelocity(clientID, self.leftMotor, velocity, sim.simx_opmode_blocking)
        res = sim.simxSetJointTargetVelocity(clientID, self.rightMotor, velocity, sim.simx_opmode_blocking)
        
    def turnToDirection(self, targetAngle):
        # method to turn the robot to a specific direction/heading based on PID
        # Input: direction / angle in rad to turn to
        # PID
        error = adjustAngle(self.getPosition()[2] - targetAngle);  #Current Error
        prevError = 0;              #Previous Error
        intError = 0;               #Cumulative Error (eg. Integral)
        
        while abs(error) > math.pi/180:   #Correct turn as long as the difference is larger than 1/2 degree
            # Update the cumulative error
            intError = intError+error;
            # Calculate the derivative  
            derivative = error - prevError;
            # Update the previous Error
            prevError=error;
            # Calculate control function
            u_t = self.turnKp*error + self.turnKi*intError + self.turnKd*derivative;
           
            self.turnArc(u_t, -u_t)

            # Calculate the current error
            error = adjustAngle(self.getPosition()[2] - targetAngle);
       
    def findDirection(self, x, y):
        #Method to find the angle between the current orientation of the robot and the target object
        #Input: target coordinates x and y
        #Output: angle in rad
        
        #getting the current position and orientation
        xCurrent, yCurrent, phiCurrent=self.getPosition()
        
        #Calculating difference vector
        xDiff=x-xCurrent
        yDiff=y-yCurrent
        
        #Calculating target angle taking into account symmetry of tan-1
        if xDiff < 0 : 
            phi=math.atan(yDiff/xDiff)-math.pi
        else:
            phi=math.atan(yDiff/xDiff)
        
        #Return direction
        return phi

    def findBeacon(self, thresholdbeacon=THRESHOLD_BEACON):
        #Input: max distance that the beacon can be detected, set standard to 7
        #Output: distance (float) and position of the beacon (array of 3 values)
        
        # Maxiumum Distance the robot can sense the beacon
        threshold = thresholdbeacon
        
        #Get the distance to the beacon
        res, distance = sim.simxReadDistance(clientID, self.distHandle, sim.simx_opmode_buffer)
        # Get Beacon Position in x,y,z cube relative to world
        res, beaconPosition = sim.simxGetObjectPosition(clientID, self.beaconHandle, -1 , sim.simx_opmode_blocking)
        
        # return distance and position if beacon is in range
        if distance >= threshold:
            return -1, [], threshold
        else:
            return distance, beaconPosition, threshold
        
    def findMiddleOfTheRoom(self):
        #Method for finding the middle of the room
        # Middle of room is (0.05,0.1) and supplied in the method in this implementation
        #Output: (x,y) coordinates of the middle of the room
        x=0.05
        y=0.1
        return (x, y)
    
    def goTo(self, x, y, tolerance):
        #method for going to a specific location
        #Input:  x,y coordinates of target
        #Output: Distance to object
        
        self.getPosition()
       
        while distance(x,y,self.position[0],self.position[1]) > tolerance:
                        
            #Determine the direction (x,y) lies and turn to it
            self.turnToDirection(self.findDirection(x, y))
            
 
            #ping the sonars
            sensor=self.senseAll()
            
            #if obstacle in front => avoid
            if (sensor[3] < ANTI_COLLISION_DISTANCE or sensor[4] < ANTI_COLLISION_DISTANCE) and (self.findBeacon()[0] > tolerance or self.findBeacon()[0] < 0):
                        
                while (sensor[3] < ANTI_COLLISION_DISTANCE or sensor[4] < ANTI_COLLISION_DISTANCE or 
                       sensor[2] < ANTI_COLLISION_DISTANCE or sensor[5] < ANTI_COLLISION_DISTANCE) and (self.findBeacon()[0] > tolerance or self.findBeacon()[0] < 0):
                    vL,vR=p3dx.drive(DRIVE_SPEED,DRIVE_SPEED)
                    p3dx.turnArc(vL,vR)
                    sensor=self.senseAll()
                    
                    
                while (sensor[8] < ANTI_COLLISION_DISTANCE or sensor[15] < ANTI_COLLISION_DISTANCE or 
                      sensor[1] < ANTI_COLLISION_DISTANCE or sensor[6] < ANTI_COLLISION_DISTANCE) :
                    self.move(DRIVE_SPEED)
                    sensor=self.senseAll()
                     
                       
            #otherwise move forward
            self.move(2)
  
   
    def senseAll(self):
        #Method for pinging all sensors and returning the distances in a vector
        #Output: list of 16 distances
        sensor_readings=[]
        
        for i in range(0, 16): sensor_readings.append(p3dx.getDistanceReading(i))
                
        return sensor_readings
    
    def drive(self,vL,vR):
        #Method with receives the proposed wheel speeds but adjusts for obstacles
        listReturnCodes=[]
        listDetectionStates=[]
        listDistances=[]
        detect=[]
    
        braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6]
        braitenbergR=[-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2]
    
        for i in range(0,8): 
            [returnCode,detectionState,distance,d1,d2]=sim.simxReadProximitySensor(clientID, self.sonarHandles[i] ,sim.simx_opmode_buffer)
            listReturnCodes.append(returnCode)
            listDetectionStates.append(detectionState)
            listDistances.append(distance)
            if detectionState==1 and np.linalg.norm(distance) < self.antiCollisionDistance:
                if np.linalg.norm(distance) < self.minFrontDistance:
                    distance = self.minFrontDistance
                detect.append(1-((np.linalg.norm(distance)-self.minFrontDistance)/(self.antiCollisionDistance-self.minFrontDistance)))
            else:    
                detect.append(0)

        vLeft=vL
        vRight=vR
    
        for i in range(0,8):
            vLeft=vLeft+braitenbergL[i]*detect[i]
            vRight=vRight+braitenbergR[i]*detect[i]
    
        return vLeft,vRight
    
 ############### END - Creating a Robot Class containing all robot functions ##############
 
 ###############        BEGIN - Creating a Robot2 Class from Robot           ############## 

class Robot2(Robot):

    def scan(self):
        #Method for getting the coordinates of a cloud of detected points in absolute/world reference
        
        xy=[]
                
        for sonar in self.sonarHandles[0:16]:       #For all sonars do
            
            # Ping the sonar
            res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,sonar,sim.simx_opmode_buffer)
            
            # If a point is detected:
            if detectionState==1:
            
                # Get the sensor orientation in Euler Angles for the sensor
                res, eulerAngle = sim.simxGetObjectOrientation(clientID, sonar, -1, sim.simx_opmode_blocking)
            
                # Get the position for the sonar vs the World frame (-1 value sees to that)
                res, position=sim.simxGetObjectPosition(clientID, sonar , -1 , sim.simx_opmode_blocking)
            
                # Rotate the coordinates of the found point to receive absolute coordinates (minus the offset of the sensor location)
                v1=rotate(detectedPoint[0],detectedPoint[1],detectedPoint[2],eulerAngle[0],eulerAngle[1],eulerAngle[2])
                
                # Append the x and y coordinates of the detected poing plus the location of the sensor to get absolute coordinates
                xy.append((position[0]+v1[0],position[1]+v1[1]))
                
        return xy
    
    def scanAround(self):
        #function returns list of coordinates detected around the robot when turning around its axis in pi/8 degree increments
        xy=[]
        for angle in ANGLES:
            self.turnToDirection(angle)
            xy+=self.scan()
        
        return xy
    
    def wallFollow(self):
        
        error = 0;                  #Current Error
        prevError = 0;              #Previous Error
        intError = 0;               #Cumulative Error (eg. Integral)
        
        for i in range(ENDTIME) :
        
            # Scan Around every 30 ticks
            if i%30==0:
                # Stop is required as otherwise the last driving instruction operates too long
                self.stop()
                map.addPoints(self.scan())
            
            # Ping sonars
            distanceFront=self.getDistanceReading(4);
            distanceRight=self.getDistanceReading(8);
        
            # Setting the Front Flag
            if (distanceFront <= SETPOINT):  # Something in Front => left turn
                frontFlag = True;
            else:
                frontFlag = False;  # Nothing in Front => Continue
            
            # Setting Right Flag
            if distanceRight == 9999 :
                rightFlag = False;      # Nothing was detected on the right => turn right
            else :
                rightFlag = True;       # It means something was detected on the right => follow wall
        
            if frontFlag :
                if distanceFront<MIN_FRONT_DISTANCE :
                #   Robot stuck, back out randomly
                    self.turnArc(-random.random(), -random.random())    
                    time.sleep(0.1);
                else:    
                    #turn left inplace
                    self.turnArc(0,DRIVE_SPEED)
            
            elif rightFlag:
                # PID
                # Calculate the current error
                error = distanceRight - SETPOINT;
                # Update the cumulative error
                intError = intError+error;
                # Calculate the derivative
                derivative = error - prevError;
                # Update the previous Error
                prevError=error;
                # Calculate control function
                u_t = self.Kp*error + self.Ki*intError + self.Kd*derivative;
           
                self.turnArc(DRIVE_SPEED,DRIVE_SPEED-u_t)
               
            else:
                # turn right inplace because the wall sloped away
                self.turnArc(DRIVE_SPEED,0)

    
        
###############           END - Creating a ROBOT2 Class from Robot            ############## 
        
############### BEGIN - Creating a Map Class containing all mapping functions ##############    
class Map():

    def __init__(self):
        # Internal parameters of the map    
        self.wayPoints=[]     #list of waypoints in (x,y) coordinates
    
    def getWayPoint(self, number):
        return self.wayPoints[number]
    
    def getWayPoints(self):
        return self.wayPoints
    
    def addWayPoint(self,wayPoint):
        self.wayPoints.append(wayPoint)
    
############### END - Creating a Map Class containing all mapping functions ##############
        
############### BEGIN - Creating a Map Class containing all mapping functions ##############    
class Map2(Map):

    def __init__(self):
        # Inherit constructor from Map   
        super().__init__() 
        self.points=[]      #List which will contain all detected points
        self.map=np.zeros((2,2))    #Creating a dummy 2x2 matrix
        
    def addPoints(self,points):
        #points = list of tuples representing x,y coordinates
        self.points+=points
        
    def getPoints(self):
        return(self.points)
        
    def printMap(self):
        #Method to print XYscatter map of detectedpoints
        coord=list(zip(*self.getPoints()))
        
        #Building the plot
        plt.clf()
        plt.ylabel('Y-Axis')
        plt.xlabel('X-Axis')
        plt.title("Map of the Room")
        plt.scatter(coord[0],coord[1])
        plt.show()  
        
    def createMap(self, resolution):
        #Input: points is a list of (x,y) coordinates
        #Input: resolution is the size of an individual square of the map in m
        
        #splitting the points in 2 list of x and y
        coord=list(zip(*self.getPoints()))
        
        #Calculating the range and number of grid squares
        Xmax= int((max(coord[0])-min(coord[0]))//resolution+1)  #range X
        Ymax= int((max(coord[1])-min(coord[1]))//resolution+1)  #range Y
            
        self.map=np.zeros((Ymax,Xmax))       # create empty grid
         
    def updateMap(self, points,resolution):   
        #Input: points is a list of (x,y) coordinates
        #Input: resolution is the size of an individual square of the map in m
        
        #splitting the points in 2 list of x and y
        coord=list(zip(*self.getPoints()))
        
        #Calculating the range and number of grid squares
        xmin=min(coord[0])      #lower x value
        ymin=min(coord[1])      #Lower y value
        
        for point in points:
            x = int((point[0]-xmin)//resolution)
            y = int(-(point[1]-ymin)//resolution)
            self.map[y][x]=1
            
    def printGrid(self):
        plt.clf()
        plt.imshow(self.map)
        plt.show
            
        
############### END - Creating a Map Class containing all mapping functions ##############

###############                   Help Functions                             ##############

def adjustAngle(angle):
    #helper function to return angles with circels added
    while angle > math.pi : angle-=math.pi*2
    while angle <= -math.pi : angle+=math.pi*2
    return angle

def distance(x1,y1,x2,y2):
    #helper function to calculate distance between 2 points in 2 dim
    #Input: (x1,y1) coordinates point 1, (x2,y2) coordinates point 2
    return math.sqrt((x1-x2)**2+(y1-y2)**2)

def test():
    sim.simxFinish(-1)
    clientID=sim.simxStart('127.0.0.1',19998,True,True,5000,5)
    p3dx = Robot2()
    return 'P3DX spawned'


def rotationMatrix(a,b,g):
    #Returns the rotation matrix using Tait-Bryan angles alpha, beta and gamma
    rot= np.array([(cos(b)*cos(g), 0, sin(b)), 
                   (cos(g)*sin(a)*sin(b), 0, -cos(b)*sin(a)),
                   (0, cos(g)*sin(a), 0)], dtype = float)
    return rot

def rotate(x,y,z,a,b,g):
    #Input: coordinates x,y, and z in the sensors frame of reference
    #Output: vector of coordinates (x',y',z') which are the rotation x,y,z
    v1 = np.array([(x),(y),(z)])
    v2 = np.matmul(rotationMatrix(a,b,g),v1)
    return v2
    
    
###############              Main Controller Logic                          #############
############### Close existing connections and create new simulation        #############
     
sim.simxFinish(-1);   # just in case, close all opened connections         
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5);   # Connect to CoppeliaSim

if (clientID!=1):
#if (clientID!=0):           
    #If connection successful => Start simulation
    print ('Connected to remote API server')
    print()
    
    #initialise the Map
    #map = Map()
    # Modification to use second Map Class
    map=Map2()
    print('Map created')
    
    #initialise the P3DX
    p3dx = Robot2()
    print('P3DX spawned')
    print()
    print("P3DX - current objective : ",p3dx.getObjective())
    print("   P3DX - location : ",p3dx.getPosition())
   
    
    #Main Program logic
    while p3dx.getObjective() != "End Program":               #While objective is not "End Program"
  
        if p3dx.getObjective() == ALL_OBJECTIVES[0]:      #First objective= find the middle of the inner room
            middle=p3dx.findMiddleOfTheRoom()
            p3dx.goTo(middle[0], middle[1], MIN_DISTANCE_ROOM) 
            
            # MODIFICATION1 - scanning the innerroom
            map.addPoints(p3dx.scanAround())
            
            p3dx.setObjective(ALL_OBJECTIVES[1])
            #Adding first waypoint to map
            map.addWayPoint(middle)
            print("Objective 1 Completed - Middle of the room reached")
            print()
            print("P3DX - current objective : ",p3dx.getObjective())
            print("   P3DX - location : ",p3dx.getPosition())
            print("   Current map : ", map.getWayPoints())
             
            
    
        elif p3dx.getObjective() == ALL_OBJECTIVES[1]:   #Second objective=leave the inner room
            
            sensor=p3dx.senseAll()  #Sensor contains a list with readings from all sonars
            
            while sensor[3]!=9999 or sensor[4]!=9999:   #turn towards the opening
                p3dx.turn(0.5)
                sensor=p3dx.senseAll()
                
                
            #leave the room    
            while sensor[0]!=9999 or sensor[15]!=9999 or sensor[7]!=9999 or sensor[8]!=9999:
                p3dx.move(DRIVE_SPEED)
                sensor=p3dx.senseAll() 
                
                #MODIFICATION2 - scanning while driving
                map.addPoints(p3dx.scan())
                
            time.sleep(1)
            p3dx.stop
            
            #Adding second waypoint to map
            currentPosition=p3dx.getPosition()
            map.addWayPoint([currentPosition[0], currentPosition[1]])
            
            #MODIFICATION3 - scanning at the second waypoint
            map.addPoints(p3dx.scan())
            
            p3dx.setObjective(ALL_OBJECTIVES[2])
            print("Objective 2 Completed - P3DX left the room")
            print()
            print("P3DX - current objective : ",p3dx.getObjective())
            print("   P3DX - location : ",p3dx.getPosition())
            print("   Current map : ", map.getWayPoints())
 
        
        elif p3dx.getObjective() == ALL_OBJECTIVES[2]:      #Third objective= wall follow for a while so room is scanned ok
            while p3dx.getDistanceReading(4) > SETPOINT:
                p3dx.move(DRIVE_SPEED)
            
            p3dx.wallFollow()
            p3dx.setObjective(ALL_OBJECTIVES[3])
            print("Objective 3 Completed - Room Scanned")
            print()
            print("P3DX - current objective : ",p3dx.getObjective())
            print("   P3DX - location : ",p3dx.getPosition())
            print("   Current map : ", map.getWayPoints())

          
        elif p3dx.getObjective() == ALL_OBJECTIVES[3]:     #Fourth objective= wander around until beacon found
            beaconSensing = p3dx.findBeacon()
            while beaconSensing[0] == -1:                  #While no beacon is detected
                vL,vR=p3dx.drive(DRIVE_SPEED,DRIVE_SPEED)
                p3dx.turnArc(vL,vR)
                beaconSensing = p3dx.findBeacon()

                #MODIFICATION4 - scanning while driving
                map.addPoints(p3dx.scan())              
                
                
            print("   Beacon Detected")    
            #Adding third waypoint to map
            currentPosition=p3dx.getPosition()
            map.addWayPoint([currentPosition[0], currentPosition[1]])
            print("   Current map : ", map.getWayPoints())
            print()
            
            p3dx.goTo(beaconSensing[1][0], beaconSensing[1][1], MIN_DISTANCE_BEACON)    #Tolerance is the distance from the center not the edge of the robot to center beacon 
            p3dx.setObjective(ALL_OBJECTIVES[4])
            currentPosition=p3dx.getPosition()
            map.addWayPoint([currentPosition[0], currentPosition[1]])
            print("Objective 4 Completed - Beacon found")
            print()
            print("P3DX - current objective : ",p3dx.getObjective())
            print("   P3DX - location : ",p3dx.getPosition())
            print("   Current map : ", map.getWayPoints())
                      
            p3dx.stop()
        
        elif p3dx.getObjective() == ALL_OBJECTIVES[4]:   #Go back Home by navigating to the waypoints of the map
            waypoints = map.wayPoints.copy()             #Copy the waypoints   
            waypoints.pop()                              #Getting rid of last waypoint because the bot is already there

            for waypoint in range(len(map.wayPoints)-1):
                target=waypoints.pop()
                p3dx.goTo(target[0],target[1],MIN_DISTANCE_ROOM)
                print("   Waypoint reached : (",target[0],",",target[1],")")
            
            #Temp: Making sure the While loop ends
            print()
            print("Objective 5 Completed - P3DX back home") 
            p3dx.setObjective(ALL_OBJECTIVES[5])
            
        p3dx.stop()
        
        print("P3DX - current objective : ",p3dx.getObjective())
        print()
    p3dx.stop()
    
    # Stop the simulation
    #sim.simxFinish(clientID);
     
# End of program    
else:
    # Connection Failed
    print('Failed connecting to remote API server')

print('Program Ended.')