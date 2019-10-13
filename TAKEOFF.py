#!/usr/bin/env python
# Takeoff Script - PID Controlled Flight
# Author: Pablo Hermoso Moreno & Roman Kastusik

# import libraries:
import rospy
import math
import sys
import time
import numpy as np

# ROS Message Structures

from mavros_msgs.msg import State  # import state message structure
from sensor_msgs.msg import Range  # import range message structure
from sensor_msgs.msg import LaserScan  # import LaserScan message structure
from geometry_msgs.msg import TwistStamped  # used to set velocity messages
from nav_msgs.msg import Odometry  # import range message structure
from mavros_msgs.srv import *  # import for arm and flight mode setting

from tf.transformations import euler_from_quaternion  # angle transformation


updrate = float(20)

# Initialise Parameters
range_ground = 0  # Range From Ground [m]
range_ceiling = 0  # Range From Ceiling [m]
velocity = np.array([0, 0, 0])
angpos = np.array([0, 0, 0])
position = np.array([0, 0, 0])
hover_time = 5


tookoff = 0
loopcount = 0

i=0
currentpos=[0,0,0]
previouspos=[0,0,0]
is_hovering=0
has_started_flying=0
goto1st=0
goto2nd=0
check=[0]
firstphase1run=1
counter=0
j=0


# Setpoint - This enables the drone to correct for initial deviations from the [0,0,0] in the environment setup.

setpoint = np.array([0,0,0.6])

###############################################################################

# ROS CLASSES:


class velControl:
    def __init__(self, attPub):  # attPub = attitude publisher
        self._attPub = attPub
        self._setVelMsg = TwistStamped()
        self._targetVelX = 0
        self._targetVelY = 0
        self._targetVelZ = 0

        self._targetAngVelX = 0
        self._targetAngVelY = 0
        self._targetAngVelZ = 0

    def setVel(self, coordinates):
        self._targetVelX = float(coordinates[0])
        self._targetVelY = float(coordinates[1])
        self._targetVelZ = float(coordinates[2])
        print(coordinates)

    def setAngVel(self, coordinates):
        self._targetAngVelX = float(coordinates[0])
        self._targetAngVelY = float(coordinates[1])
        self._targetAngVelZ = float(coordinates[2])

    def publishTargetPose(self, stateManagerInstance):
        self._setVelMsg.header.stamp = rospy.Time.now()  # construct message to publish with time, loop count and id
        self._setVelMsg.header.seq = stateManagerInstance.getLoopCount()
        self._setVelMsg.header.frame_id = 'fcu'

        self._setVelMsg.twist.linear.x = self._targetVelX
        self._setVelMsg.twist.linear.y = self._targetVelY
        self._setVelMsg.twist.linear.z = self._targetVelZ

        self._setVelMsg.twist.angular.x = self._targetAngVelX
        self._setVelMsg.twist.angular.y = self._targetAngVelY
        self._setVelMsg.twist.angular.z = self._targetAngVelZ

        self._attPub.publish(self._setVelMsg)


class stateManager:  # class for monitoring and changing state of the controller
    def __init__(self, rate):
        self._rate = rate
        self._loopCount = 0
        self._isConnected = 0
        self._isArmed = 0
        self._mode = None

    def incrementLoop(self):
        self._loopCount = self._loopCount + 1

    def getLoopCount(self):
        return self._loopCount

    def stateUpdate(self, msg):
        self._isConnected = msg.connected
        self._isArmed = msg.armed
        self._mode = msg.mode
        #rospy.logwarn("Connected is {}, armed is {}, mode is {} ".format(self._isConnected, self._isArmed,
                                                                        # self._mode))  # some status info

    def armRequest(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            modeService = rospy.ServiceProxy('/mavros/set_mode',
                                             mavros_msgs.srv.SetMode)  # get mode service and set to offboard control
            modeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("Service mode set faild with exception: %s" % e)

    def offboardRequest(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm = rospy.ServiceProxy('/mavros/cmd/arming',
                                     mavros_msgs.srv.CommandBool)  # get arm command service and arm
            arm(True)
        except rospy.ServiceException as e:  # except if failed
            print("Service arm failed with exception :%s" % e)

    def waitForPilotConnection(self):  # wait for connection to flight controller
        #rospy.logwarn("Waiting for pilot connection")
        while not rospy.is_shutdown():  # while not shutting down
            if self._isConnected:  # if state isConnected is true
                #rospy.logwarn("Pilot is connected")
                return True
            self._rate.sleep
        #rospy.logwarn("ROS shutdown")
        return False


###############################################################################

# CONTROL CLASSES:

class PID:
    def __init__(self,P,I,D,_integrator=0,_derivator=0,integrator_max=500,integrator_min=-500):
        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=_derivator #stores derivative of the error --> multiply this by Kd
        self.Integrator=_integrator #stores integral of the error --> multiply this by Ki
        self.set_point=0.0
        self.error=0.0 # --> multiply this by Kp
        self.velToSet=0

    def setPoint(self,set_point):
	self.set_point = set_point
	self.Integrator=0
	self.Derivator=0

    def update(self,position):
        self.error=self.set_point - position
        #set the P, I and D
        self.propTerm=self.Kp*self.error
        self.derTerm=self.Kd*(self.error-self.Derivator)
        self.Derivator=self.error #for the next loop

        self.Integrator=self.Integrator + self.error
        self.intTerm=self.Ki*self.Integrator
        self.velToSet=self.propTerm+self.intTerm+self.derTerm
        return self.velToSet

    def getSetpoint(self) :
        return self.set_point



###############################################################################


# CALLBACK FUNCTIONS:

def distanceCheck(msg):
    global range_ground  # import global range
    # print(msg.range)  # for debugging
    range_ground = msg.range  # set range = recieved range


def teraArrayCheck(msg):
    global range_tera_array

    # GAZEBO SIMULATION ENVIRONMENT
    # Understanding Message Structure of "msg.ranges"
    # Degrees : [180,135,90,45,0,315,270,225,-] - Saved as GLOBAL Array
    # IMP : Real Setup No 180deg
    range_tera_array = msg.ranges[1:7]

def teraUpCheck(msg):
    global range_ceiling
    range_ceiling = msg.ranges[0]

def teraDownCheck(msg):
    global range_ground
    range_ground = msg.ranges[0]

def odomCheck(msg):
    global position
    global angpos
    global range_ground
    angpos = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    xPosition = msg.pose.pose.position.x
    yPosition = msg.pose.pose.position.y
    zPosition = msg.pose.pose.position.z

    #zPosition = range_ground * math.cos(math.radians(angpos[1])) * math.cos(math.radians(angpos[0]))

    position = np.array([xPosition, yPosition, zPosition])

def velCheck(msg):
    global velocity
    xVel = msg.twist.linear.x
    yVel = msg.twist.linear.y
    zVel = msg.twist.linear.z
    velocity = np.array([xVel, yVel, zVel])

############################################################# FUNCTIONS RUNNING IN MAIN #####################################################################

def startup() : # Getting off the ground phase
    global goto1st
    global has_started_flying
    global xpositionControl014
    global zpositionControl014
    global yawControl
    global xVelocity
    global yVelocity
    global zVelocity
    global rollRate
    global pitchRate
    global yawRate
    global counter
    global position

    global setpoint

    if position[2]<=0.2 :
        counter=counter+1
        xpositionControl014.setPoint(setpoint[0]) #setting them within the while loop as it doesnt really matter where we do it
        zpositionControl014.setPoint(setpoint[2]) #this only changes in phase4()
        ypositionControl.setPoint(setpoint[1])
        yawControl.setPoint(0.0) #never changes
        rollControl.setPoint(0.0)
        pitchControl.setPoint(0.0)

    else :
        goto1st=1
        has_started_flying=1
    xVelocity = xpositionControl014.update(position[0]) #need to update xVelocity in each stage seperately as some use PID and others don't.
    yVelocity = ypositionControl.update(position[1])
    zVelocity = zpositionControl014.update(position[2])
    if zVelocity>0.3:
        zVelocity=0.3
    rollRate = yawControl.update(angpos[0])
    pitchRate = yawControl.update(angpos[1])
    yawRate = yawControl.update(angpos[2])

def phase1() : # HOVER at 0.6m before terminating stage.
    #global posxx
    global goto1st
    global goto2nd
    global currentime
    global prevtime
    global currentpos
    global previouspos
    global check
    global i
    global is_hovering
    global firstphase1run
    global xpositionControl014
    global xVelocity
    global zVelocity
    global yVelocity
    global rollRate
    global pitchRate
    global yawRate
    global j
    global zpositionControl014
    global stateManagerInstance

    xpositionControl014.setPoint(setpoint[0])
    zpositionControl014.setPoint(setpoint[2])
    ypositionControl.setPoint(setpoint[1])
    yawControl.setPoint(0.0) #never changes
    rollControl.setPoint(0.0)
    pitchControl.setPoint(0.0)
    #rospy.loginfo("\zsetpoint: {} \n ---".format(zpositionControl014.getSetpoint()))

    if(firstphase1run): # Only runs the first time phase1() runs so that hover checking variables are initialised
        is_hovering=0
        check=[0]
        i=0
        prevtime=time.time()
        previouspos=[position[0],position[1],position[2]]
        firstphase1run=0
        j=0


    currenttime=time.time()
    if (currenttime-prevtime>0.5) :
        i=i+1
        previouspos=currentpos
        currentpos=[position[0],position[1],position[2]]
        prevtime=currenttime #prev time records 0.4 second intervals
        if abs(currentpos[0]-previouspos[0])>0.05 or abs(currentpos[1]-previouspos[1])>0.05 or abs(currentpos[2]-previouspos[2])>0.05 :
            check.append(0)
        else :
            check.append(1)
            j=j+1

        if (j>=20): #checking how the drone has behaved in the last 5 intervals of 0.4 seconds (i.e over the last 2 seconds)
            checkfiltered=[x for x in check if (i-4)<=x<=i] #taking the last 5 values of check
            if all(x==1 for x in checkfiltered) : # if drone has moved minimally in the given time period then checkfiltered=[1,1,1,1,1]
                is_hovering=1
                print('Hover Complete')
                goto2nd=1 #stop returning to phase1() and proceed
                goto1st=0

   #set points for PID in this stage are exactly the same as for startup so no need to modify
    xVelocity = xpositionControl014.update(position[0]) #need to update xVelocity in each stage seperately as some use PID and others don't.
    yVelocity = ypositionControl.update(position[1])
    zVelocity = zpositionControl014.update(position[2])
    if zVelocity>0.3:
        zVelocity=0.3
    rollRate = yawControl.update(angpos[0])
    pitchRate = yawControl.update(angpos[1])
    yawRate = yawControl.update(angpos[2])

#############################################################################################################################################

##### CONTROLLER = creating objects

zpositionControl014 = PID(1.2, 0.00005, 0.55) #create PID controller objects with specified gains
xpositionControl014 = PID(1.2, 0.000015, 0.6)
ypositionControl = PID(1.2, 0.000002, 0.45)
yawControl = PID(1, 0, 0)
rollControl = PID(1, 0, 0)
pitchControl = PID(1, 0, 0)

######################################################################################

def takeoff():


    global tookoff
    global stateManagerInstance
    global firstphase1run

    rate = rospy.Rate(updrate)  # rate will update publisher
    stateManagerInstance = stateManager(rate)  # create new statemanager

    # State:
    rospy.Subscriber("/mavros/state", State,
                     stateManagerInstance.stateUpdate)  # Autopilot State including Arm State, Connection Status and Mode
    #rospy.Subscriber("/tera_2_array", LaserScan, teraArrayCheck)
    # Upward Facing LiDAR:
    #rospy.Subscriber("/tera_1_up", LaserScan, teraUpCheck)
    # Downward Facing LiDAR:
    #rospy.Subscriber("/tera_1_down", LaserScan, teraDownCheck)
    # REAL SETUP ONLY -> ZHAO CHANGED DOWN FACING LIDAR TO BE:
    rospy.Subscriber("/mavros/distance_sensor/hrlv_ez4_pub", Range,distanceCheck)  # Current Distance from Ground
    # Local Position Topics
    rospy.Subscriber("/mavros/local_position/odom", Odometry,odomCheck)  # Fuses Sensor Data
    rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, velCheck)

    # 7. Publishers:
    # Velpub allows to set Linear and Angular Velocity with a message structure of type TwistStamped. Queue Size balance between latest and completeness of data.
    velPub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=2)

    # 8. Velocity Control Object - Initialise with Velocity Publisher.
    controller = velControl(velPub)  # Create New Controller Class and Pass in Publisher and State Manager
    stateManagerInstance.waitForPilotConnection()  # Wait for Connection to Flight Controller

    # Avoid Proceeding Before Subscritptions Have Been Completed
    k = 0
    while (k == 0):
        if any(v == 0 for v in (position[1], range_ground)):
            print('Waiting for all of the subscriptions')
            rate.sleep()
        else:
            print('All subscriptions are active ')
            k = 1

    # DEFINE setPoint
    global setpoint
    setpoint = np.array([position[0] + setpoint[0],position[1] + setpoint[1], setpoint[2]])


    while not tookoff:

        if(has_started_flying==0):
            startup() #set goto1st as true after grange>0.2 or so. Use PID for height control. Set has_started_flying=1
        if(goto1st):
            phase1() #Still using PID for height control but checking for 5sec hover. Set goto2nd=1 when is_hovering=true and goto1st=0
        if(goto2nd):
            tookoff = 1
            #print('Took off')


################################################################


        controller.setVel([xVelocity,yVelocity, zVelocity])
        controller.setAngVel([rollRate, pitchRate, yawRate])


        controller.publishTargetPose(stateManagerInstance)
        stateManagerInstance.incrementLoop()

        rate.sleep()  # sleep at the set rate

        # Don't know what that is, but better not to touch it
        if stateManagerInstance.getLoopCount() > 100 :  # to prevent offboard rejection
            stateManagerInstance.offboardRequest()  # request control from external computer
            stateManagerInstance.armRequest()  # arming must take place after offboard is requested

    #rospy.spin()


if __name__ == '__main__':
    takeoff()
