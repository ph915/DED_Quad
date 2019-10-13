#!/usr/bin/env python

# Potential Field Based Path Planner - ROS Based Solution
# Author: Pablo Hermoso Moreno

# Version: 09_06_2018 - V13:

# - Potential Field Based Navigation.
# - 3D (x-y-z) Plane
# - Class Based Python Algorithm
# - Radius of Drone -> d_eff = d - r_drone : Avoid Collision.
# - Radius of Goal -> Once inside: Target Acquisition Mechanism Triggered.
# - Vectorised Motion: Output Velocity Commands to UAV
# - Position Estimate from IMU Data
# - Continuous Walls and Obstacles
# - Imperfect Localisation - This does not affect Obstacle Avoidance
# - Imperfect Sensors
# -> TeraRanger Tower (8 * 45 deg apart)
#    - Stated Range Precision = +- 4cm
#    - Stated Field of View = 3 deg
# -> 2 X TeraRanger One (Downward & Upward Facing)
#    - Stated Range Precision = +- 4cm
#    - Stated Field of View = 3 deg
# - Static Environment
# - Local Minima Solver
#    - Via Simulated Annealing
# - Return to Base Incorporated

# Work In Progress for - V11:
# - Localisation and Mapping for Perching STAGE
# - Grid Based Stage
# - Perching Strategy

# Description:
# Potential Field based planners work using gradient descent to go in direction which locally minimises potential function.
# Obstacles are positively charged and Goal is negatively charged.


########################################################################################################################

# Import Libraries
import numpy as np

# Plotting Library
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
plt.rcParams['legend.fontsize'] = 10

# ROS Specific Libraries:

import rospy
import math
import sys

# Takeoff Stage - Controlled by specifically built PID for increased accuracy and robustness
import TAKEOFF

# Import Message Structures:
from mavros_msgs.msg import State  # import state message structure
from sensor_msgs.msg import Range  # import range message structure
from sensor_msgs.msg import LaserScan  # import LaserScan message structure
from geometry_msgs.msg import TwistStamped  # used to set velocity messages
from nav_msgs.msg import Odometry  # import range message structure
from mavros_msgs.srv import *  # import for arm and flight mode setting

########################################################################################################################

# ROS Specific Parameters:

updrate = float(20)  # Rate at Which Publisher is Updated
deltat = float(1 / updrate)  # Timestep

# Sensor Outputs:
range_ground = 0  # Range From Ground [m]
range_ceiling = 0  # Range From Ceiling [m]
range_tera_array = np.array([0, 0, 0, 0, 0, 0, 0, 0]) # TeraRanger Array Ranges [m] - Total of 8
angles_tera_array = np.array([180, 135, 90, 45, 0, -45, -90, -135])  # TeraRanger Array Angles [deg] - Total of 8
position = np.array([0, 0, 0])
velocity = np.array([0, 0, 0])
angpos = np.array([0, 0, 0])

# Stage Parameters:

# Stage 1 - Takeoff
stage1Height = 0.6  # [m]
stage1Climb_Vel = 0.10 #[m/s]

# Stage 2 & Stage 4:
hover_time = 5 # [s]

# Stage 3 - Cruise to Perching Location:
# Note -> Significant Levels Of Overshoot!
go = np.array([2.5, 0, stage1Height])  # Goal x and y and z position [m]

# Stage 4 - Hover 2
stage4_hover_pos = np.array([0, 0, 0]) # This is populated once the drone is within goal radius.

# Stage 6 - First Descent:
stage6_land_pos = np.array([0, 0, 0]) # This is populated once the drone is within goal radius.
stage7Des_Vel = -0.10 #[m/s]

# Stage 7 - Second Descent:
stage8Des_Vel = -0.05 #[m/s]

# Stages: 1-Takeoff; 2-Hover 1; 3-Cruise; 4-Hover 2; 5-Return to Base; 6-Hover; 7-Landing 1; 8-Landing 2
stage1Completed = 0
stage2Completed = 0
stage3Completed = 0
stage4Completed = 0
stage5Completed = 0
stage6Completed = 0
stage7Completed = 0
# No need to initialise last landing stage

# PID Parameters:

# Initialise PID integration term
errorint = np.array([0, 0, 0])

########################################################################################################################

# Potential Fields Parameters
KP = 3.5 # Attractive Potential Gain (zeta in PRM Book)
ETA = 0.00125  # Repulsive Potential Gain

# Repusive Action - Number of Obstacles - After meeting with Dr Paranjape 29_05_2018:
# e.g -> For 1 LIDAR obstacle detection / update -> For 1 second memory -> deltat = 0.05 -> n = 20
# Calculation: 10 Sensors - deltat = 0.05 - Desired Memory ~ 3s
memory_repulsive = 600 # Number of Repulsive Obstacles Memory.

# Drone Radius and Goal Radius
r_drone = 0.35
r_goal = 0.30

# Gradient Descent Parameters:
# - alpha_pot : Gradient Descent Factor Gain Factor (Critical to limit velocity outputs)
alpha_pot = 0.025

# Maximum and Minimum Velocities:
max_vel = 0.20
min_vel = 0.05

# Potential Function Parameters:

# Attractive Potential:
# Threshold Distance d^{*}_{goal}
d_star = 1.0  # [m]

# Repulsive Potential:
# Critical Range Q^{*}
q_star = 1.0  # [m]

# Local Minima Solving: Simulated Annealing ->

# 1. Identify Local Minimum State:
r_local_min = 0.15  # [m]
num_traj_local_min = 400 # 14s Local Minima

# 2. Trigger Simulated Annealing Algorithm:
r_sim_ann = 0.35  # [m] - Same as Drone Radius
disc_theta_sim_ann = 5  # [deg]
disc_phi_sim_ann = 5  # [deg]

T_0_init = 300  # [K]
T_0_curr = 300  # [K]

cooling_r = 0.9 # Simulated Annealing Cooling Rate

# show_animation -> if false - Trajectory and Vectors not plotted.
show_animation = True


########################################################################################################################

# USEFUL FUNCTIONS:

# 1. Modified Vertical Stack:
# - Includes check for empty arrays.
def vstack_mod(or_array, new_array):
    if len(or_array) > 1:
        return np.vstack([or_array, new_array])
    else:
        return np.array(new_array)


########################################################################################################################

# POTENTIAL FIELD CLASS DEFINITIONS:

class uav:

    # ROS Implementation:
    # For this version - ROS based functions are identified with "_2"

    # Initialisation:
    # Key Parameter:
    # - obs_quad : n*2 array containing (x,y) of obstacles DETECTED BY UAV.
    # - r_quad : Trajectory (x,y) History [m]
    def __init__(self, cp):
        self.cp = cp

        # Drone's Obstacle Map:
        self.obs_quad = np.array([[]])

        # Drone's Trajectory
        self.r_quad = np.array([cp])

    # Distance Return: Hypotenuse to any location.
    def dist(self, pos):

        # 3D
        d = np.linalg.norm(position - pos)

        return d

    # Calculate Unit Vector to Given Location in 3D Space
    def unit_vec(self, pos):

        # Pre_Divide distance to save computational power.
        if ( (self.dist(pos)) != 0 ) and ( (self.dist(pos)) != float('Inf') ):
            d_inv = 1 / (self.dist(pos))
            return np.array([d_inv * (pos[0] - position[0]), d_inv * (pos[1] - position[1]), d_inv * (pos[2] - position[2])])
        else:
            return np.array([0,0,0])

    def terraranger_scan(self):

        # Append Obstacles to obs_quad

        # 1. Down Facing TeraRanger One:
        x_obs_down = position[0] - range_ground * np.sin(np.deg2rad(angpos[1]))
        y_obs_down = position[1] + range_ground * np.sin(np.deg2rad(angpos[0]))
        #z_obs_down = position[2] - range_ground * np.cos(np.deg2rad(angpos[0])) * np.cos(np.deg2rad(angpos[1]))
        z_obs_down = 0

        self.obs_quad = vstack_mod(self.obs_quad, [x_obs_down, y_obs_down, z_obs_down])

        # 2. Up Facing TeraRanger One:
        x_obs_up = position[0] + range_ceiling * np.sin(np.deg2rad(angpos[1]))
        y_obs_up = position[1] - range_ceiling * np.sin(np.deg2rad(angpos[0]))
        z_obs_up = position[2] + range_ceiling * np.cos(np.deg2rad(angpos[0])) * np.cos(np.deg2rad(angpos[1]))

        self.obs_quad = vstack_mod(self.obs_quad, [x_obs_up, y_obs_up, z_obs_up])

        # 2. TeraRanger Tower:
        for i in range(len(range_tera_array)):
            if range_tera_array[i] != float('Inf'):
                x_obs_array = position[0] + range_tera_array[i] * np.cos( np.deg2rad( angles_tera_array[i] - angpos[2] ) ) * np.cos(np.deg2rad(angpos[1]))
                y_obs_array = position[1] - range_tera_array[i] * np.sin( np.deg2rad( angles_tera_array[i] - angpos[2] ) ) * np.cos(np.deg2rad(angpos[0]))
                z_obs_array = position[2] - range_tera_array[i] * np.sin( np.deg2rad( angles_tera_array[i] ) ) * np.sin(np.deg2rad(angpos[0])) - range_tera_array[i] * np.cos( np.deg2rad( angles_tera_array[i] ) ) * np.sin(np.deg2rad(angpos[1]))

                self.obs_quad = vstack_mod(self.obs_quad, [x_obs_array, y_obs_array, z_obs_array])

        return self.obs_quad

    def calc_potential_field_quad(self, go):

        # Scan the environment and add LIDAR detected obstacles.
        obs_quad = self.terraranger_scan()

        # For the current position, calculate the Resultant Gradient Descent Vector:

        # 3D
        [u_g, v_g, w_g] = self.calc_attractive_potential_vector(go)

        # Calculate Repulsive Potential With Respect to Obstacles

        # 3D
        [u_o, v_o, w_o] = self.calc_repulsive_potential_vector()

        return [u_g + u_o, v_g + v_o, w_g + w_o], obs_quad

    def calc_attractive_potential_vector(self, go):
        # Need to implement Combined (Quadratic + Conic) Variation.

        # Reason: In some cases, it may be desirable to have distance functions
        # that grow more slowly to avoid huge velocities far from the goal.

        # 3D

        # Distance to Goal
        d = self.dist(go)

        # Unit Vector to Goal:
        d_hat = self.unit_vec(go)

        # Gradient Calculation
        if d <= d_star:
            return [KP * d * d_hat[0], KP * d * d_hat[1], KP * d * d_hat[2]]
        else:
            return [d_star * KP *  d_hat[0], d_star * KP *  d_hat[1], d_star * KP *  d_hat[2]]

    def calc_repulsive_potential_vector(self):

        # Better implementation of the repulsive potential would be to sum the actions of all the individual repulsive
        # potentials of the obtacles within Vision Range.

        # Previous model would search only for closest obsacle and take the action of only that one.
        # Problem: When numerically implementing this solution, a path may form that oscillates around points that are
        #  two-way equidistant from obstacles, i.e., points where D is nonsmooth. To avoid these oscillations,
        # instead of defining the repulsive potential function in terms of distance to the closest obstacle,
        # the repulsive potential function (4.6) is redefined in terms of distances to individual obstacles
        # where d i ( q )i s the distance to obstacle.

        # Modified Repulsive Potential Vector - After meeting with Dr Paranjape 29_05_2018:
            # Take only the latest "memory_repulsive" detected obstacles
            # "range(len(self.obs_quad))" -> "range(min(len(self.obs_quad),20))"
            # Start reading from the bottom obs obsquad -> [-i]
            # Hopefully this will in this first test reduce the overshoot in height


        u_o = 0
        v_o = 0
        w_o = 0

        if len(self.obs_quad) > 1:
            for i in range(min(len(self.obs_quad),memory_repulsive)):
                if len(self.obs_quad.shape) != 1:

                    # 3D
                    d = self.dist(self.obs_quad[-i])

                    # Unit Vector to Obstacle
                    d_hat = self.unit_vec(self.obs_quad[-i])
                else:

                    # 3D
                    d = self.dist(self.obs_quad)

                    d_hat = self.unit_vec(self.obs_quad)


                # Dylan's Collision Avoidance:
                d -= r_drone

                if (d <= q_star):
                    u_o += ETA * ((1 / q_star) - (1 / d)) * ((1 / d) ** 2) * d_hat[0]
                    v_o += ETA * ((1 / q_star) - (1 / d)) * ((1 / d) ** 2) * d_hat[1]
                    w_o += ETA * ((1 / q_star) - (1 / d)) * ((1 / d) ** 2) * d_hat[2]

        return [u_o, v_o, w_o]

    # Simulated Annealing Algorithm - NOT IMPLEMENTED WITHOUT TERARANGER TOWER:
    def calc_attractive_potential(self, new_pos, go):
        # Combined (Quadratic + Conic) Variation.

        # Reason: In some cases, it may be desirable to have distance functions
        # that grow more slowly to avoid huge velocities far from the goal.

        # Distance to Goal
        # 3D
        d = np.linalg.norm(go - new_pos)

        if d <= d_star:
            return 0.5 * KP * (d ** 2)
        else:
            return (d_star * KP * d) - (0.5 * KP * (d_star ** 2))

    def calc_repulsive_potential(self, new_pos):

        # Better implementation of the repulsive potential would be to sum the actions of all the individual repulsive
        # potentials of the obtacles within Vision Range.

        # Previous model would search only for closest obsacle and take the action of only that one.
        # Problem: When numerically implementing this solution, a path may form that oscillates around points that are
        #  two-way equidistant from obstacles, i.e., points where D is nonsmooth. To avoid these oscillations,
        # instead of defining the repulsive potential function in terms of distance to the closest obstacle,
        # the repulsive potential function (4.6) is redefined in terms of distances to individual obstacles
        # where d i ( q )i s the distance to obstacle.

        # MODIFIED ETA FOR SIMULATED ANNEALING:
        ETA = 0.005

        u_rep = 0

        if len(self.obs_quad) > 1:
            for i in range(min(len(self.obs_quad),memory_repulsive)):

                if len(self.obs_quad.shape) != 1:
                    # 3D
                    d = np.linalg.norm(self.obs_quad[-i] - new_pos)
                else:

                    #3D
                    d = np.linalg.norm(self.obs_quad - new_pos)

                # Collision Avoidance:
                # d_eff = d - r_drone: Avoid Collision
                d -= r_drone

                # Check if the obstacle if within Vision Range Q^{*}.
                if (d <= q_star):
                    u_rep += 0.5 * ETA * ((1.0 / d) - (1.0 / q_star)) ** 2

        return u_rep

    def check_local_min(self):

        # Check if the latest "num_traj_local_min" trajectory lies with radius "r_local_min" from current trajectory:
        if len(self.r_quad) > num_traj_local_min:

            d = self.dist(self.r_quad[-num_traj_local_min])

            if d < r_local_min:  # Radius: "r_local_min"
                return True
            else:
                return False

    def sim_annealing_alg(self, go):

        # Set Initial Temperature:
        global T_0_curr

        # Current U(x)
        ug_cp = self.calc_attractive_potential(position, go)
        uo_cp = self.calc_repulsive_potential(position)


        # Spherical Coordinates:
        # Strategy:
        # -> First test in "theta" (X-Y)
        # -> If no success, test "phi" (Z)

        theta = 0
        phi = 90
        sign_theta = -1
        counter_theta = 0
        sign_phi = -1
        counter_phi = 0

        while abs(phi - 90) < 30:
            while abs(theta) < 180:

                # 1. Calculate x' = x + delta_x
                # Spherical Coordinates:
                x_new = self.cp[0] + r_sim_ann * np.cos(np.deg2rad(theta)) * np.sin(np.deg2rad(phi))
                y_new = self.cp[1] + r_sim_ann * np.sin(np.deg2rad(theta)) * np.sin(np.deg2rad(phi))
                z_new = self.cp[2] + r_sim_ann * np.cos(np.deg2rad(phi))

                # 2. Calculate U(x')
                ug_new = self.calc_attractive_potential([x_new, y_new, z_new], go)
                uo_new = self.calc_repulsive_potential([x_new, y_new, z_new])

                # 3. Calculate delta_U
                delta_U = (-ug_new + uo_new) - (-ug_cp + uo_cp)

                # 4. Probabilistic Algorithm:
                if delta_U < 0:
                    # Found Direction With Negative Gradient:
                    break
                else:
                    # Probability:
                    p_new = np.exp((-delta_U) / (T_0_curr))
                    if np.random.random_sample() < p_new:
                        break

                theta = (sign_theta * counter_theta * disc_theta_sim_ann)
                sign_theta = sign_theta * -1

                if sign_theta == 1:
                    counter_theta += 1

            phi = 90 + (sign_phi * counter_phi * disc_phi_sim_ann)

            sign_phi = sign_phi * -1

            if sign_phi == 1:
                counter_phi += 1

        # Reduce T_0 by cooling rate:
        T_0_curr = cooling_r * T_0_curr

        # Once the angle has been set, the drone will try to escape local minima at minimum velocity:
        u_res = min_vel * np.cos(np.deg2rad(theta)) * np.sin(np.deg2rad(phi))
        v_res = min_vel * np.sin(np.deg2rad(theta)) * np.sin(np.deg2rad(phi))
        w_res = min_vel * np.cos(np.deg2rad(phi))

        return u_res, v_res, w_res

########################################################################################################################

# ROS SPECIFIC CLASS DEFINITIONS

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
        # rospy.logwarn("Connected is {}, armed is {}, mode is {} ".format(self._isConnected, self._isArmed,self._mode))  # some status info

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
        # rospy.logwarn("Waiting for pilot connection")
        while not rospy.is_shutdown():  # while not shutting down
            if self._isConnected:  # if state isConnected is true
                # rospy.logwarn("Pilot is connected")
                return True
            self._rate.sleep
        # rospy.logwarn("ROS shutdown")
        return False

class velControl:
    def __init__(self, attPub):  # attPub = attitude publisher
        self._attPub = attPub
        self._setVelMsg = TwistStamped()
        self._targetVelX = 0
        self._targetVelY = 0
        self._targetVelZ = 0

    def setVel(self, coordinates):
        self._targetVelX = float(coordinates[0])
        self._targetVelY = float(coordinates[1])
        self._targetVelZ = float(coordinates[2])
        # rospy.logwarn("Target velocity is \nx: {} \ny: {} \nz: {}".format(self._targetVelX, self._targetVelY,self._targetVelZ))

    def publishTargetPose(self, stateManagerInstance):
        self._setVelMsg.header.stamp = rospy.Time.now()  # construct message to publish with time, loop count and id
        self._setVelMsg.header.seq = stateManagerInstance.getLoopCount()
        self._setVelMsg.header.frame_id = 'fcu'

        self._setVelMsg.twist.linear.x = self._targetVelX
        self._setVelMsg.twist.linear.y = self._targetVelY
        self._setVelMsg.twist.linear.z = self._targetVelZ

        self._attPub.publish(self._setVelMsg)

class ROS:
    # Functions Called When Subscribers Are Updated
    # - distanceCheck = distancesensor - callback = OpticalFlow

    def distanceCheck(self, msg):
        global range_ground  # import global range
        range_ground = msg.range  # set range = received range

    def teraArrayCheck(self, msg):
        global range_tera_array

        # GAZEBO SIMULATION ENVIRONMENT
        # Understanding Message Structure of "msg.ranges"
        # Degrees : [180,135,90,45,0,315,270,225,-] - Saved as GLOBAL Array
        # IMP : Real Setup No 180deg
        # NOTE: Array Slicing in Python Requires to end index one item before.
        range_tera_array = np.array(msg.ranges[0:8])

    def teraUpCheck(self, msg):
        global range_ceiling
        range_ceiling = msg.ranges[0]

    def teraDownCheck(self, msg):
        global range_ground
        range_ground = msg.ranges[0]

    def odomCheck(self, msg):
        global position
        global angpos
        global range_ground
        angpos = self.quaternion_to_euler_angle(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                                msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
        xPosition = msg.pose.pose.position.x
        yPosition = msg.pose.pose.position.y

        # EXPERIMENT - Does not work correctly - Use range_ground from LiDAR
        #zPosition = msg.pose.pose.position.z

        # NOTE:
        # I found that using "msg.pose.pose.position.z" gives smoother results for current height.
        # For some reason "range_ground" was able to detect I was climbing?
        # For the moment being, use PID in height once the terraranger tower has been installed

        # Z-Position Correction with Angles - theta_x & theta_y :

        # x_Angle = angpos[0]
        # y_Angle = angpos[1]
        # z_Angle = angpos[2]

        zPosition = range_ground * math.cos(math.radians(angpos[1])) * math.cos(math.radians(angpos[0]))

        position = np.array([xPosition, yPosition, zPosition])

    def velCheck(self, msg):
        global velocity
        xVel = msg.twist.linear.x
        yVel = msg.twist.linear.y
        zVel = msg.twist.linear.z
        velocity = np.array([xVel, yVel, zVel])

    # Quartenion to Euler Angle Conversion

    def quaternion_to_euler_angle(self, w, x, y, z):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.degrees(math.atan2(t3, t4))
        angpos = np.array([X, Y, Z])
        return angpos

    # PID Controller:
    def PID(self, position, vel, setpoint,const):  # Inputs: current position, current velocity, desired state, PID constants

        global errorint
        global deltat

        error = np.array(setpoint) - position
        prop = const[0] * error
        integ = const[1] * errorint
        der = -1 * const[2] * vel  # NB: d(error)/dt = d(setpoint)/dt - d(position)/dt = -velocity

        output = prop + integ + der
        errorint = errorint + error * deltat

        for i in [0, 1, 2]:
            output[i] = np.sign(output[i]) * min(abs(output[i]), max_vel)

        return output

########################################################################################################################
def main():
    print("Potential_Field_Planning_Start")

    # ARTIFICIAL POTENTIAL FIELDS:

    # 1. Set the Environment

    # MAKE SURE THE STARTING POSITION IS NOT ON A BORDER
    st = np.array([0, 0, 0])  # Start x and y position [m]
    global go  # Goal x and y position [m]

    # 2. Set the Drone:

    # Drone Class:
    drone = uav(st)

    # 3. Initialise ROS Class:
    ros_class = ROS()

    # 4. Initialise ROS Node:
    rospy.init_node('Invincible_Destroyer')

    # 5. Set Up State Manager:
    rate = rospy.Rate(updrate)  # Rate will update publisher at 20hz, higher than the 2hz minimum before timeouts occur
    stateManagerInstance = stateManager(rate)  # Create StateManager Object and initialise with rate.

    # 6. Subscriptions:
    # We require:
    # - State
    # - Distance From Ground
    # - Distance From Ceiling
    # - TeraRanger Tower Distances
    # - Velocity


    # State:
    rospy.Subscriber("/mavros/state", State,
                     stateManagerInstance.stateUpdate)  # Autopilot State including Arm State, Connection Status and Mode

    # Range Topics: REMEMBER TO SUBSTITUTE FOR REAL TOPICS IN REAL SETUP

    # TeraRanger Tower
    rospy.Subscriber("/tera_2_array", LaserScan, ros_class.teraArrayCheck)

    # Upward Facing LiDAR:
    rospy.Subscriber("/tera_1_up", LaserScan, ros_class.teraUpCheck)

    # Downward Facing LiDAR:

    rospy.Subscriber("/tera_1_down", LaserScan, ros_class.teraDownCheck)
    # REAL SETUP ONLY -> ZHAO CHANGED DOWN FACING LIDAR TO BE:
    #rospy.Subscriber("/mavros/distance_sensor/hrlv_ez4_pub", Range,ros_class.distanceCheck)  # Current Distance from Ground

    # Local Position Topics
    rospy.Subscriber("/mavros/local_position/odom", Odometry,ros_class.odomCheck)  # Fuses Sensor Data
    rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, ros_class.velCheck)

    # 7. Publishers:
    # Velpub allows to set Linear and Angular Velocity with a message structure of type TwistStamped. Queue Size balance between latest and completeness of data.

    velPub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=2)

    # 8. Velocity Control Object - Initialise with Velocity Publisher.
    controller = velControl(velPub)  # Create New Controller Class and Pass in Publisher and State Manager
    stateManagerInstance.waitForPilotConnection()  # Wait for Connection to Flight Controller

    # 9. Start Giving Instructions:



    while not rospy.is_shutdown():

        if 'range_ground' in globals():

            global stage1Completed
            global stage2Completed
            global stage3Completed
            global stage4Completed
            global stage5Completed
            global stage6Completed
            global stage7Completed

            global position
            global angpos
            global deltat
            global velocity
            global errorint
            global T_0_curr
            global T_0_init

            # Stage parameters:
            global stage1Height

            # Stages Description:

            # 1. Takeoff to Height = stage1Height
            # Use PID Control Design for L3
            # Climb at stage1ClimbVel m/s

            # 2. Hover for 5s @ Height = stage1Height

            # 3. Cruise to Perching Location
            # Potential Fields Algorithm

            # 4. Hover for 5s @ Height = stage1Height

            # 5. Return to Base

            # 6. First Descent

            # 7 Second & Last Descent

            if stage1Completed == 0:

                # Initiliaise Takeoff & Hover From Separate Script
                TAKEOFF.takeoff()

                stage1Completed = 1
                stage2Completed = 1
                print('Stage 1 - Takeoff - COMPLETE')
                print('Stage 2 - Hover - COMPLETE')

                # Hover for 5 seconds
                loopCount = stateManagerInstance.getLoopCount()

            # IMPORTANT UPDATE: Hover has been incorporated in Takeoff Script
            # This Stage Is Left for Backup Purposes
            if (stage1Completed == 1) & (stage2Completed == 0):

                # 2. Hover for 5s

                # Add Trajectory To History
                drone.r_quad = np.vstack([drone.r_quad, position])

                if stateManagerInstance.getLoopCount() - loopCount < (hover_time * updrate):
                    PIDout = ros_class.PID(position, velocity, [0, 0, stage1Height], [3.3, 0.3, 0.1])
                    controller.setVel([PIDout[0], PIDout[1], PIDout[2]])

                else:
                    print('Stage 2 - Hover - COMPLETE')

                    # Move to stage 3
                    stage2Completed = 1
                    errorint = np.array([0, 0, 0])

            if (stage1Completed == 1) & (stage2Completed == 1) & (stage3Completed == 0):

                # ARTIFICIAL POTENTIAL FIELDS:

                # Global Distance from Start to Goal:
                d = drone.dist(go)  # Global Distance To Target

                # Add Trajectory To History
                drone.r_quad = np.vstack([drone.r_quad, position])

                if d >= r_goal:  # UAV within Goal Radius

                    # Calculate Gradient and Velocity Command in Inertial Frame:
                    [u_res, v_res, w_res], obs_quad = drone.calc_potential_field_quad(go)

                    # Check for Existance of Local Minima State:
                    if drone.check_local_min():
                        print('LOCAL MINIMA STATE')
                        # Trigger Simulated Annealing Algorithm:
                        u_res, v_res, w_res = drone.sim_annealing_alg(go)
                    else:
                        # If Drone Escapes Local Minima - Reinitialise Current Temperature
                        T_0_curr = T_0_init

                    alpha = alpha_pot

                    # 3D
                    if np.linalg.norm([alpha * u_res, alpha * v_res, alpha * w_res]) == 0:
                        alpha = 0
                    elif np.linalg.norm(
                            [alpha * u_res, alpha * v_res, alpha * w_res]) > max_vel:  # Maximum Velocity Limit:
                        alpha = max_vel / (np.linalg.norm([u_res, v_res, w_res]))
                    elif np.linalg.norm(
                            [alpha * u_res, alpha * v_res, alpha * w_res]) < min_vel:  # Minimum Velocity Limit:
                        alpha = min_vel / (np.linalg.norm([u_res, v_res, w_res]))

                    # Update Position by Sending Velocity Command:
                    controller.setVel([alpha * u_res, alpha * v_res, alpha * w_res])


                else: # Withing Goal Radius

                    print('Stage 3 - Cruise to Perching Location - COMPLETE')

                    #Move to stage 4.
                    stage3Completed = 1
                    # Hover for 2 second
                    loopCount = stateManagerInstance.getLoopCount()
                    # Restart errorint after each stage.
                    errorint = np.array([0, 0, 0])

                    # For Inital Phase Testing - Set Current Pos as Landing Pos
                    stage4_hover_pos = position

            if (stage1Completed == 1) & (stage2Completed == 1) & (stage3Completed == 1) & (stage4Completed == 0):

                # Add Trajectory To History
                drone.r_quad = np.vstack([drone.r_quad, position])

                # Begin by hovering:
                # Hover for 2 second
                if stateManagerInstance.getLoopCount() - loopCount < (hover_time * updrate):
                    PIDout = ros_class.PID(position, velocity, stage4_hover_pos, [3, 1.10, 0.23])
                    controller.setVel([PIDout[0], PIDout[1], PIDout[2]])

                else:
                    print('Stage 4 - Hover - COMPLETE')

                    # Move to stage 5.
                    stage4Completed = 1
                    errorint = np.array([0, 0, 0])

            if (stage1Completed == 1) & (stage2Completed == 1) & (stage3Completed == 1) & (stage4Completed == 1) & (stage5Completed == 0):

                # Return to Base - New Goal = [0,0,0] = Origin
                return_goal = np.array([0,0,stage1Height])

                # ARTIFICIAL POTENTIAL FIELDS:

                # Global Distance from Start to Goal:
                d = drone.dist(return_goal)  # Global Distance To Target

                # Add Trajectory To History
                drone.r_quad = np.vstack([drone.r_quad, position])

                if d >= r_goal:  # UAV within Goal Radius

                    # Calculate Gradient and Velocity Command in Inertial Frame:
                    [u_res, v_res, w_res], obs_quad = drone.calc_potential_field_quad(return_goal)

                    # Check for Existance of Local Minima State:
                    if drone.check_local_min():
                        print('LOCAL MINIMA STATE')
                        # Trigger Simulated Annealing Algorithm:
                        u_res, v_res, w_res = drone.sim_annealing_alg(return_goal)
                    else:
                        # If Drone Escapes Local Minima - Reinitialise Current Temperature
                        T_0_curr = T_0_init

                    alpha = alpha_pot

                    # 3D
                    if np.linalg.norm([alpha * u_res, alpha * v_res, alpha * w_res]) == 0:
                        alpha = 0
                    elif np.linalg.norm(
                            [alpha * u_res, alpha * v_res, alpha * w_res]) > max_vel:  # Maximum Velocity Limit:
                        alpha = max_vel / (np.linalg.norm([u_res, v_res, w_res]))
                    elif np.linalg.norm(
                            [alpha * u_res, alpha * v_res, alpha * w_res]) < min_vel:  # Minimum Velocity Limit:
                        alpha = min_vel / (np.linalg.norm([u_res, v_res, w_res]))

                    # Update Position by Sending Velocity Command:
                    controller.setVel([alpha * u_res, alpha * v_res, alpha * w_res])


                else: # Withing Goal Radius

                    print('Stage 5 - Return to Base - COMPLETE')

                    #Move to stage 6.
                    stage5Completed = 1
                    # Hover for 2 second
                    loopCount = stateManagerInstance.getLoopCount()
                    # Restart errorint after each stage.
                    errorint = np.array([0, 0, 0])

                    # For Inital Phase Testing - Set Current Pos as Landing Pos
                    stage6_land_pos = position

            if (stage1Completed == 1) & (stage2Completed == 1) & (stage3Completed == 1) & (stage4Completed == 1) & (stage5Completed == 1) & (stage6Completed == 0):

                # Add Trajectory To History
                drone.r_quad = np.vstack([drone.r_quad, position])

                # Begin by hovering:
                # Hover for 2 second
                if stateManagerInstance.getLoopCount() - loopCount < (hover_time * updrate):
                    PIDout = ros_class.PID(position, velocity, stage6_land_pos, [3, 1.10, 0.23])
                    controller.setVel([PIDout[0], PIDout[1], PIDout[2]])

                else:
                    print('Stage 6 - Hover - COMPLETE')

                    # Move to stage 7.
                    stage6Completed = 1
                    errorint = np.array([0, 0, 0])

            if (stage1Completed == 1) & (stage2Completed == 1) & (stage3Completed == 1) & (stage4Completed == 1) & (stage5Completed == 1) & (stage6Completed == 1) & (stage7Completed == 0):

                # Add Trajectory To History
                drone.r_quad = np.vstack([drone.r_quad, position])

                if position[2] > 0.4:
                    # Control PID in x and y

                    PIDout = ros_class.PID(position, velocity, stage6_land_pos, [3, 1.01, 0.25])
                    controller.setVel([PIDout[0], PIDout[1], stage7Des_Vel])

                else:
                    print('Stage 7 - First Descent - COMPLETE')

                    stage7Completed = 1
                    errorint = np.array([0, 0, 0])

            if (stage1Completed == 1) & (stage2Completed == 1) & (stage3Completed == 1) & (stage4Completed == 1) & (stage5Completed == 1) & (stage6Completed == 1) & (stage7Completed == 1):

                # Add Trajectory To History
                drone.r_quad = np.vstack([drone.r_quad, position])

                if position[2] > 0.1:
                    # Control PID in x and y

                    PIDout = ros_class.PID(position, velocity, stage6_land_pos, [3, 1.01, 0.25])
                    controller.setVel([PIDout[0], PIDout[1], stage8Des_Vel])

                else:
                    print('Stage 8 - Second Descent - COMPLETE')

                    ##########################################################

                    # FINISHED TRAJECTORY:

                    ##########################################################

                    # Visualisation of Obstacles & Trajectory:

                    if show_animation:

                        # 2D Plots

                        # X-Y Plane:

                        # Create Figure + Subplot
                        fig1, ax1 = plt.subplots()

                        ax1.plot(drone.r_quad[:,0], drone.r_quad[:,1], '.r' , label='UAV Trajectory')

                        if drone.obs_quad.size != 0:
                            if drone.obs_quad.size > 3:
                                ax1.plot(drone.obs_quad[:,0] ,drone.obs_quad[:,1] , 'sg', label='Detected Obstacles')
                            else:
                                ax1.plot(drone.obs_quad[0], drone.obs_quad[1], 'sg', label='Detected Obstacles')

                        ax1.legend()
                        ax1.set_xlabel('X / [m]')
                        ax1.set_ylabel('Y / [m]')
                        ax1.set_title('X-Y UAV Trajectory and Detected Obstacles')

                        # Y-Z Plane:

                        # Create Figure + Subplot
                        fig3, ax3 = plt.subplots()

                        ax3.plot(drone.r_quad[:, 1], drone.r_quad[:, 2], '.r', label='UAV Trajectory')

                        if drone.obs_quad.size != 0:
                            if drone.obs_quad.size > 3:
                                ax3.plot(drone.obs_quad[:, 1], drone.obs_quad[:, 2], 'sg', label='Detected Obstacles')
                            else:
                                ax3.plot(drone.obs_quad[1], drone.obs_quad[2], 'sg', label='Detected Obstacles')

                        ax3.legend()
                        ax3.set_xlabel('Y / [m]')
                        ax3.set_ylabel('Z / [m]')
                        ax3.set_title('Y-Z UAV Trajectory and Detected Obstacles')

                        # X-Z Plane:

                        # Create Figure + Subplot
                        fig4, ax4 = plt.subplots()

                        ax4.plot(drone.r_quad[:, 0], drone.r_quad[:, 2], '.r', label='UAV Trajectory')

                        if drone.obs_quad.size != 0:
                            if drone.obs_quad.size > 3:
                                ax4.plot(drone.obs_quad[:, 0], drone.obs_quad[:, 2], 'sg', label='Detected Obstacles')
                            else:
                                ax4.plot(drone.obs_quad[0], drone.obs_quad[2], 'sg', label='Detected Obstacles')

                        ax4.legend()
                        ax4.set_xlabel('X / [m]')
                        ax4.set_ylabel('Z / [m]')
                        ax4.set_title('X-Z UAV Trajectory and Detected Obstacles')

                        # 3D Trajectory Path & Obstacles Detected:

                        #Create 3D Figure + Subplots:

                        fig2 = plt.figure()
                        ax2 = fig2.add_subplot(111, projection='3d')

                        if drone.r_quad.size != 0:
                            if drone.r_quad.size > 3:
                                #ax2.plot(drone.r_quad[:,0],drone.r_quad[:,1],drone.r_quad[:,2])
                                ax2.scatter(drone.r_quad[:,0],drone.r_quad[:,1],drone.r_quad[:,2], c='r', marker='o', label='UAV Trajectory')
                                ax2.scatter(drone.obs_quad[:,0],drone.obs_quad[:,1],drone.obs_quad[:,2], c='g', marker='s', label='Detected Obstacles')
                            else:
                                #ax2.plot(drone.r_quad[0],drone.r_quad[1],drone.r_quad[2])
                                ax2.scatter(drone.r_quad[0],drone.r_quad[1],drone.r_quad[2], c='r', marker='o',label='UAV Trajectory')
                                ax2.scatter(drone.obs_quad[0],drone.obs_quad[1],drone.obs_quad[2], c='g', marker='s', label='Detected Obstacles')

                        ax2.legend()
                        ax2.set_xlabel('X / [m]')
                        ax2.set_ylabel('Y / [m]')
                        ax2.set_zlabel('Z / [m]')
                        ax2.set_title('3D UAV Trajectory and Detected Obstacles')

                        plt.draw()
                        plt.show()


                    # Save to text:

                    # 1. Trajectory
                    np.savetxt('drone.r_quad.out', drone.r_quad, delimiter=',')
                    # 2. obs_quad
                    np.savetxt('drone.obs_quad.out', drone.obs_quad, delimiter=',')

                    # Goodbye!
                    print('Goodbye!')

                    sys.exit()

        else:
            print('Waiting for Range')

        controller.publishTargetPose(stateManagerInstance)

        stateManagerInstance.incrementLoop()
        rate.sleep()  # sleep at the set rate
        if stateManagerInstance.getLoopCount() > 100:  # need to send some position data before we can switch to offboard mode otherwise offboard is rejected
            stateManagerInstance.offboardRequest()  # request control from external computer
            stateManagerInstance.armRequest()  # arming must take place after offboard is requested
    rospy.spin()  # keeps python from exiting until this node is stopped


if __name__ == '__main__':
    main()
