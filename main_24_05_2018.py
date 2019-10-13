"""
Potential Field Based Path Planner
Author: Pablo Hermoso Moreno

Version: 24_05_2018 - V4:

- Potential Field Based Navigation.
- 2D (x-y) Plane
- Class Based Python Algorithm
- Radius of Drone -> d_eff = d - r_drone : Avoid Collision.
- Radius of Goal -> Once inside: Target Acquisition Mechanism Triggered.
- Vectorised Motion: Assumed Constant Velocity Motion s.t delta_x = u * delta_t
- Continuous Walls and Obstacles
- Perfect Localisation
- Imperfect Sensors -> Teraranger Tower (8 * 45 deg apart)
    - Range Precision of +- 4cm has been incorporated
    - Field of View has NOT been modelled. i.e Field of View = 0 deg
- Static Environment
- Local Minima Solver
    - Via Simulated Annealing

Work In Progress for - V5:
- Coupled Localsiation and Mapping

Description:
Potential Field based planners work using gradient descent to go in direction which locally minimises potential function.
Obstacles are positively charged and Goal is negatively charged.

# IMPORTANT NOTES:
- Coordinates (CP) are rounded to 4 d.p -> For Teraranger detection purposes.
    - This GLOBAL variable has been left to the used for modification.

"""

# Import Libraries
import numpy as np
import matplotlib.pyplot as plt

# Parameters
KP = 5 # Attractive Potential Gain (zeta in PRM Book)
ETA = 0.1  # Repulsive Potential Gain

# Map Dimensions & Resolution:
mapSizeX = 4
mapSizeY = 3
res_map = 0.5

# Drone Radius and Goal Radius
r_drone = 0.35
r_goal = 0.20

# Precision Parameters Model:
dp_coord = 4
dp_ang = 3

# Dynamics of Drone:

# Key Parameter Definitions:
        # - delta_t : Set timestep between position updates.
        # - alpha_pot : Gradient Descent Factor Gain Factor (Critical to limit velocity outputs)

delta_t = 0.1
alpha_pot = 0.0125

# Maximum and Minimum Velocities:
max_vel = 0.3
min_vel = 0.1

# Sensor Parameters:
lidar_range = 14 # [m]
lidar_precision = 0.04 # [m] - Remember +-

# White Noise Addition:
add_sensor_noise = True

# Potential Function Parameters:

# Attractive Potential:
# Threshold Distance d^{*}_{goal}
d_star = 3  # [m]

# Repulsive Potential:
# Critical Range Q^{*}
q_star = 1.0 # [m]

# Local Minima Solving: Simulated Annealing ->

# 1. Identify Local Minimum State:
r_local_min = 0.1 # [m]
num_traj_local_min = 20

# 2. Trigger Simulated Annealing Algorithm:
r_sim_ann = 0.1 #[m]
disc_theta_sim_ann = 5 #[deg]
T_0 = 500 #[K]

# show_animation -> if false - Trajectory and Vectors not plotted.
show_animation = True

#################################################################################

# USEFUL FUNCTIONS:

# 1. Modified Vertical Stack:
    # - Includes check for empty arrays.
def vstack_mod(or_array,new_array):
    if len(or_array) > 1:
        return np.vstack([or_array, new_array])
    else:
        return new_array

#################################################################################

# CLASS DEFINITIONS:

# ENVIRONMENT:

class environment:

    # Initialisation:
        # Key Parameter:
        # - obs_env : n*2 array containing (x,y) of obstacles in Environment.
        # - st : Start (x,y) Location [m]
        # - go : Goal (x,y) Location [m]
    def __init__(self, size_x , size_y , go = np.array([]) , st = np.array([])):
        self.size_x = size_x
        self.size_y = size_y
        self.obs_env = np.array([ np.array([]) ])
        self.go = go
        self.st = st

    # ADDING OBSTACLES
    # Terminology:
        # - disc : Discretisation of Obstacle (x,y)
        # (For Sinusoidal)
        # - amp : Amplitude of Oscillation
        # - freq : Frequency of Oscillation

    # 1. Simple Point:
    def add_obs_env_simple(self,co_obs_simple):
        self.obs_env = vstack_mod( self.obs_env , co_obs_simple )

    # 2. Straight Horizontal Wall:
    def add_straight_hor_wall(self,x_in,y_in,length,disc):

        # Coordinates Of Wall:
        x_wall = np.arange(x_in, x_in + length, disc)
        y_wall = np.ones(x_wall.size) * y_in

        for i in range(x_wall.size):
            self.obs_env = vstack_mod(self.obs_env, [round(x_wall[i], dp_coord), round(y_wall[i], dp_coord)] )

    # 2. Straight Vertical Wall:
    def add_straight_ver_wall(self, x_in, y_in, length, disc):

        # Coordinates Of Wall:
        y_wall = np.arange(y_in, y_in + length, disc)
        x_wall = np.ones(y_wall.size) * x_in

        for i in range(y_wall.size):
            self.obs_env = vstack_mod(self.obs_env, [round(x_wall[i], dp_coord), round(y_wall[i], dp_coord)])

    # 3. Sinusoidally Curved Horizontal Wall:
    def add_sinusoid_hor_wall(self,x_in,y_in,length,disc,amp,freq):

        # Coordinates Of Wall:
        x_wall = np.arange(x_in, x_in + length, disc)
        y_wall = y_in + (amp * np.sin(x_wall * freq))

        for i in range(x_wall.size):
            self.obs_env = vstack_mod(self.obs_env, [round(x_wall[i], dp_coord), round(y_wall[i], dp_coord)] )

    # 3. Sinusoidally Curved Vertical Wall:
    def add_sinusoid_ver_wall(self, x_in, y_in, length, disc, amp, freq):

        # Coordinates Of Wall:
        y_wall = np.arange(y_in, y_in + length, disc)
        x_wall = x_in + (amp * np.sin(y_wall * freq))

        for i in range(x_wall.size):
            self.obs_env = vstack_mod(self.obs_env, [round(x_wall[i], dp_coord), round(y_wall[i], dp_coord)])

    # 4. Circular Obstacle;
    def add_circ_obs(self,x_c,y_c,r_obs,disc_theta):
        theta = 0
        while theta < 360:
            self.obs_env = vstack_mod(self.obs_env, [round(x_c + r_obs * np.cos(np.deg2rad(theta)), dp_coord), round(y_c + r_obs * np.sin(np.deg2rad(theta)), dp_coord)])
            theta = theta + disc_theta

    # VISUALISATION
    def plot_map(self):

        # Equal Grid
        plt.grid(True)
        plt.axis("equal")

        # Plot Walls & Obstacle Map
        if len(self.obs_env) > 1:
            plt.plot( self.obs_env[:,0] / res_map, self.obs_env[:,1] / res_map, '.k', markersize=1)

        # Start And Goal Positions:
        plt.plot(self.st[0] / res_map, self.st[1] / res_map, "*k")  # START
        plt.plot(self.go[0] / res_map, self.go[1] / res_map, "*m")  # GOAL

class uav:

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
        d = np.hypot(self.cp[0] - pos[0], self.cp[1] - pos[1])
        return d

    def update_pos(self,u_res,v_res,go):


        # Check for Existance of Local Minima State:
        if self.check_local_min():
            # Trigger Simulated Annealing Algorithm:
            [u_res,v_res] = self.sim_annealing_alg(go)

        # Gradient Descent Method:

        # Avoid Backward Motion of Drone -> Eliminate Tendency of Escaping: # NOT NECESSARY!
        #if u_res < 0:
            #u_res = 0

        # Velocity limits are set by modification of the gradient descent scaling factor alpha.

        alpha = alpha_pot

        if np.hypot(alpha * u_res, alpha * v_res) == 0:
            alpha = 0
        elif np.hypot(alpha * u_res, alpha * v_res) > max_vel: # Maximum Velocity Limit:
            alpha = max_vel / (np.hypot(u_res, v_res))
        elif np.hypot(alpha * u_res, alpha * v_res) < min_vel: # Minimum Velocity Limit:
            alpha = min_vel / (np.hypot(u_res, v_res))

        # Update Position:
            # - We assume constant velocity in delta_t
        self.cp[0] += alpha * u_res * delta_t
        self.cp[1] += alpha * v_res * delta_t

        # Round to 4 d.p
            # - Rounding is critical for numerical errors.
        self.cp[0] = round(self.cp[0], 4)
        self.cp[1] = round(self.cp[1], 4)

        # Update the trajectory:
        self.r_quad = np.vstack([self.r_quad, self.cp])

    def terraranger_scan(self,obs_env):


        # Buffer: These arrays will hold the obstacles detected by each sensor. Only the closest one will be appended to obs_quad.
        buffer_0 = np.array([[]])
        buffer_180 = np.array([[]])
        buffer_90 = np.array([[]])
        buffer_270 = np.array([[]])
        buffer_45 = np.array([[]])
        buffer_135 = np.array([[]])
        buffer_225 = np.array([[]])
        buffer_315 = np.array([[]])

        if obs_env.size != 0:
            for i in range(len(obs_env)):

                if len(obs_env.shape) != 1:
                    d = np.hypot(self.cp[0] - obs_env[i][0], self.cp[1] - obs_env[i][1])
                else:
                    d = np.hypot(self.cp[0] - obs_env[0], self.cp[1] - obs_env[1])


                # Obstancle Detection:
                # 1. Must be within visible range
                # 2. Should not have been recorded previously
                # 3. Only record closest obstacle in sight of LIDAR (no vision across obstacles).

                if (d <= lidar_range):

                    # Calculate Angle to Obstacle:
                    if len(obs_env) != 0:
                        theta = np.arctan2(obs_env[i][1] - self.cp[1], obs_env[i][0] - self.cp[0])  # -pi / +pi

                    else:
                        theta = np.arctan2(obs_env[1] - self.cp[1], obs_env[0] - self.cp[0])  # -pi / +pi

                    # Round to Specifiend Precision -> For Numerical Innacuracies
                    theta = round(theta,dp_ang)

                    # 0 Degrees:
                    if theta == round(0, dp_ang):
                        buffer_0 = vstack_mod(buffer_0,obs_env[i])

                    # 45 Degrees:
                    if theta == round(np.pi / 4, dp_ang):
                        buffer_45 = vstack_mod(buffer_45, obs_env[i])

                    # 90 Degrees:
                    if theta == round(np.pi / 2, dp_ang):
                        buffer_90 = vstack_mod(buffer_90, obs_env[i])

                    # 135 Degrees:
                    if theta == round(3 * np.pi / 4, dp_ang):
                        buffer_135 = vstack_mod(buffer_135, obs_env[i])

                    # 180 Degrees:
                    if theta == round(np.pi, dp_ang):
                        buffer_180 = vstack_mod(buffer_180, obs_env[i])

                    # 225 Degrees:
                    if theta == round(- 3 * np.pi / 4, dp_ang):
                        buffer_225 = vstack_mod(buffer_225, obs_env[i])

                    # 270 Degrees:
                    if theta == round( - np.pi / 2, dp_ang):
                        buffer_270 = vstack_mod(buffer_270, obs_env[i])

                    # 315 Degrees:
                    if theta == round( - np.pi / 4, dp_ang):
                        buffer_315 = vstack_mod(buffer_315, obs_env[i])

        # Find the closest distance obstacle in the buffer for the sensors and append to obs_quad:

        # NOTE:
        # minid = -1 - EMPTY
        # minid = -2 - 1 ENTRY
        # minid >= 0 - MORE THAN 1 ENTRY

        # Noise addition:
        if add_sensor_noise:

            # Range Precision:

            # 0 deg:
            buffer_0 += (2 * lidar_precision * (np.random.random(buffer_0.shape))) - lidar_precision
            # 45 deg:
            buffer_45 += (2 * lidar_precision * (np.random.random(buffer_45.shape))) - lidar_precision
            # 90 deg:
            buffer_90 += (2 * lidar_precision * (np.random.random(buffer_90.shape))) - lidar_precision
            # 135 deg:
            buffer_135 += (2 * lidar_precision * (np.random.random(buffer_135.shape))) - lidar_precision
            # 180 deg:
            buffer_180 += (2 * lidar_precision * (np.random.random(buffer_180.shape))) - lidar_precision
            # 225 deg:
            buffer_225 += (2 * lidar_precision * (np.random.random(buffer_225.shape))) - lidar_precision
            # 270 deg:
            buffer_270 += (2 * lidar_precision * (np.random.random(buffer_270.shape))) - lidar_precision
            # 315 deg:
            buffer_315 += (2 * lidar_precision * (np.random.random(buffer_315.shape))) - lidar_precision

        minid_front = -1
        dmin_front = float("inf")
        minid_back = -1
        dmin_back = float("inf")
        minid_right = -1
        dmin_right = float("inf")
        minid_left = -1
        dmin_left = float("inf")
        minid_45 = -1
        dmin_45 = float("inf")
        minid_135 = -1
        dmin_135 = float("inf")
        minid_225 = -1
        dmin_225 = float("inf")
        minid_315 = -1
        dmin_315 = float("inf")

        if buffer_0.size != 0:  # BUFFER NOT EMPTY
            for i in range(len(buffer_0)):
                if len(buffer_0.shape) != 1:
                    d = np.hypot(self.cp[0] - buffer_0[i][0], self.cp[1] - buffer_0[i][1])
                else:
                    minid_front = -2
                    break

                # Search for closest distance obstacle.
                if dmin_front >= d:
                    dmin_front = d
                    minid_front = i

            # Collection of if statements used to find the appropiate way to append the new obstacle to obs_quad:
            if (self.obs_quad.size == 0) & (minid_front == -2):
                self.obs_quad = buffer_0
            elif (self.obs_quad.size == 0) & (minid_front >= 0):
                self.obs_quad = buffer_0[minid_front]
            elif (self.obs_quad.size == 2) & (minid_front == -2):
                if (not (self.obs_quad == buffer_0).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_0])
            elif (self.obs_quad.size == 2) & (minid_front >= 0):
                if (not (self.obs_quad == buffer_0[minid_front]).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_0[minid_front]])
            elif (self.obs_quad.size > 2) & (minid_front == -2):
                if (not (self.obs_quad == buffer_0).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_0])
            elif (self.obs_quad.size > 2) & (minid_front >= 0):
                if (not (self.obs_quad == buffer_0[minid_front]).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_0[minid_front]])

        if buffer_180.size != 0:  # BUFFER NOT EMPTY
            for i in range(len(buffer_180)):
                if len(buffer_180.shape) != 1:
                    d = np.hypot(self.cp[0] - buffer_180[i][0], self.cp[1] - buffer_180[i][1])
                else:
                    minid_back = -2
                    break

                # Search for closest distance obstacle.
                if dmin_back >= d:
                    dmin_back = d
                    minid_back = i

            # Collection of if statements used to find the appropiate way to append the new obstacle to obs_quad:
            if (self.obs_quad.size == 0) & (minid_back == -2):
                self.obs_quad = buffer_180
            elif (self.obs_quad.size == 0) & (minid_back >= 0):
                self.obs_quad = buffer_180[minid_back]
            elif (self.obs_quad.size == 2) & (minid_back == -2):
                if (not (self.obs_quad == buffer_180).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_180])
            elif (self.obs_quad.size == 2) & (minid_back >= 0):
                if (not (self.obs_quad == buffer_180[minid_back]).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_180[minid_back]])
            elif (self.obs_quad.size > 2) & (minid_back == -2):
                if (not (self.obs_quad == buffer_180).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_180])
            elif (self.obs_quad.size > 2) & (minid_back >= 0):
                if (not (self.obs_quad == buffer_180[minid_back]).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_180[minid_back]])

        if buffer_90.size != 0:  # BUFFER NOT EMPTY
            for i in range(len(buffer_90)):
                if len(buffer_90.shape) != 1:
                    d = np.hypot(self.cp[0] - buffer_90[i][0], self.cp[1] - buffer_90[i][1])
                else:
                    minid_right = -2  # Only one obstacle found
                    break

                # Search for closest distance obstacle.
                if dmin_right >= d:
                    dmin_right = d
                    minid_right = i

            # Collection of if statements used to find the appropiate way to append the new obstacle to obs_quad:
            if (self.obs_quad.size == 0) & (minid_right == -2):
                self.obs_quad = buffer_90
            elif (self.obs_quad.size == 0) & (minid_right >= 0):
                self.obs_quad = buffer_90[minid_right]
            elif (self.obs_quad.size == 2) & (minid_right == -2):
                if (not (self.obs_quad == buffer_90).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_90])
            elif (self.obs_quad.size == 2) & (minid_right >= 0):
                if (not (self.obs_quad == buffer_90[minid_right]).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_90[minid_right]])
            elif (self.obs_quad.size > 2) & (minid_right == -2):
                if (not (self.obs_quad == buffer_90).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_90])
            elif (self.obs_quad.size > 2) & (minid_right >= 0):
                if (not (self.obs_quad == buffer_90[minid_right]).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_90[minid_right]])

        if buffer_270.size != 0:  # BUFFER NOT EMPTY
            for i in range(len(buffer_270)):
                if len(buffer_270.shape) != 1:
                    d = np.hypot(self.cp[0] - buffer_270[i][0], self.cp[1] - buffer_270[i][1])
                else:
                    minid_left = -2
                    break

                # Search for closest distance obstacle.
                if dmin_left >= d:
                    dmin_left = d
                    minid_left = i

            # Collection of if statements used to find the appropiate way to append the new obstacle to obs_quad:
            if (self.obs_quad.size == 0) & (minid_left == -2):
                self.obs_quad = buffer_270
            elif (self.obs_quad.size == 0) & (minid_left >= 0):
                self.obs_quad = buffer_270[minid_left]
            elif (self.obs_quad.size == 2) & (minid_left == -2):
                if (not (self.obs_quad == buffer_270).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_270])
            elif (self.obs_quad.size == 2) & (minid_left >= 0):
                if (not (self.obs_quad == buffer_270[minid_left]).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_270[minid_left]])
            elif (self.obs_quad.size > 2) & (minid_left == -2):
                if (not (self.obs_quad == buffer_270).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_270])
            elif (self.obs_quad.size > 2) & (minid_left >= 0):
                if (not (self.obs_quad == buffer_270[minid_left]).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_270[minid_left]])

        if buffer_45.size != 0:  # BUFFER NOT EMPTY
            for i in range(len(buffer_45)):
                if len(buffer_45.shape) != 1:
                    d = np.hypot(self.cp[0] - buffer_45[i][0], self.cp[1] - buffer_45[i][1])
                else:
                    minid_45 = -2
                    break

                # Search for closest distance obstacle.
                if dmin_45 >= d:
                    dmin_45 = d
                    minid_45 = i

            # Collection of if statements used to find the appropiate way to append the new obstacle to obs_quad:
            if (self.obs_quad.size == 0) & (minid_45 == -2):
                self.obs_quad = buffer_45
            elif (self.obs_quad.size == 0) & (minid_45 >= 0):
                self.obs_quad = buffer_45[minid_45]
            elif (self.obs_quad.size == 2) & (minid_45 == -2):
                if (not (self.obs_quad == buffer_45).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_45])
            elif (self.obs_quad.size == 2) & (minid_45 >= 0):
                if (not (self.obs_quad == buffer_45[minid_45]).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_45[minid_45]])
            elif (self.obs_quad.size > 2) & (minid_45 == -2):
                if (not (self.obs_quad == buffer_45).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_45])
            elif (self.obs_quad.size > 2) & (minid_45 >= 0):
                if (not (self.obs_quad == buffer_45[minid_45]).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_45[minid_45]])

        if buffer_135.size != 0:  # BUFFER NOT EMPTY
            for i in range(len(buffer_135)):
                if len(buffer_135.shape) != 1:
                    d = np.hypot(self.cp[0] - buffer_135[i][0], self.cp[1] - buffer_135[i][1])
                else:
                    minid_135 = -2
                    break

                # Search for closest distance obstacle.
                if dmin_135 >= d:
                    dmin_135 = d
                    minid_135 = i

            # Collection of if statements used to find the appropiate way to append the new obstacle to obs_quad:
            if (self.obs_quad.size == 0) & (minid_135 == -2):
                self.obs_quad = buffer_135
            elif (self.obs_quad.size == 0) & (minid_135 >= 0):
                self.obs_quad = buffer_135[minid_135]
            elif (self.obs_quad.size == 2) & (minid_135 == -2):
                if (not (self.obs_quad == buffer_135).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_135])
            elif (self.obs_quad.size == 2) & (minid_135 >= 0):
                if (not (self.obs_quad == buffer_135[minid_135]).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_135[minid_135]])
            elif (self.obs_quad.size > 2) & (minid_135 == -2):
                if (not (self.obs_quad == buffer_135).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_135])
            elif (self.obs_quad.size > 2) & (minid_135 >= 0):
                if (not (self.obs_quad == buffer_135[minid_135]).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_135[minid_135]])

        if buffer_225.size != 0:  # BUFFER NOT EMPTY
            for i in range(len(buffer_225)):
                if len(buffer_225.shape) != 1:
                    d = np.hypot(self.cp[0] - buffer_225[i][0], self.cp[1] - buffer_225[i][1])
                else:
                    minid_225 = -2
                    break

                # Search for closest distance obstacle.
                if dmin_225 >= d:
                    dmin_225 = d
                    minid_225 = i

            # Collection of if statements used to find the appropiate way to append the new obstacle to obs_quad:
            if (self.obs_quad.size == 0) & (minid_225 == -2):
                self.obs_quad = buffer_225
            elif (self.obs_quad.size == 0) & (minid_225 >= 0):
                self.obs_quad = buffer_225[minid_225]
            elif (self.obs_quad.size == 2) & (minid_225 == -2):
                if (not (self.obs_quad == buffer_225).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_225])
            elif (self.obs_quad.size == 2) & (minid_225 >= 0):
                if (not (self.obs_quad == buffer_225[minid_225]).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_225[minid_225]])
            elif (self.obs_quad.size > 2) & (minid_225 == -2):
                if (not (self.obs_quad == buffer_225).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_225])
            elif (self.obs_quad.size > 2) & (minid_225 >= 0):
                if (not (self.obs_quad == buffer_225[minid_225]).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_225[minid_225]])

        if buffer_315.size != 0:  # BUFFER NOT EMPTY
            for i in range(len(buffer_315)):
                if len(buffer_315.shape) != 1:
                    d = np.hypot(self.cp[0] - buffer_315[i][0],self.cp[1] - buffer_315[i][1])
                else:
                    minid_315 = -2
                    break

                # Search for closest distance obstacle.
                if dmin_315 >= d:
                    dmin_315 = d
                    minid_315 = i

            # Collection of if statements used to find the appropiate way to append the new obstacle to obs_quad:
            if (self.obs_quad.size == 0) & (minid_315 == -2):
                self.obs_quad = buffer_315
            elif (self.obs_quad.size == 0) & (minid_315 >= 0):
                self.obs_quad = buffer_315[minid_315]
            elif (self.obs_quad.size == 2) & (minid_315 == -2):
                if (not (self.obs_quad == buffer_315).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_315])
            elif (self.obs_quad.size == 2) & (minid_315 >= 0):
                if (not (self.obs_quad == buffer_315[minid_315]).all().any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_315[minid_315]])
            elif (self.obs_quad.size > 2) & (minid_315 == -2):
                if (not (self.obs_quad == buffer_315).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_315])
            elif (self.obs_quad.size > 2) & (minid_315 >= 0):
                if (not (self.obs_quad == buffer_315[minid_315]).all(1).any()):
                    self.obs_quad = np.vstack([self.obs_quad, buffer_315[minid_315]])

        return self.obs_quad

    def calc_potential_field_quad(self, go, obs_env):

        # Scan the environment and add LIDAR detected obstacles.
        obs_quad = self.terraranger_scan(obs_env)

        # For the current position, calculate the Resultant Gradient Descent:
        [u_g, v_g] = self.calc_attractive_potential_vector(go)

        # Calculate Repulsive Potential With Respect to Obstacles
        [u_o, v_o] = self.calc_repulsive_potential_vector()

        return [u_g + u_o, v_g + v_o], obs_quad

    def calc_attractive_potential_vector(self,go):
        # Need to implement Combined (Quadratic + Conic) Variation.

        # Reason: In some cases, it may be desirable to have distance functions
        # that grow more slowly to avoid huge velocities far from the goal.

        # Distance to Goal
        d = np.hypot(go[0] - self.cp[0], go[1] - self.cp[1])

        # Angle to Goal:
        theta = np.arctan2(go[1] - self.cp[1], go[0] - self.cp[0])  # -pi / +pi

        # Gradient Calculation
        if d <= d_star:
            return [KP * d * np.cos(theta), KP * d * np.sin(theta)]
        else:
            return [d_star * KP * np.cos(theta), d_star * KP * np.sin(theta)]

            # Previous version would only use d and not d**2:
            # return 0.5 * KP * np.hypot(x - go[0], y - go[1])

    def calc_repulsive_potential_vector(self):

        # Better implementation of the repulsive potential would be to sum the actions of all the individual repulsive
        # potentials of the obtacles within Vision Range.

        # Previous model would search only for closest obsacle and take the action of only that one.
        # Problem: When numerically implementing this solution, a path may form that oscillates around points that are
        #  two-way equidistant from obstacles, i.e., points where D is nonsmooth. To avoid these oscillations,
        # instead of defining the repulsive potential function in terms of distance to the closest obstacle,
        # the repulsive potential function (4.6) is redefined in terms of distances to individual obstacles
        # where d i ( q )i s the distance to obstacle.

        u_rep = 0
        v_rep = 0

        if self.obs_quad.size != 0:
            for i in range(len(self.obs_quad)):
                theta = 0
                if len(self.obs_quad.shape) != 1:
                    d = np.hypot(self.cp[0] - self.obs_quad[i][0], self.cp[1] - self.obs_quad[i][1])

                    # Dylan's Collision Avoidance:
                    #d_eff = d - r_drone: Avoid Collision
                    d -= r_drone

                    theta = np.arctan2(self.obs_quad[i][1] - self.cp[1], self.obs_quad[i][0] - self.cp[0])  # -pi / +pi
                else:
                    d = np.hypot(self.cp[0] - self.obs_quad[0], self.cp[1] - self.obs_quad[1])

                    # Dylan's Collision Avoidance:
                    d -= r_drone

                    theta = np.arctan2(self.obs_quad[1] - self.cp[1], self.obs_quad[0] - self.cp[0])  # -pi / +pi

                if (d <= q_star):
                    u_rep += ETA * ((1 / q_star) - (1 / d)) * ((1 / d) ** 2) * np.cos(theta)
                    v_rep += ETA * ((1 / q_star) - (1 / d)) * ((1 / d) ** 2) * np.sin(theta)

        return [u_rep, v_rep]

    # Simulated Annealing Algorithm:
    def calc_attractive_potential(self,x, y, go):
        # Combined (Quadratic + Conic) Variation.

        # Reason: In some cases, it may be desirable to have distance functions
        # that grow more slowly to avoid huge velocities far from the goal.

        # MODIFIED KP FOR SIMULATED ANNEALING:
        #KP = 5

        # Distance to Goal
        d = np.hypot(go[0] - x, go[1] - y)

        if d <= d_star:
            return 0.5 * KP * (d ** 2)
        else:
            return (d_star * KP * d) - (0.5 * KP * (d_star ** 2))

    def calc_repulsive_potential(self,x, y):

        # Better implementation of the repulsive potential would be to sum the actions of all the individual repulsive
        # potentials of the obtacles within Vision Range.

        # Previous model would search only for closest obsacle and take the action of only that one.
        # Problem: When numerically implementing this solution, a path may form that oscillates around points that are
        #  two-way equidistant from obstacles, i.e., points where D is nonsmooth. To avoid these oscillations,
        # instead of defining the repulsive potential function in terms of distance to the closest obstacle,
        # the repulsive potential function (4.6) is redefined in terms of distances to individual obstacles
        # where d i ( q )i s the distance to obstacle.

        # MODIFIED ETA FOR SIMULATED ANNEALING:
        ETA = 5

        u_rep = 0

        for i in range(len(self.obs_quad)):
            if len(self.obs_quad.shape) != 1:
                d = np.hypot(x - self.obs_quad[i][0], y - self.obs_quad[i][1])
            else:
                d = np.hypot(x - self.obs_quad[0], y - self.obs_quad[1])

            # Dylan's Collision Avoidance:
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

            if d < r_local_min: # Radius: "r_local_min"
                return True
            else:
                return False

    def sim_annealing_alg(self,go):

        # Set Initial Temperature:
        #global T_0

        # Current U(x)
        ug_cp = self.calc_attractive_potential(self.cp[0], self.cp[1], go)
        uo_cp = self.calc_repulsive_potential(self.cp[0], self.cp[1])

        theta = 0
        while theta < 360:

            # 1. Calculate x' = x + delta_x
            x_new = self.cp[0] + r_sim_ann * np.cos(np.deg2rad(theta))
            y_new = self.cp[1] + r_sim_ann * np.sin(np.deg2rad(theta))

            # 2. Calculate U(x')
            ug_new = self.calc_attractive_potential(x_new,y_new,go)
            uo_new = self.calc_repulsive_potential(x_new,y_new)

            # 3. Calculate delta_U
            delta_U = (-ug_new + uo_new) - (-ug_cp + uo_cp)

            # 4. Probabilistic Algorithm:
            if delta_U < 0:
                # Found Direction With Negative Gradient:
                break
            else:
                # Probability:
                p_new = np.exp( (-delta_U) / (T_0) )
                if np.random.random_sample() < p_new:
                    break

            theta += disc_theta_sim_ann

        # Reduce T_0:
        #T_0 = 0.90 * T_0
        #print(T_0)

        # Once the angle has been set, the drone will try to escape local minima at minimum velocity:
        u_res = min_vel * np.cos(np.deg2rad(theta))
        v_res = min_vel * np.sin(np.deg2rad(theta))
        return [u_res,v_res]

def main():
    print("Potential_Field_Planning_Start")

    # 1. Set the Environment

    # MAKE SURE THE STARTING POSITION IS NOT ON A BORDER
    st = np.array([0.5, 1.5])  # Start x and y position [m]
    go = np.array([3.5, 1.5])  # Goal x and y position [m]

    # Environment Class:
    env = environment(mapSizeX, mapSizeY, go, st)

    # Add Walls & Obstacles:

    # env.add_sinusoid_hor_wall(0,0,mapSizeX,0.01,0.2,2)
    # env.add_sinusoid_hor_wall(0,mapSizeY,mapSizeX,0.01,0.2,4)

    # Regular Obstacles: No Local Minima ->

    # This obstacle causes local minima apparition:
    #env.add_circ_obs(2,1.5,0.05,0.5)

    # env.add_circ_obs(1.5, 2, 0.15, 0.5)
    # env.add_circ_obs(1.0, 2.5, 0.1, 0.5)
    # env.add_circ_obs(0.45, 0.5, 0.20, 0.5)
    # env.add_circ_obs(2.6, 0.2, 0.25, 0.5)


    # Local Minima Obstacles ->
    # Straight Vertical Wall

    env.add_straight_hor_wall(0,0,4,0.005)
    env.add_straight_hor_wall(0,3,4,0.005)
    env.add_straight_ver_wall(2,1.25,0.45,0.001)


    # Visualisation:
    env.plot_map()

    # 2. Set the Drone:

    # Drone Class:
    drone = uav(st)

    # 3. Potential Fields Solver:

    # Global Distance from Start to Goal:
    d = drone.dist(env.go)  # Global Distance To Target

    # VISUALISATION: Initialise ->

    # Subplot Base:
    ax = plt.gca()

    # Plot Drone Radius:
    circle_drone = plt.Circle((drone.cp[0] / res_map, drone.cp[1] / res_map), r_drone / res_map, color='r', fill=False)
    ax.add_artist(circle_drone)

    # Plot Goal Radius:
    circle_goal = plt.Circle((env.go[0] / res_map, env.go[1] / res_map), r_goal / res_map, color='b', fill=False)
    ax.add_artist(circle_goal)

    # Plot Resultant Vector:
    vector_drone = plt.quiver(drone.cp[0] / res_map, drone.cp[1] / res_map, 0, 0, edgecolor='k', facecolor='None',linewidth=.5)
    ax.add_artist(vector_drone)

    while d >= r_goal:  # UAV within Goal Radius

        # Calculate Gradient
        [u_res, v_res], obs_quad = drone.calc_potential_field_quad(env.go,env.obs_env)

        drone.update_pos(u_res,v_res,env.go)

        # Re-Calculate the Distance:
        d = drone.dist(env.go)

        if show_animation:

            plt.plot(drone.cp[0] / res_map, drone.cp[1] / res_map, ".r")

            if drone.obs_quad.size != 0:
                if drone.obs_quad.size > 2:
                    plt.plot(drone.obs_quad[:,0]/res_map ,drone.obs_quad[:,1]/res_map ,"sg")
                else:
                    plt.plot(drone.obs_quad[0] / res_map, drone.obs_quad[1] / res_map, "sg")

            # Subplot Exists -> Update:

            # Plot Drone Radius:
            circle_drone.remove()
            circle_drone = plt.Circle((drone.cp[0] / res_map, drone.cp[1] / res_map), r_drone / res_map,color='r', fill=False)
            ax.add_artist(circle_drone)

            # Plot Resultant Vector:
            vector_drone.set_visible(False)
            vector_drone = plt.quiver(drone.cp[0] / res_map, drone.cp[1] / res_map, u_res, v_res, edgecolor='k', facecolor='None',linewidth=.5)
            ax.add_artist(vector_drone)

            plt.pause(0.00000001)

    print("Goal!!")


    if show_animation:
        plt.show()

if __name__ == '__main__':
    main()
