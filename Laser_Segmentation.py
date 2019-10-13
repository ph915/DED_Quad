"""
Potential Field Based Path Planner
Author: Pablo Hermoso Moreno

Description:
Up to know, the potential fields planner had been done with previous knowledge of the environment.
We will now try to implement the use of lidar to map and navigate through a maze. For the purpose of this
excercise, two maps will be created, an obstacle based map for the user and one for the drone. The latter will
be filled in as more obstacles are detected.


Assumptions:
    1. Drone knows at all times with exact precision its location.
    2. Surrounding environment is NOT dynamic.
"""

# Import Libraries
import numpy as np
import matplotlib.pyplot as plt
import math

# Parameters
KP = 0.01  # Attractive Potential Gain (zeta in PRM Book)
ETA = 0.10  # Repulsive Potential Gain
mapSizeX = 50
mapSizeY = 50

#####################################


def readings(pos, obs_env):
    one_obstacle = 0
    #left-facing sensor
    for i in range(len(obs_env)):
        if (pos[0] == obs_env[i][0]) & (pos[1] <= obs_env[i][1]):
            if (not one_obstacle):
                k = np.array([i])
                one_obstacle = 1
                d = np.array([ np.hypot(pos[0] - obs_env[i][0], pos[1] - obs_env[i][1]) ])
            elif (one_obstacle):
                k = np.append(k,i)
                d = np.append(d,np.hypot(pos[0] - obs_env[i][0], pos[1] - obs_env[i][1]))
    d_min = np.amin(d)
    #d_min = round(d_min,2)
    LiDar = np.array([d_min])

    k = None
    d = None
    #front-facing sensor
    one_obstacle = 0
    global nothing_in_front
    nothing_in_front = 0
    for i in range(len(obs_env)):
        if (pos[1] == obs_env[i][1]) & (pos[0] <= obs_env[i][0]):
            #print('YEP')
            if (not one_obstacle):
                k_front = np.array([i])
                one_obstacle = 1
                d = np.array([ np.hypot( pos[0] - obs_env[i][0], pos[1]-obs_env[i][1] ) ])
            elif (one_obstacle):
                k_front = np.append(k_front,i)
                d = np.append(d, np.hypot(pos[0] - obs_env[i][0], pos[1] - obs_env[i][1]))
    if (one_obstacle):
        d_min = np.amin(d)
        #d_min = np.round(d_min, 2)
    else:
        nothing_in_front = 1
        d_min = 14
    LiDar = np.append(LiDar, d_min)

    k = None
    d = None

    one_obstacle = 0
    #right-facing sensor
    for i in range(len(obs_env)):
        if (pos[0] == obs_env[i][0]) & (pos[1] >= obs_env[i][1]):
            if (not one_obstacle):
                k = np.array([i])

                one_obstacle = 1
                d = np.array([ np.hypot(pos[0] - obs_env[i][0], pos[1] - obs_env[i][1]) ])
            elif (one_obstacle):
                k = np.append(k,i)
                d = np.append(d,np.hypot(pos[0] - obs_env[i][0], pos[1] - obs_env[i][1]))
    #print('d right facing ', str(d))
    d_min = np.amin(d)
    #print('d_min', str(d_min))
    #d_min = round(d_min,2)

    LiDar = np.append(LiDar, d_min)

    k = None
    d = None
    #print('LiDar', str(LiDar))
    LiDar = np.around(LiDar, 3)# - 0.35
    return LiDar






def ProcessingReadings(LiDar_1,LiDar_2, terra_alpha, pos_1, pos_2):
#function constructing unit vector normal to the obstacle based on 2 redings

    global n_prev
    global shit
    shit = 0
    r = np.zeros([len(terra_alpha),2])
    n = np.zeros([len(terra_alpha),2])

    for i in range(len(terra_alpha)):

        # r - vector comprising 2 sensed points
        #print(r[i][0])
        #print(terra_alpha[0])
        r[i][0] = (pos_2[0] - pos_1[0]) + LiDar_2[i]*math.cos(abs(terra_alpha[i])) - LiDar_1[i]*math.cos(abs(terra_alpha[i]))
        #r[i][1] = (pos_2[1] - pos_1[1]) + LiDar_1[i]*math.sin(abs(terra_alpha[i])) - LiDar_2[i]*math.sin(abs(terra_alpha[i]))
        r[i][1] = (pos_2[1] - pos_1[1]) + LiDar_1[i]*math.sin(terra_alpha[i]) - LiDar_2[i]*math.sin(terra_alpha[i])
        r[i][0] = abs(r[i][0])
        r[i][1] = abs(r[i][1])

        r = np.round(r,3)
        len_r = np.sqrt(r[i][0]**2 + r[i][1]**2)
        #print('len r = ', str(len_r))
        print('r')
        print(r[i][0], r[i][1])


        # need to do a special case of when obstacle is directly in front of
        # the drone, then vector r is zero and we want our 'normal vector to be directly in front'
        x_vector = 0
        if (r[i][0] == 0) & (r[i][1] == 0):
            print('r = 0')
            if (pos_2[0] == pos_1[0]):
                print('y-vectro')
                n[i][0] = -1
                n[i][1] = 0
            if (pos_2[1] == pos_1[1]):
                x_vector =1
                print('x-vector')
                n[i][0] = 0
                n[i][1] = -1
        else:
            n[i][0] = r[i][1]/len_r
            n[i][1] = -r[i][0]/len_r


        # if (terra_alpha[i] == -math.pi/2) & (LiDar_2[i] < LiDar_1[i]):
        #     print('front')
        #     n[i][0] = -1*n[i][0]
        #     #n[i][1] = -1*n[i][1]
        # elif (terra_alpha[i] == -math.pi/2) & (LiDar_2[i] > LiDar_1[i]):
        #     print('back')
        #     #n[i][0] = -1*n[i][0]
        #     n[i][1] = -1*n[i][1]
        # #if (terra_alpha[i] == math.pi/2):
        #     n[i][1] = -1*n[i][1]

        if (terra_alpha[i] == math.pi/2) & (LiDar_2[i] < LiDar_1[i]):
            print('front')
            n[i][0] = -1*n[i][0]
            n[i][1] = -1*n[i][1]
        elif (terra_alpha[i] == math.pi/2) & (LiDar_2[i] > LiDar_1[i]):
            print('back')
            #n[i][0] = -1*n[i][0]
            n[i][1] = -1*n[i][1]

        if (terra_alpha[i] == 0.0) & (not x_vector) & (LiDar_2[i] > (LiDar_1[i])):# - (pos_2[0] - pos_1[0]))):
            n[i][0] = -1*n[i][0]
            n[i][1] = -1*n[i][1]
        elif (terra_alpha[i] == 0.0) & (not x_vector) & (LiDar_2[i] < (LiDar_1[i])):# - (pos_2[0] - pos_1[0]))):
            n[i][0] = -1*n[i][0]
            n[i][1] = -1*n[i][1]

        #if readings experience a sudden jump need to set vector to zero since it will
        # create a vector, which is large and in the wrong Direction
        if (abs(LiDar_2[i] - LiDar_1[i]) > 0.35):
            print('HUGE Difference')
            #if (terra_alpha[i] == 0.0)
            n[i][:] = n_prev[i][:]
            shit = 1


            #n[i][0] = 0
            #n[i][1] = 0

    n = np.round(n,3)
    #print(n)
    return n



#def vectorfield(terra_alpha, n):
    # x,y - location of vector start
    # dx, dy - distance in

def velocities(pos_2, n, terra_alpha, go, LiDar_1, LiDar_2):
    global shit
    global nothing_in_front
    too_close = 0

    #attractive potential
    # for now only in x direction since presumably we don't know the  exact y Location
    if (go[0] - pos_2[0] > 3):
        u_at =  (go[0] - pos_2[0]) * KP #* 0.2

    elif (go[0] - pos_2[0] <= 3):
        u_at = KP * (go[0] - pos_2[0])
    print('u_at = %3f' % (u_at))
    v_at = KP* (go[1] - pos_2[1])
    # repulsive potential
    mag_v = np.zeros([len(terra_alpha)])
    vel = np.zeros([n.shape[0],n.shape[1]])

    for i in range(len(terra_alpha)):
        #setting distance to the obstacles as the average between 2 readings
        d = (LiDar_1[i] + LiDar_2[i])/2 #+ 0.35;
        if (d < 0.4):
            too_close = 1
            print('TOO CLOSE!!!!')
            D = i

        #elif(d >= 6):
            #mag_v[i] = 0
        else:
            mag_v[i] = ETA * (1/d)
        if shit:
            mag_v[i] = 0.2
        #if nothing_in_front:
            #mag_v[i] = 0
        vel[i][:] = n[i][:]*mag_v[i]
    #print('vel for %s' % (str(terra_alpha[i])))
    print(vel)
    velocity = np.zeros([2])
    velocity[0] = u_at + vel.sum(axis = 0)[0]
    if (velocity[0]>0.25):
        velocity[0] = 0.25
    if (velocity[0]<-0.25):
        velocity[0] = -0.25
    velocity[1] = v_at + vel.sum(axis = 0)[1]
    if (velocity[1]>0.25):
        velocity[1] = 0.25
    if (velocity[1] < -0.25):
        velocity[1] = -0.25

    if (velocity[0] < 0.05):
        velocity[0] = 0.05

    if too_close:
        if (terra_alpha[D] == -math.pi/2):
            velocity = np.array([0, -0.05])
        if (terra_alpha[D] == 0.0):
            velocity = np.array([-0.05, 0])
        if (terra_alpha[D] == math.pi/2):
            velocity = np.array([0, 0.05])
    velocity = np.round(velocity,3)

    #print('velocity')
    #print(velocity)
    return velocity



############################




#
def main():
    print("Potential_Field_Planning Start")

    show_animation = False

    # MAKE SURE THE STARTING POSITION IS NOT ON A BORDER

    # Drone will know what its starting and goal positions are.

    res_map = 0.05  # Resolution Map Grip [m]
    vision_radius = 14.0  # Robot Vision Radius [m] - Q* IN BOOK.
    # Check vision radius with specifications from Terraranger.


    # CREATE TUNNEL WALLS

    # 1. STRAIGHT WALLS
    #left wall
    wLx = np.arange(0,4.5, 0.001)
    wLy = 0.4 * np.sin(wLx) + 1.5 #1.5 * np.ones(len(wLx))#np.sqrt(wLx)*0.6 + 1.3


    wall_left = np.zeros([len(wLx), 2])
    for i in range(len(wLx)):
        wall_left[i][0] = wLx[i]
        wall_left[i][1] = wLy[i]
    #print('left wall', str(wall_left), 'SIZE', wall_left.shape )


    # right wall

    wRx = np.arange(0,4.5, 0.001)
    wRy = -1.5*np.ones(len(wRx))#np.sqrt(wRx)*0.3 -1.4

    wall_right = np.zeros([len(wRx), 2])
    for i in range(len(wRx)):
        wall_right[i][0] = wRx[i]
        wall_right[i][1] = wRy[i]
    #print('size', wall_right.shape )

    #circle centered 15,25:
    c_th = np.arange(0, 2*math.pi, 0.001)

    r1 = 0.4#*np.ones(len(c_th))
    cen_1 = np.array([1.5,-0.3])
    c1x = r1 * np.cos(c_th) + cen_1[0]
    c1y = r1 * np.sin(c_th) + cen_1[1]



    circ_front = np.zeros([len(c1x), 2])
    for i in range(len(c1x)):
        circ_front[i][0] = c1x[i]
        circ_front[i][1] = c1y[i]

    #circle centered 25,15:
    r2 = 0.2#*np.ones(len(c2x))
    cen_2 = np.array([2.5,0.2])
    c2x = r2*np.cos(c_th) + cen_2[0]
    c2y = r2*np.sin(c_th) + cen_2[1]



    circ_rear = np.zeros([len(c2x), 2])
    for i in range(len(c2x)):
        circ_rear[i][0] = c2x[i]
        circ_rear[i][1] = c2y[i]




    obs_env = np.vstack([wall_left, wall_right, circ_front, circ_rear])
    obs_env = np.round(obs_env, 3)
    #print('size', len(obs_env) )
    np.savetxt("obstacle.txt", obs_env,fmt="%3f")

    plt.plot(wLx, wRy, color = 'dimgrey')
    plt.plot(wRx,wLy, color = 'grey')
    plt.plot(c1x, c1y, color = 'grey')
    plt.plot(c2x, c2y, color = 'grey')
    #plt.scatter(cen_1[0],cen_1[1], marker = 'o', color = 'grey', s = 5500*r1 )
    plt.ylim(-2,2)
    plt.xlim(-1,5)
    #plt.arrow(15,15, 2,3, head_width = 0.5)






    ################################################RRRRROMMMMMAAAANNN#########################
    t = 0 #initialise time
    dt = 0.5
    show_animation = True
    st = np.array([0.0,0.0]) # Start x and y position [m]
    go = np.array([4,0])
    #Note that y is forward-facing drone direction
    v0 = np.array([0.15, 0.15]) # initial velocity

    #define an array of sensors characterised by their angle to the front face of the DRONE
    # starboard positive
    terra_alpha = np.array([ -math.pi/2, 0.0, math.pi/2 ])

    if (t == 0):
        pos_1 = st
        print('initial position', str(pos_1))
        v = v0
        # readings at the very first time instant
    LiDar_1 = readings(pos_1, obs_env)
    print('LiDar_1', str(LiDar_1))

    #plt.show()
    k = 0
    global nothing_in_front
    global n_prev
    global v_prev
    while (pos_1[0] <= go[0]):
    #while (k<150):
        pos_2 = (dt*v) + pos_1
        pos_2 = np.round(pos_2, 3)
        print('pos(t-1_)', str(pos_1))
        print('pos', str(pos_2))
        LiDar_2 = readings(pos_2, obs_env)
        print('LiDar_1',  str(LiDar_1))
        print('LiDar_2', str(LiDar_2))
        n =ProcessingReadings(LiDar_1, LiDar_2, terra_alpha, pos_1, pos_2)
        n_prev = n
        print('n')
        print(n)
        # if (k == 0 ) | (not k%10):
        #     for i in range(len(terra_alpha)):
        #         print(i)
        #         plt.scatter(pos_2[0], pos_2[1], marker = 'x', color = 'r')
        #         plt.quiver(pos_2[0] + LiDar_2[i]*math.cos(terra_alpha[i]), pos_2[1] - LiDar_2[i]*math.sin(terra_alpha[i]), n[i][0], n[i][1], scale_units = 'x', width = 0.002, headwidth = 5, headlength = 8)
        if show_animation:
            for i in range(len(terra_alpha)):
                #xprint(i)
                if not ((i ==1)  & (nothing_in_front)):
                    plt.quiver(pos_2[0] + LiDar_2[i]*math.cos(terra_alpha[i]), pos_2[1] - LiDar_2[i]*math.sin(terra_alpha[i]), n[i][0], n[i][1], scale_units = 'x', width = 0.002, headwidth = 5, headlength = 8)
            plt.scatter(pos_2[0], pos_2[1], marker = 'x', color = 'r')
            plt.pause(0.05)
        v = velocities(pos_2, n, terra_alpha, go, LiDar_1, LiDar_2)
        if (v[0]<0):
            v[0]=-0.1
        print('velocity', str(v))
        v_prev = v
        #now prepare fot the next iteration
        pos_1 = pos_2
        LiDar_1 = LiDar_2
        k = k+1
        print('Loop %d' % (k))
    ###########################################################################################




    if show_animation:
        plt.grid(True)
        plt.axis("equal")

    # Path Generation
    # Without Environment Knowledge
    #  r_quad = potential_field_planning_quad(st, go , obs_env, obs_quad, res_map, vision_radius)
    #plt.show()
    if show_animation:
        plt.show()


if __name__ == '__main__':
    main()
