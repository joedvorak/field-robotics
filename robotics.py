# -*- coding: utf-8 -*-
"""
Created on Mon Nov 30 21:05:08 2020

@author: Tuck
"""
# dependent modules 
import numpy as np
import matplotlib.pyplot as plt

'''
Position and Frame Transformation
======================================================================================
'''

def trans12(x_tran,y_tran):
    """Creates a homogeneous 2D translation matrix
       
    Parameters
    ----------
    x_tran : float
        The translation in the x-direction
    y_tran : float
        The translation in the y-direction    
    
    Returns
    ----------
    res_mat: np.array
        A homogenous matrix for the requested translation
    """

    mat = np.zeros((3,3))
    mat[0][2] = x_tran
    mat[1][2] = y_tran
    mat[0][0] = 1
    mat[1][1] = 1
    mat[2][2] = 1
    
    return mat

def trot2(angle, deg_or_rad = "rad"):
    """Creates a 2D rotation matrix
    
    Parameters
    ----------
    angle : float
        The angle of rotation for the transformation
    deg_or_rad : str, optional, default = "rad"
        A label for the frame printed near its origin
    
    Returns
    ----------
    res_mat: np.array
        A homogenous matrix for the requested rotation
    
    """
        
    if deg_or_rad == 'deg':
        angle = angle*np.pi/180
        res_mat = np.array([[np.cos(angle),-1*np.sin(angle),0],
                            [np.sin(angle),np.cos(angle),0],
                           [0,0,1]])
    return res_mat

def trplot2(T, frame_color='r', frame_Name=False, frame_length = 1, ax = False, axis = [-1, 5, -1, 5]):
    """Plots a 2D reference frame
    
    At a minimum, a homogeneous transformation matrix for the frame must be
    provided. Other parameters are optional.
    
    Parameters
    ----------
    frame_color : str, optional
        The frame color using the Matplotlib color notation
    frame_Name : str, optional
        A label for the frame printed near its origin
    frame_length, int, optional
        The length of the arms of the frame
    ax: matplotlib.axes.Axes object, optional
        A new figure is created if ax is not provided. Otherwise the function
        will plot on the provided axis, ax.
    axis: list, optional
        A list with the min and max values for the figure in the form:
        [xmin, xmax, ymin, ymax]. Default is from -1 to 5 for both x and y.
    
    """

    frame = np.array([[0,0,frame_length],
                      [frame_length,0,0],
                      [1,1,1]])
    frameR = T @ frame
    if ax == False:
        fig, ax = plt.subplots(figsize=(5,5))
    ax.plot()
    ax.set_xlim(axis[0],axis[1])
    ax.set_ylim(axis[2],axis[3])
    ax.grid(True)
    ax.annotate("",xy=(frameR[0,0],frameR[1,0]), xytext=(frameR[0,1],frameR[1,1]),
                arrowprops=dict(arrowstyle="->", color=frame_color,
                                shrinkA=0,shrinkB=0))
    ax.annotate("",xy=(frameR[0,2],frameR[1,2]), xytext=(frameR[0,1],frameR[1,1]),
                arrowprops=dict(arrowstyle='->',color=frame_color,
                                shrinkA=0,shrinkB=0))
    if frame_Name == False:
        ax.annotate(r"$x$", xy = (frameR[0,2],frameR[1,2]), #Annotate x
                horizontalalignment='left', verticalalignment='bottom')
        ax.annotate(r"$y$".format(frame_Name), xy = (frameR[0,0],frameR[1,0]), #Annotate y
                    horizontalalignment='left', verticalalignment='bottom')
    else:
        ax.annotate("{"+"{}".format(frame_Name)+"}", xy = (frameR[0,1],frameR[1,1]), #Annotate the orign
                horizontalalignment='right', verticalalignment='top')
        ax.annotate(r"$x_{{{}}}$".format(frame_Name), xy = (frameR[0,2],frameR[1,2]), #Annotate x
                    horizontalalignment='left', verticalalignment='bottom')
        ax.annotate(r"$y_{{{}}}$".format(frame_Name), xy = (frameR[0,0],frameR[1,0]), #Annotate y
                horizontalalignment='left', verticalalignment='bottom')
    return ax

def plot_car(x, y, yaw, truckcolor="k"):  # pragma: no cover
    """Plots a vehicle representation  at a given position
    
    Parameters
    ----------
    x : float
        The vehicle location in the x-direction
    y : float
        The vehicle location in the y-direction
    yaw : float
        The pose angle for the vehicle in radians
    truckcolor: str, optional, default = "k"
        The line format (both color and style in Matplotlib representation)
        for the outline of the vehicle.
    
    """    
    
    car_frame = np.array([[0,0,4.5,4.5,0],
                          [0,2,2  ,0  ,0],
                          [1,1,1  ,1  ,1]])
    
    BR_tire = np.array([[.5 ,.5,1.5,1.5,.5 ],
                        [.25,.5,.5 ,.25,.25],
                        [1  ,1 ,1  ,1  ,1  ]])
    
    BL_tire = np.array([[.5 ,.5  ,1.5 ,1.5,.5 ],
                        [1.5,1.75,1.75,1.5,1.5],
                        [1  ,1   ,1   ,1  ,1  ]])
    
    FR_tire = np.array([[3  ,3 ,4 ,4  ,3  ],
                        [.25,.5,.5,.25,.25],
                        [1  ,1 ,1 ,1  ,1  ]])
    
    FL_tire = np.array([[3  ,3   ,4   ,4  ,3  ],
                        [1.5,1.75,1.75,1.5,1.5],
                        [1  ,1   ,1   ,1  ,1  ]])
    
    arrow = np.array([[4,3,0],
                      [1,1,0],
                      [1,1,1]])
    
    car_frame_res = trot2(yaw,"deg")@trans12(x,y)@car_frame
    FL_tire_res = trot2(yaw,"deg")@trans12(x,y)@FL_tire
    FR_tire_res = trot2(yaw,"deg")@trans12(x,y)@FR_tire
    BL_tire_res = trot2(yaw,"deg")@trans12(x,y)@BL_tire
    BR_tire_res = trot2(yaw,"deg")@trans12(x,y)@BR_tire
    arrow_res = trot2(yaw,"deg")@trans12(x,y)@arrow

    fig, ax = plt.subplots(figsize=(5, 5))
    ax.set_xlim(-1, 5)
    ax.set_ylim(-1, 5)
    ax.grid(True)
    
    ax.plot(car_frame_res[0],car_frame_res[1],color=truckcolor)
    ax.plot(BL_tire_res[0],BL_tire_res[1],color=truckcolor)
    ax.plot(BR_tire_res[0],BR_tire_res[1],color=truckcolor)    
    ax.plot(FL_tire_res[0],FL_tire_res[1],color=truckcolor)
    ax.plot(FR_tire_res[0],FR_tire_res[1],color=truckcolor)
    
    ax.annotate("",xy=(arrow_res[0,0],arrow_res[1,0]), xytext=(arrow_res[0,1],arrow_res[1,1]),
                arrowprops=dict(arrowstyle="->", color='r',
                                shrinkA=0,shrinkB=0))
 
    return ax

'''
Differential Steer
=============================================================================================
'''

def diffsteerm1(wr,wl,rw,W,dt,x,y,theta):
    
    angle = theta*np.pi/180
    vr = wr * rw
    vl = wl * rw
    V = (vr+vl)/2
    omega = (vr-vl)/W
    R = V/omega
    xICC = x-R*np.sin(angle)
    yICC = y+R*np.cos(angle)
    ICC = np.array([[xICC], [yICC]])
    rotM=np.array([[np.cos(omega*dt),-np.sin(omega*dt)],
                   [np.sin(omega*dt), np.cos(omega*dt)]])
    newAngle = (omega*dt + angle)*(180/np.pi)
    newXY = rotM@(np.array([[x],[y]])-ICC)+ICC
    #newPose = np.vstack((newXY, np.array([newAngle])))
    newX = newXY[0,0]
    newY = newXY[1,0]
   
    return newX, newY, newAngle

def diffsteerm2(wr,wl,rw,W,dt,x,y,theta):
    
    angle = theta*np.pi/180 #converts to radians for calculation
    vr = wr * rw
    vl = wl * rw
    V = (vr+vl)/2
    omega = (vr-vl)/W
    newX=x+dt*V*np.cos(angle)
    newY=y+dt*V*np.sin(angle)
    newAngle = (omega*dt+angle)*(180/np.pi) #converts back to degrees to print the new pose
    #newPose = np.vstack([newX,newY,newAngle])
    
    return newX, newY, newAngle

def diffsteerm2ss(wr,wl,rw,W,dt,x,y,theta,prior_wr=0,prior_wl=0,ssl=0,ssr=0,delta=0):
    
    if (wr-prior_wr) >=0:
        sr=ssr
        vr=wr*rw*(1-sr)
    else:
        sr=ssr*-1
        vr = (wr*rw)/(1+sr)
    if (wl-prior_wl) >=0:
        sl=ssl
        vl=wl*rw*(1-sl)
    else:
        sl=ssl*-1
        vl=(wl*rw)/(1+sl)
    
    angle = theta*np.pi/180
    delta_rad = delta*np.pi/180
    Vlong = (vr+vl)/2
    omega = (vr-vl)/W  
    Vlat = np.tan(delta_rad)*Vlong
    newX=x+dt*Vlong*np.cos(angle)-dt*Vlat*np.sin(angle)
    newY=y+dt*Vlong*np.sin(angle)+dt*Vlat*np.cos(angle)
    newAngle = (omega*dt+angle)*(180/np.pi)
    #newPose = np.vstack([newX,newY,newAngle])
    
    return newX,newY,newAngle

'''
Ackermann Steer
=====================================================================================
'''

def bicycleMS4(u = np.array([[0],[1]]), q = np.array([[0],[0],[-1.57],[1],[0]]), 
               dt = 0.01, DT = 0.01, L = 2.5, ss = 0, prior_v = 0, delta1 = 0, 
               delta2 = 0, tauV = 0, tauSteer = 0, maxSteer = 90*np.pi/180):
    """Calculates the future position of a vehicle using the bicycle  model.
    
    It runs for one complete sampling interval (DT) of the speed and steering 
    controllers. It processes vehicle state for floor(DT/dt) loops.
       
    Parameters
    ----------
    u : np.array
        [[desired steering angle],
         [desired vehicle speed]]
        This is an array of inputs to the control system of the vehicle.
    q : np.array
        [[x position],
         [y position],
         [vehicle angle],
         [vehicle velocity],
         [vehicle steering angle]]
        This is an array of state variables describing the current state of the vehicle.
    dt : float
        The integration time step of the kinematic model
    DT : float
        The sampling interval of the steering and speed controller. Must be longer than dt to have an effect.
    L : float
        The wheelbase of the vehicle. The distance between the front and rear axles.
    ss : float
        The slip ratio of the rear wheel
    prior_v : float
        The prior velocity of the vehicle. Used to determine if the vehicle is braking or driving.
    delta1 : float
        The slip angle of the front wheel
    delta2 : float
        The slip angle of the rear wheel  
    tauV : float
        The time constant of the speed control subsystem
    tauSteer : float
        The time constant of the steering control subsystem
    maxSteer : float
        The maximum turning angle. It applies to left and right turning.
    
    
    Returns
    ----------
    q : np.array
        The state of the vehicle after the greater of dt or 
        DT (actually dt*floor(DT/dt)) time passes.
    """
    #Check our input. We need min and max steering angles.
    if u[0,0]>maxSteer:
        u[0,0] = maxSteer
    if u[0,0]<-1*maxSteer:
        u[0,0] = -1*maxSteer
    # BEGIN dt CONTROL LOOP HERE:
    # You should determine the number of interations and size your arrays before starting the loop.
    num_loops = int(DT/dt)+1
    newq = np.zeros((5, num_loops))  # Each column contains a different time step.
    # Column 0 of newq will be state at time 0, i.e. initial state, q.
    newq[0, 0] = q[0, 0]
    newq[1, 0] = q[1, 0]
    newq[2, 0] = q[2, 0]
    newq[3, 0] = q[3, 0]
    newq[4, 0] = q[4, 0]
    for i in range(1,num_loops):
        if newq[4,0]>maxSteer:
            newq[4,0] = maxSteer
        if newq[4,0]<-1*maxSteer:
            newq[4,0] = -1*maxSteer
        if tauV < dt:
            tauV = dt
        if tauSteer < dt:
            tauSteer = dt 
        newq[3,i] = newq[3,i-1]*(1-dt/tauV)+(dt/tauV)*u[1,0] 
        newq[4,i] = newq[4,i-1]*(1-dt/tauSteer)+(dt/tauSteer)*u[0,0] # Steering Control Equation
        # Test if we are braking or driving. If the new vecolity newq[3,0] 
        # is greater than current velocity q[3,0], we assume that we will be accelerating/driving.
        if (newq[3,i]-newq[3,i-1]) >=0: #driving 
            s = ss
            vlong = newq[3,i] * (1-s) #driving: wheel velocity
        else: #braking
            s = ss*-1
            vlong = (newq[3,i]) / (1+s) #braking: wheel velocity
        vlat = vlong*np.tan(delta2)
        newq[0,i] = newq[0,i-1]+dt*(vlong*np.cos(newq[2,i-1]) - vlat*np.sin(newq[2,i-1]))
        newq[1,i] = newq[1,i-1]+dt*(vlong*np.sin(newq[2,i-1]) + vlat*np.cos(newq[2,i-1]))
        newq[2,i] = newq[2,i-1]+(dt*(vlong*np.tan(newq[4,i-1]+delta1)/L-vlat/L))
    #END dt CONTROL LOOP HERE
    return newq

'''
Move to Pose & Point
==========================================================================================
'''

def angdiff(angle1, angle2):
    """Determines the smallest difference between two angles

    Result will be in the range [-pi, pi)

    Parameters
    ----------
    angle1 : float
        The first angle in radians
    angle2 : float
        The second angle in radians

    Returns
    ----------
    a : float
        The smallest angle between angle1 and angle2
    """
    a = angle1 - angle2
    while a < (-1 * np.pi):
        a = a + 2 * np.pi
    while a >= np.pi:
        a = a - 2 * np.pi
    return a


def moveToPose(current_x, current_y, current_angle,
               desired_x, desired_y, desired_angle,
               k_rho, k_alpha, k_beta, wheelbase):
    """Implements a simple proportional controller for Move to Point
    For stability with Ackermann style steering (bicycle model):
        k_rho > 0; k_beta < 0; k_alpha - k_rho > 0

    Parameters
    ----------
    current_x : float
        The current x position of the robot
    current_y : float
        The current y position of the robot
    current_angle : float
        The current heading angle of the robot
    desired_x : float
        The desired x position of the robot
    desired_y : float
        The desired y position of the robot
    desired_angle : float
        The desired heading angle of the robot
    k_rho : float
        The proportional gain for direction to point (velocity)
    k_alpha : float
        The proportional gain for heading to point
    k_beta : float
        The proportional gain for heading once final point is reached
    wheelbase : float
        The distance between the front and rear axles. Units should be
        consistent with x & y units.

    Returns
    ----------
    newV : float
        The newly calculated velocity for the robot
    newSteer : float
        The newly calculated steering angle for the robot in radians
    """
    rho = np.sqrt(np.float_power(desired_x - current_x, 2)
                  + np.float_power(desired_y - current_y, 2))
    alpha = (np.arctan((desired_y - current_y) / (desired_x - current_x))
             - current_angle)
    beta = -1 * current_angle - alpha + desired_angle
    newV = k_rho * rho
    omega = k_alpha * alpha + k_beta*beta
    newSteer = np.arctan((omega * wheelbase) / newV)
    return newV, newSteer


def moveToPoint(current_x, current_y, current_angle,
                desired_x, desired_y, kv, kh):
    """Implements a simple proportional controller for Move to Point

    Parameters
    ----------
    current_x : float
        The current x position of the robot
    current_y : float
        The current y position of the robot
    current_angle : float
        The current heading angle of the robot
    desired_x : float
        The desired x position of the robot
    desired_y : float
        The desired y position of the robot
    kv : float
        The proportional gain for velocity
    kh : float
        The proportional gain for heading/steering angle, kh > 0.

    Returns
    ----------
    newV : float
        The newly calculated velocity for the robot
    newSteer : float
        The newly calculated steering angle for the robot
    """
    newV = kv * np.sqrt(np.float_power(desired_x - current_x, 2)
                        + np.float_power(desired_y - current_y, 2))
    desired_angle = np.arctan((desired_y - current_y) /
                              (desired_x - current_x))
    newSteer = kh * angdiff(desired_angle, current_angle)
    return newV, newSteer

def moveToPoseConstV(current_x, current_y, current_angle, current_V,
                     desired_x, desired_y, desired_angle,
                     k_alpha, k_beta, wheelbase):
    """Implements a simple proportional controller for Move to Pose
    For stability with Ackermann style steering (bicycle model):
        k_beta < 0; k_alpha  > 0?
    This version of Move to Pose only adjusts steering. Velocity is assumed
    constant. Calculation of the steering angle depends on Velocity, so it
    must be provided as an input to the function.

    Parameters
    ----------
    current_x : float
        The current x position of the robot
    current_y : float
        The current y position of the robot
    current_angle : float
        The current heading angle of the robot
    current_V : float
        The current velocity of the robot
    desired_x : float
        The desired x position of the robot
    desired_y : float
        The desired y position of the robot
    desired_angle : float
        The desired heading angle of the robot
    k_alpha : float
        The proportional gain for heading to point
    k_beta : float
        The proportional gain for heading once final point is reached
    wheelbase : float
        The distance between the front and rear axles. Units should be
        consistent with x & y units.

    Returns
    ----------
    newSteer : float
        The newly calculated steering angle for the robot in radians
    """

    alpha = (np.arctan2((desired_y - current_y), (desired_x - current_x))
             - current_angle)
    beta = -1 * current_angle - alpha + desired_angle
    omega = k_alpha * alpha + k_beta*beta
    newSteer = np.arctan((omega * wheelbase) / current_V)
    return newSteer


def moveToPointConstV(current_x, current_y, current_angle,
                      desired_x, desired_y, kh):
    """Implements a simple proportional controller for Move to Point

    Parameters
    ----------
    current_x : float
        The current x position of the robot
    current_y : float
        The current y position of the robot
    current_angle : float
        The current heading angle of the robot
    desired_x : float
        The desired x position of the robot
    desired_y : float
        The desired y position of the robot
    kh : float
        The proportional gain for heading/steering angle, kh > 0.

    Returns
    ----------
    newSteer : float
        The newly calculated steering angle for the robot
    """
    desired_angle = np.arctan2((desired_y - current_y),
                               (desired_x - current_x))
    newSteer = kh * angdiff(desired_angle, current_angle)
    return newSteer

'''
Path Following: Pure Pursuit & Manuvers
==================================================================================
'''

def purePursuitController(q=np.array([[0], [0], [-1.57], [1], [0]]),
                          L=2.5, ld=10,
                          path=np.array((np.linspace(0, 10, 11),
                                         np.linspace(0, 10, 11)))):
    """Runs a pure pursuit controller to follow a path.

    Parameters
    ----------
    q : np.array
        [[x position],
         [y position],
         [vehicle angle],
         [vehicle velocity],
         [vehicle steering angle]]
        This is an array of state variables describing the current
        state of the vehicle.
    L : float
        The wheelbase of the vehicle.
        The distance between the front and rear axles.
    ld : float
        The look ahead distance.
    path : np.array
        [[x1, x2, x3,...]
         [y1, y2, y3,...]]
        The path that the vehicle is to follow. It is defined by a set of
        x,y points.

    Returns
    ----------
    steerAngle : np.array
        The angle at which the steering wheel should be set
    distanceMin : np.array
        The cross track error (XTE) of the vehicle current position.     
    """
    robotX = q[0, 0]
    robotY = q[1, 0]
    robotAngle = q[2, 0]
    min_ld_index = 0
    min_XTE_index = 0
    # Calculate the first point and save as the initial minimum distance
    # We are calculating a minimum to ld and a minimum to robot.
    pathX = path[0, 0]
    pathY = path[1, 0]
    distanceMinld = np.abs(np.sqrt(np.float_power(robotX - pathX, 2)
                                   + np.float_power(robotY - pathY, 2)) - ld)
    distanceMin = np.sqrt(np.float_power(robotX - pathX, 2)
                          + np.float_power(robotY - pathY, 2))
    path_len = path.shape[1]
    # Check every point in the path to see which provides the minXTE and which
    # is closest to ld from the robot.
    for i in range(path_len):
        pathX = path[0, i]
        pathY = path[1, i]
        distance = np.sqrt(np.float_power(robotX - pathX, 2)
                           + np.float_power(robotY - pathY, 2))
        if (distance < distanceMin):
            min_XTE_index = i
            distanceMin = distance
        if (np.abs(distance - ld) < distanceMinld):
            min_ld_index = i
            distanceMinld = np.abs(distance - ld)

    # To calculate ey, express our path in the robot frame
    fShiftAngle = -1 * robotAngle
    fShiftX = -1 * robotX
    fShiftY = -1 * robotY
    matTRbad = np.array([[np.cos(fShiftAngle), -1 * np.sin(fShiftAngle), fShiftX],
                         [np.sin(fShiftAngle), np.cos(fShiftAngle), fShiftY],
                         [0, 0, 1]])
    matT = np.array([[fShiftX],
                     [fShiftY]])
    matR = np.array([[np.cos(fShiftAngle), -1 * np.sin(fShiftAngle)],
                     [np.sin(fShiftAngle), np.cos(fShiftAngle)]])
    path_robot_framebad = matTRbad @ np.vstack((path, np.ones((1, path.shape[1]))))
    path_robot_frameT = matT + path
    path_robot_frameTR = matR @ path_robot_frameT
    ey = path_robot_frameTR[1, min_ld_index]
    steerAngle = np.arctan((2*ey*L)/(np.float_power(ld, 2)))
    return steerAngle, distanceMin

def piTurn(r_min, w_row, rows):
    """Determines a path (set of points) representing a pi turn.

    The resulting path starts at 0,0 with a angle of 0 deg. (pose = 0,0,0). It
    will turn left or right depending on if rows is positive (right turn) or
    negative (left turn). Path should be translated and rotated to its proper
    position in the field by the calling function.

    Parameters
    ----------
    r_min : float
        Turning radius of the vehicle.
    w_row : float
        The width of a row in the field.
    rows : int
        The number of rows between the current row and the target row
        e.g. Vehicle is turning from the mid-point of row i
             into the mid-point of row i+N

    Returns
    ----------
    path : np.array
        [[x1, x2, x3,...]
         [y1, y2, y3,...]]
        The path that the vehicle is to follow. It is defined by a set of
        x,y points.
    distance : float
        The length of the path that accomplishes the requested pi-turn.
    """
    # First check if a pi turn is possible
    if rows * np.abs(w_row) < 2 * r_min:
        path = np.zeros((0, 0))  # Turn is not possible. Path is empty
        distance = np.nan  # Distance cannot be calculated
        return (path, distance)

    d = rows * w_row  # distance from start path to end path
    if d > 0:  # Turn to the right
        # Create the starting arc for leaving
        # the initial path (60 points+endpoint)
        a = np.linspace(-np.pi/2, 0, 61)
        x_start = 0 + r_min * np.cos(a)
        y_start = -1 * r_min - r_min * np.sin(a)
        # Create the final arc for entering
        # the target path (60 points+endpoint)
        a = np.linspace(0, np.pi/2, 61)
        x_end = 0 + r_min * np.cos(a)
        y_end = -1 * rows * w_row + r_min - r_min * np.sin(a)
        # Create straight section if necessary
        if rows * w_row == 2 * r_min:  # no straight section. Connect arcs
            # The first point in x_end repeats x_start.
            # Same for y_end and y_start
            x = np.hstack((x_start, x_end[1:]))
            y = np.hstack((y_start, y_end[1:]))
            path = np.array((x, y))
        else:
            # Create straight section
            x_straight = np.linspace(x_start[-1], x_end[0], 61)
            y_straight = np.linspace(y_start[-1], y_end[0], 61)
            # Connect segments. Once again each segment repeats the start
            # and end.
            x = np.hstack((x_start, x_straight[1:], x_end[1:]))
            y = np.hstack((y_start, y_straight[1:], y_end[1:]))
    else:
        # Create the starting arc for leaving
        # the initial path (60 points+endpoint)
        a = np.linspace(np.pi/2, 0, 61)
        x_start = 0 + r_min * np.cos(a)
        y_start = r_min - r_min * np.sin(a)
        # Create the final arc for entering
        # the target path (60 points+endpoint)
        a = np.linspace(0, -1 * np.pi/2, 61)
        x_end = 0 + r_min * np.cos(a)
        y_end = -rows * w_row - r_min - r_min * np.sin(a)
        # Create straight section if necessary
        if rows * w_row == 2 * r_min:  # no straight section. Connect arcs
            # The first point in x_end repeats x_start.
            # Same for y_end and y_start
            x = np.hstack((x_start, x_end[1:]))
            y = np.hstack((y_start, y_end[1:]))
            path = np.array((x, y))
        else:
            # Create straight section
            x_straight = np.linspace(x_start[-1], x_end[0], 61)
            y_straight = np.linspace(y_start[-1], y_end[0], 61)
            # Connect segments. Once again each segment repeats the start
            # and end.
            x = np.hstack((x_start, x_straight[1:], x_end[1:]))
            y = np.hstack((y_start, y_straight[1:], y_end[1:]))
    path = np.array((x, y))
    distance = rows * w_row + (np.pi - 2) * r_min
    return path, distance


def omegaTurn(r_min, w_row, rows):
    """Determines a path (set of points) representing a omega turn.

    The resulting path starts at 0,0 with a angle of 0 deg. (pose = 0,0,0). It
    will turn left or right depending on if rows is positive (right turn) or
    negative (left turn). Path should be translated and rotated to its proper
    position in the field by the calling function.

    Parameters
    ----------
    r_min : float
        Turning radius of the vehicle.
    w_row : float
        The width of a row in the field.
    rows : int
        The number of rows between the current row and the target row
        e.g. Vehicle is turning from the mid-point of row i
             into the mid-point of row i+N

    Returns
    ----------
    path : np.array
        [[x1, x2, x3,...]
         [y1, y2, y3,...]]
        The path that the vehicle is to follow. It is defined by a set of
        x,y points.
    distance : float
        The length of the path that accomplishes the requested pi-turn.
    """
    # First check if a omega turn is possible
    d = rows * w_row  # distance from start path to end path
    if rows * w_row > 2 * r_min:
        path = np.zeros((0, 0))  # Turn is not possible. Path is empty
        distance = np.nan  # Distance cannot be calculated
        return (path, distance)

    if d > 0:  # Turn to the right
        # Create the starting arc for leaving the  path (60 points+endpoint)
        # Arc starts at pi/2 and rotates up/back toward 0, angle will be alpha
        alpha = np.arccos((r_min + d / 2) / (2 * r_min))
        a = np.linspace(np.pi / 2, np.pi / 2 - alpha, 61)
        x_start = 0 + r_min * np.cos(a)
        y_start = r_min - r_min * np.sin(a)
        # Create the final arc for entering the  path (60 points+endpoint)
        a = np.linspace(-1 * np.pi / 2 + alpha, -1 * np.pi/2, 61)
        x_end = 0 + r_min * np.cos(a)
        y_end = -1 * d - r_min - r_min * np.sin(a)
        # Create bulb section
        bulb_center_x = 2 * r_min * np.sqrt(1 -
                                            np.float_power((r_min + d / 2) /
                                                           (2 * r_min), 2))
        bulb_center_y = -1 * d / 2
        a = np.linspace(-1 * np.pi/2 - alpha, np.pi / 2 + alpha, 61)
        x_bulb = bulb_center_x + r_min * np.cos(a)
        y_bulb = bulb_center_y - r_min * np.sin(a)
    else:
        # Create the starting arc for leaving the path (60 points+endpoint)
        d = d * -1
        # Arc starts at pi/2 and rotates up/back toward 0, angle will be alpha
        alpha = np.arccos((r_min + d / 2) / (2 * r_min))
        a = np.linspace(-1 * np.pi/2, -1 * np.pi / 2 + alpha, 61)
        x_start = 0 + r_min * np.cos(a)
        y_start = -1 * r_min - r_min * np.sin(a)
        # Create the final arc for entering the path (60 points+endpoint)
        a = np.linspace(np.pi / 2 - alpha, np.pi / 2, 61)
        x_end = 0 + r_min * np.cos(a)
        y_end = d + r_min - r_min * np.sin(a)
        # Create bulb section
        bulb_center_x = 2 * r_min * np.sqrt(1 -
                                            np.float_power((r_min + d / 2) /
                                                           (2 * r_min), 2))
        bulb_center_y = d / 2
        a = np.linspace(np.pi / 2 + alpha, -1 * np.pi/2 - alpha, 61)
        x_bulb = bulb_center_x + r_min * np.cos(a)
        y_bulb = bulb_center_y - r_min * np.sin(a)
    # Connect segments. Each segment repeats the start and end.
    x = np.hstack((x_start, x_bulb[1:], x_end[1:]))
    y = np.hstack((y_start, y_bulb[1:], y_end[1:]))
    path = np.array((x, y))
    distance = (4 * alpha + np.pi) * r_min
    return path, distance


def dist(x1, y1, x2, y2):
    d = np.sqrt(np.float_power(x1 - x2, 2)
                + np.float_power(y1 - y2, 2))
    return d

'''
Area Coverage
================================================================================
'''

def pathsToTSP(paths):
    """Converts paths into TSP nodes and creates a distance matrix

    This function takes the paths created in the fieldToPaths function
    and creates nodes that describe the endpoints of the paths.

    Parameters
    ----------
    paths : np.array
        [[[line0_x0, line0_x1],
          [line0_y0, line0_y1]],
         [[line1_x0, line1_x1],
          [line1_y0, line1_y1]],
        ...                    ]
        The paths that the vehicle is to follow. It is defined by a sets of
        x,y points representing the endpoints of the paths. It is a three
        dimensional array with the first dimension representing the path
        and then a 2D array of points below that.

    Returns
    ----------
    tsp_nodes: np.array
        [[x0, x1, x2, x3,...],
         [y0, y1, y2, y3,...]]

         The points that represent the nodes for the TSP problem.
         The nodes are derived from endpoints in the paths array.
    distance: np.array
        [[d00, d10, d20, d30,...],
         [d01, d11, d21, d31,...]
         ...]
        A distance matrix of the distances (d_xy) between node x and node y.
         """

    tsp_nodes = np.array([np.hstack(paths[:, 0, :]),
                          np.hstack(paths[:, 1, :])])

    distances = np.zeros((np.shape(tsp_nodes)[1], np.shape(tsp_nodes)[1]))
    for node1 in range(0, np.shape(tsp_nodes)[1]):
        for node2 in range(node1, np.shape(tsp_nodes)[1]):
            truth_arr = np.zeros(np.shape(paths)[0])  #<----- beginning of modifications
            for i in range(0,np.shape(paths)[0]):
                if (tsp_nodes[0][node1] == paths[i,0,0] and tsp_nodes[1][node1] == paths[i,1,0]) or (
                    tsp_nodes[0][node1] == paths[i,0,1] and tsp_nodes[1][node1] == paths[i,1,1]):
                    node1path = paths[i]
                else: 
                    node1path = 0 # arbitrary value different from node2path
                if (tsp_nodes[0][node2] == paths[i,0,0] and tsp_nodes[1][node2] == paths[i,1,0]) or (
                    tsp_nodes[0][node2] == paths[i,0,1] and tsp_nodes[1][node2] == paths[i,1,1]):
                    node2path = paths[i]
                else: 
                    node2path = 1 # arbitrary value different from node1path
                truth_arr[i] = np.array_equal(node1path,node2path)
            
            if np.any(truth_arr) == True:
                dist = 0
            else: 
                dist = np.sqrt((tsp_nodes[0][node1] -
                                  tsp_nodes[0][node2]) ** 2 +
                                 (tsp_nodes[1][node1] -
                                  tsp_nodes[1][node2]) ** 2)   #<----- end of modifications
            distances[node1, node2] = dist
            distances[node2, node1] = dist
    return tsp_nodes, distances


def tspToSolution1(nodes, cost_mat):
    """Creates a solution to the TSP problem.
    Converts nodes to Cities and uses function from TSP notebook
    Cities has now been redefined as its own type of class instead of a
    complex point. It includes a varible to record its node number so that it
    can be referenced later.

    Parameters
    ----------
    nodes : np.array
    [[x0, x1, x2...]
     [y0, y1, y2...]]


    Returns
    ----------
    tour : list of Cities
    """
    # define neccessary functions from TSP notebook
    def cost(A, B):
        return cost_mat[A.num, B.num]

    def shortest_edges_first(cities):
        # Return all edges between distinct cities, sorted shortest first."
        edges = [(A, B) for A in cities for B in cities
                 if id(A) < id(B)]
        return sorted(edges, key=lambda edge: cost(*edge))

    def join_endpoints(endpoints, A, B):
        # Join B's segment onto the end of A's and return the segment.
        # Maintain endpoints dict."
        Asegment, Bsegment = endpoints[A], endpoints[B]
        if Asegment[-1] is not A:
            Asegment.reverse()
        if Bsegment[0] is not B:
            Bsegment.reverse()
        Asegment.extend(Bsegment)
        del endpoints[A], endpoints[B]  # A and B are no longer endpoints
        endpoints[Asegment[0]] = endpoints[Asegment[-1]] = Asegment
        return Asegment

    def greedy_tsp(cities):
        """Go through edges, shortest first.
        Use edge to join segments if possible."""
        endpoints = {c: [c] for c in cities}
        for (A, B) in shortest_edges_first(cities):
            if (A in endpoints and B in endpoints and
                    endpoints[A] != endpoints[B]):
                new_segment = join_endpoints(endpoints, A, B)
                if len(new_segment) == len(cities):
                    return new_segment

    # start of additional code

    # converting nodes into a list of cities
    class Node():
        def __init__(self, x, y, num):
            self.x = x
            self.y = y
            self.num = num

    City = Node
    cities = [City(nodes[0, i], nodes[1, i], i) for i in range(nodes.shape[1])]

    # apply greedy algorithm
    tour = greedy_tsp(cities)

    return tour


def routeToField(tour, start_point):
    """Converts a TSP route of nodes into waypoints to follow in a field.
    Returns a list of the points to go to.

    Parameters
    ----------
    tour : list of Cities
    [Node3 (3, 7), Node6 (7, 5), ...]

    start_point: [x0, y0]


    Returns
    ----------
    waypoint_list : np.array
    [[x0, x1, x2...]
     [y0, y1, y2...]]
     """

    for count, node in enumerate(tour):
        if(node.x == start_point[0] and node.y == start_point[1]):
            start = node
            start_pos = count
    waypoint = [[], []]
    for count, node in enumerate(tour):
        xi = int(tour[(start_pos + count) % len(tour)].x)
        yi = int(tour[(start_pos + count) % len(tour)].y)
        point = [[xi], [yi]]
        waypoint = np.hstack((waypoint, point))
    waypoint = np.hstack((waypoint, [[start.x], [start.y]]))
    return waypoint
