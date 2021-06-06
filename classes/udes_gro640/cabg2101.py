#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 14 23:19:16 2020
Last updated on Tue May 11 HH:MM:SS 2020

@authors:
    Girard, Alexandre | gira2403
    Cabana, Gabriel   | cabg2101
------------------------------------


Fichier d'amorce pour les livrables de la problématique GRO640'


"""

import numpy as np

from matplotlib import pyplot as plt 
from mpl_toolkits import mplot3d

from datetime import datetime

from pyro.control.robotcontrollers import EndEffectorPID
from pyro.control.robotcontrollers import EndEffectorKinematicController


##################################################
### PART 1                                     ###
##################################################

def dh2T(r, d, theta, alpha) -> np.ndarray:
    """
    Computes the local transformation matrix for classic DH parameters.

    :param r: DH parameter scalar (translation in x)
    :type r: float, int
    :param d: DH parameter scalar (translation in z)
    :type d: float, int
    :param theta: DH parameter scalar (rotation in z)
    :type theta: float, int
    :param alpha: DH parameter scalar (rotation in x)
    :type alpha: float, int

    :rtype: numpy.ndarray
    :return: Local transformation matrix

    """
    
    T = np.zeros((4,4))
    
    ##################################################
    ### Votre code ici !!!                         ###
    ##################################################
    
    # Sine and cosine operations
    crx = np.cos(alpha)
    srx = np.sin(alpha)
    crz = np.cos(theta)
    srz = np.sin(theta)
    
    # Transformation matrix for classic DH parameters 
    T = np.array(([crz, -srz*crx,  srz*srx, r*crz],
                  [srz,  crz*crx, -crz*srx, r*srz],
                  [  0,      srx,      crx,     d],
                  [  0,        0,        0,     1]))
    
    return T


def dhs2T(r, d, theta, alpha) -> np.ndarray:
    """
    Computes the global transformation matrix for classic DH parameters.

    :param r: DH parameter vector (translation in x)
    :type r: list of floats or numpy.ndarray of floats
    :param d: DH parameter vector (translation in z)
    :type d: list of floats or numpy.ndarray of floats
    :param theta: DH parameter vector (rotation in z)
    :type theta: list of floats or numpy.ndarray of floats
    :param alpha: DH parameter vector (rotation in x)
    :type alpha: list of floats or numpy.ndarray of floats

    :raises Exception: if DH parameter vectors differ in length
    
    :rtype: numpy.ndarray
    :return: Global transformation matrix
    
    """
    
    WTT = np.zeros((4,4))
    
    ##################################################
    ### Votre code ici !!!                         ###
    ##################################################
    
    # Verify length of inputs ('!=' for values, 'is not' for objects)
    # Python being a vocal proponent of duck typing, and this code being in a 
    # custom library, we will not verify type. It will hurt code reuse and
    # performance.
    if any(len(r) != value for value in [len(d), len(theta), len(alpha)]):
        raise Exception(f"DH parameter vectors differ in length.\n" \
                        f"\t\t   len(r):\t\t{len(r)}\n" \
                        f"\t\t   len(d):\t\t{len(d)}\n" \
                        f"\t\t   len(theta):\t{len(theta)}\n" \
                        f"\t\t   len(alpha):\t{len(alpha)}")
    
    # Initialize local transformation matrix
    T = np.zeros((4,4))
    
    # Find transformation matrix for every set of DH parameters and compute
    # global transformation matrix.
    for i in range(len(r)):
        T = dh2T(r[i], d[i], theta[i], alpha[i])
        
        # Initialize global transformation matrix
        if i == 0:
            WTT = T
            
        # Multiply global transformation matrix with latest local one
        else:
            WTT = WTT @ T
            
    return WTT


def f(q) -> np.ndarray:
    """
    Computes the effector position for given joint coordinates. Only works
    for one specific set of DH parameters.
    
    :param q: Joint space coordinates
    :type q: list of floats or numpy.ndarray of floats
    
    :rtype: numpy.ndarray
    :return: Effector space coordinates

    """
    
    r = np.zeros((3,1))
    
    ##################################################
    ### Votre code ici !!!                         ###
    ##################################################
    
    # Length measurements (m)
    l1x = 0.1470
    l1y = 0.0330
    l2  = 0.1550
    l3  = 0.1350
    l4  = 0.2175
    l5  = 0.0095
    
    # Custom chart for classic DH parameters
    tz = [    l1x,            0,    0,            0,    -l4,         0]
    rz = [   q[0], q[1]+np.pi/2, q[2], q[3]-np.pi/2,   q[4],   np.pi/2]
    tx = [    l1y,           l2,   l3,            0,      0, l5+q[5]/2]
    rx = [np.pi/2,            0,    0,      np.pi/2,  np.pi,         0]
    
    # Online chart for classic DH parameters (reference only)
    # tz = [    l1x,    0,    0,       0,    l4, 0]
    # rz = [   q[0], q[1], q[2],     q[3], q[4], 0]
    # tx = [    l1y,   l2,   l3,       0,     0, 0]
    # rx = [np.pi/2,    0,    0, np.pi/2,     0, 0]
    
    # Obtain global transformation matrix
    WTT = dhs2T(tx, tz, rz, rx)
    
    # Assign effector position (T vector)
    # WTT = [[     ][ ]]
    #       [[  R  ][T]]
    #       [[     ][ ]]
    #       [0, 0, 0, 1]
    r = np.array([WTT[0,-1], WTT[1,-1], WTT[2,-1]])
    
    return r


##################################################
### PART 2                                     ###
##################################################
    
class CustomPositionController(EndEffectorKinematicController):
    """
    Kinematic effector coordinates controller using the system Jacobian. It is
    an extended representation of the
    :class:`pyro.control.EndEffectorKinematicController` class.
    
    :param lagrange_mult: Correction factor for high speeds
    :type lagrange_mult: float, int
            
    Controller Parameter(s)
    -------------------------
    r = r_d (list, numpy.ndarray): Reference signal vector
    y = q         (numpy.ndarray): Sensor signal vector
    u = dq  (list, numpy.ndarray): Control input vector
    t                     (float): Time scalar
    
    Controller System
    -------------------------
    u = c(y, r, t)
      = (J(q)^T * J(q) + lambda * I)^(-1) * J(q)^T * [(r_d - r(q)) * k]
                     
    """
    
    ############################
    def __init__(self, manipulator):
        """ """
        
        # Include constructor of parent class
        super().__init__(manipulator, 1)
        
        # Label
        self.name = 'Custom Position Controller'
        
        ##################################################
        ### Vos paramètres de loi de commande ici !!   ###
        ##################################################
        
        # Correction factor for high speeds (helps avoid singularities)
        self.lagrange_mult = 0.1
        
    
    #############################
    def c(self, y, r, t = 0):
        """ 
        Feedback law
        
        Parameter(s)
        -------------------------
        y = q   (numpy.ndarray, float dof x 1):
            Sensor signal vector  = Joint angular positions
        r = r_d (list, numpy.ndarray, float e x 1):
            Reference signal vector  = Desired effector position
        t (float): Time scalar
        
        Return(s)
        -------------------------
        u = dq (list, numpy.ndarray, float dof x 1):
            Control inputs vector = Joint velocities
        
        """
        
        # Feedback from sensors
        q = y
        
        # Jacobian computation
        J = self.J(q)
        
        # Ref
        r_desired = r
        r_actual  = self.fwd_kin(q)
        
        # Error
        e  = r_desired - r_actual
        
        ################
        dq = np.zeros(self.m)  # place-holder de bonne dimension
        
        ##################################################
        ### Votre loi de commande ici !!!              ###
        ##################################################
        
        # dq = (J^T * J + lambda * I)^(-1) * J^T * ((r_d - r) * k)
        dq = np.dot(
            np.linalg.inv(
                J.T @ J + 
                self.lagrange_mult**2 * np.identity(self.dof)
            ) @ J.T,
            e * self.gains
        )
        
        return dq


##################################################
### PART 3                                     ###
##################################################

class CustomDrillingController(EndEffectorPID) :
    """ 
    PID in effector coordinates, using the Jacobian of the system
    ---------------------------------------
    r  : reference signal vector  e   x 1
    y  : sensor signal vector     n   x 1
    u  : control inputs vector    dof x 1
    t  : time                     1   x 1
    ---------------------------------------
    u = c(y, r, t) = (r - q) * kp + dq * kd + int(r - q) * ki

    """
    
    ############################
    def __init__(self, robot_model):
        """ """
        
        # Include constructor of parent class
        super().__init__(robot_model)
        
        # Label
        self.name = 'Custom Drilling Controller'
        
        ##################################################
        ### Vos paramètres de loi de commande ici !!   ###
        ##################################################
        
        # Target effector force
        self.rbar = np.array([0.25,0.25,0.45])
        
        # Spring, damper, force
        self.K_f = 100*np.identity(self.dof)
        self.B_f = 30*np.identity(self.dof)
        self.f_d = np.array([0,0,-200]) 
        
        # Gains
        self.gains = np.ones(self.e) * self.kp
        
        # Gravity
        self.g = robot_model.g

    #############################
    def c(self, y, r, t = 0):
        """ 
        Feedback static computation u = c(y,r,t)
        
        INPUTS
        y  : sensor signal vector     p x 1
        r  : reference signal vector  k x 1
        t  : time                     1 x 1
        
        OUPUTS
        u  : control inputs vector    m x 1
        
        """
        
        # Ref
        f_e = r
        
        # Feedback from sensors
        x = y
        [q, dq] = self.x2q(x)
        
        ##################################################
        ### Votre loi de commande ici !!!              ###
        ##################################################
        
        tau = np.zeros(self.m)  # place-holder de bonne dimension
        
        # Jacobian computation
        J = self.J(q)
        
        # Position reference
        r_desired = self.rbar
        r_actual = self.fwd_kin(q)
        
        # Error
        e  = r_desired - r_actual
        
        # Speed reference
        dr_desired = e * self.gains
        dr_actual = np.dot(J, dq)
        de = dr_desired - dr_actual
        
        # Force
        f_e = np.dot(self.K_f, e) + np.dot(self.B_f, de)
        
        # If hole was drilled (20 cm deep), then stop. Else, drill when close
        # to target in xy plane.
        if abs(r_actual[2] - 0.2) <= 0.01:
            f_e[2] = 0
        elif e[0]**2 + e[1]**2 <= 0.01**2:
            f_e[2] = self.f_d[2]   
        
        # tau = J^T * (K * (r_d - f(q)) + B*(dr_d - J(q) * dq)) + g * dq
        tau = np.dot(J.T, f_e) + self.g(q)
        
        return tau
        
    
##################################################
### PART 4                                     ###
##################################################

def goal2r(r_0, r_f, t_f):
    """
    
    Parameters
    ----------
    r_0 : numpy array float 3 x 1
        effector initial position
    r_f : numpy array float 3 x 1
        effector final position
    t_f : float
        time 

    Returns
    -------
    r   : numpy array float 3 x l
    dr  : numpy array float 3 x l
    ddr : numpy array float 3 x l

    """
    # Time discretization
    l = 1000 # nb of time steps
    
    # Number of DoF for the effector only
    m = 3
    
    r = np.zeros((m,l))
    dr = np.zeros((m,l))
    ddr = np.zeros((m,l))
    
    ##################################################
    ### Votre code ici !!!                         ###
    ##################################################
    
    # Time vector
    t = np.linspace(0, t_f, num=l)

    # Reduce number of dimensions    
    r_0 = np.squeeze(r_0)
    r_f = np.squeeze(r_f)
    
    # Distance vector
    r_e = r_f - r_0
    
    # Since the logistic curve is asymptotic, we have to define a starting and
    # end point that are close but not exactly on the initial and and final
    # position. If we add or subtract that tolerance to the position inputs,
    # then it acts more as a dampening factor for speed and acceleration.
    tol = 0.001 * np.ones(m) 
    r_0 = r_0 - tol * np.sign(r_e)
    r_f = r_f + tol * np.sign(r_e)
    
    # Distance vector
    r_e = r_f - r_0
    
    # Tolerance for end position
    tol = np.divide(tol, np.absolute(r_e), out=np.zeros_like(tol), where=r_e!=0)
    
    # Time-based analysis
    c = t_f/2*np.ones(m)
    b = np.divide(np.ones(m)-tol, tol, out=np.zeros_like(tol), where=tol!=0)
    b = np.divide(np.log(b, out=np.zeros_like(b), where=b!=0), c)
    
    for i in range(len(t)):
        # Position
        e      = np.exp(-b * (np.ones(m)*t[i]-c))
        r[:,i] = r_0 + np.divide(r_e, np.ones(m) + e)
        
        # Speed
        de      = np.divide(b * e, (np.ones(m) + e)**2)
        dr[:,i] = r_e * de
        
        # Acceleration
        d2e      = np.divide(b**2 * e**2 * (np.ones(m) - e**-1), (1 + e)**3)
        ddr[:,i] = r_e * d2e
    
    return r, dr, ddr


def r2q(r, dr, ddr, manipulator):
    """

    Parameters
    ----------
    r   : numpy array float 3 x l
    dr  : numpy array float 3 x l
    ddr : numpy array float 3 x l
    
    manipulator : pyro object 

    Returns
    -------
    q   : numpy array float 3 x l
    dq  : numpy array float 3 x l
    ddq : numpy array float 3 x l

    """
    # Time discretization
    l = r.shape[1]
    
    # Number of DoF
    n = 3
    
    # Output dimensions
    q = np.zeros((n,l))
    dq = np.zeros((n,l))
    ddq = np.zeros((n,l))
    
    ##################################################
    ### Votre code ici !!!                         ###
    ##################################################
    
    # Time interval and time step
    t_f = 3
    dt = t_f/l
    
    # Speed regulation coefficient
    lagrange_mult = 0.01
    
    # Initialization
    J = manipulator.J(manipulator.x0)
    
    dq[:,0] = np.dot(
        np.linalg.inv(
            J.T @ J + 
            lagrange_mult**2 * np.identity(n)
        ) @ J.T,
        dr[:,0]
    )
    
    q[:,0] = dq[:,0] * dt
    
    for i in range(1, l):
        J = manipulator.J(q[:,i-1])
        
        dq[:,i] = np.dot(
            np.linalg.inv(
                J.T @ J + 
                lagrange_mult**2 * np.identity(n)
            ) @ J.T,
            dr[:,i]
        )
        
        q[:,i]   = q[:,i-1] + dq[:,i] * dt
        
        ddq[:,i] = (dq[:,i] - dq[:,i-1])/dt
    
    return q, dq, ddq


def q2torque(q, dq, ddq, manipulator):
    """

    Parameters
    ----------
    q   : numpy array float 3 x l
    dq  : numpy array float 3 x l
    ddq : numpy array float 3 x l
    
    manipulator : pyro object 

    Returns
    -------
    tau   : numpy array float 3 x l

    """
    # Time discretization
    l = q.shape[1]
    
    # Number of DoF
    n = 3
    
    # Output dimensions
    tau = np.zeros((n,l))
    
    ##################################################
    ### Votre code ici !!!                         ###
    ##################################################
    
    return tau


##################################################
### MAIN                                       ###
##################################################

if __name__ == "__main__":
    
    file = open("cabg2101.log", "w")
    
    my_date = datetime.now()
    file.write(f"FILE:\tcabg2101.py\n")
    file.write(f"DATE:\t{my_date.strftime('%Y-%m-%d | %H:%M:%S')}\n\n")
    
    #------------------------------------------------#
    
    file.write(\
        f"##################################################\n" \
        f"### PART 1                                     ###\n" \
        f"##################################################\n\n")
    
    file.write(\
        f"Test #1\n"\
        f"-------------------------\n\n")
    
    q = np.array([0,0,0,0,0,0])
    r = f(q)
    
    file.write(f"Joint angles:\n" \
        f"q1:\t{q[0]:.4f} rad\nq2:\t{q[1]:.4f} rad\nq3:\t{q[2]:.4f} rad\n" \
        f"q4:\t{q[3]:.4f} rad\nq5:\t{q[4]:.4f} rad\nq6:\t{q[5]:.4f} rad\n\n" \
        f"Effector position:\n" \
        f"x:\t{r[0]:.4f} m\ny:\t{r[1]:.4f} m\nz:\t{r[2]:.4f} m\n\n")
        
    file.write(\
        f"Test #2\n"\
        f"-------------------------\n\n")
    
    q = np.array([0,-np.pi/2,np.pi/2,0,0,0])
    r = f(q)
    
    file.write(f"Joint angles:\n" \
        f"q1:\t{q[0]:.4f} rad\nq2:\t{q[1]:.4f} rad\nq3:\t{q[2]:.4f} rad\n" \
        f"q4:\t{q[3]:.4f} rad\nq5:\t{q[4]:.4f} rad\nq6:\t{q[5]:.4f} rad\n\n" \
        f"Effector position:\n" \
        f"x:\t{r[0]:.4f} m\ny:\t{r[1]:.4f} m\nz:\t{r[2]:.4f} m\n\n")
        
    file.write(\
        f"Test #3\n"\
        f"-------------------------\n\n")
    
    q = np.array([0,0,0,-np.pi/2,0,0])
    r = f(q)
    
    file.write(f"Joint angles:\n" \
        f"q1:\t{q[0]:.4f} rad\nq2:\t{q[1]:.4f} rad\nq3:\t{q[2]:.4f} rad\n" \
        f"q4:\t{q[3]:.4f} rad\nq5:\t{q[4]:.4f} rad\nq6:\t{q[5]:.4f} rad\n\n" \
        f"Effector position:\n" \
        f"x:\t{r[0]:.4f} m\ny:\t{r[1]:.4f} m\nz:\t{r[2]:.4f} m\n\n")
        
    file.write(\
        f"Test #4\n"\
        f"-------------------------\n\n")
    
    q = np.array([0,0,0,0,np.pi/2,0])
    r = f(q)
    
    file.write(f"Joint angles:\n" \
        f"q1:\t{q[0]:.4f} rad\nq2:\t{q[1]:.4f} rad\nq3:\t{q[2]:.4f} rad\n" \
        f"q4:\t{q[3]:.4f} rad\nq5:\t{q[4]:.4f} rad\nq6:\t{q[5]:.4f} rad\n\n" \
        f"Effector position:\n" \
        f"x:\t{r[0]:.4f} m\ny:\t{r[1]:.4f} m\nz:\t{r[2]:.4f} m\n\n")
        
    file.write(\
        f"Test #5\n"\
        f"-------------------------\n\n")
    
    q = np.array([0,0,0,0,0,0.1-2*0.0095])
    r = f(q)
    
    file.write(f"Joint angles:\n" \
        f"q1:\t{q[0]:.4f} rad\nq2:\t{q[1]:.4f} rad\nq3:\t{q[2]:.4f} rad\n" \
        f"q4:\t{q[3]:.4f} rad\nq5:\t{q[4]:.4f} rad\nq6:\t{q[5]:.4f} rad\n\n" \
        f"Effector position:\n" \
        f"x:\t{r[0]:.4f} m\ny:\t{r[1]:.4f} m\nz:\t{r[2]:.4f} m\n\n")
        
    #------------------------------------------------#
    
    file.write(\
        f"##################################################\n" \
        f"### PART 2                                     ###\n" \
        f"##################################################\n\n")
        
    #------------------------------------------------#
    
    file.write(\
        f"##################################################\n" \
        f"### PART 3                                     ###\n" \
        f"##################################################\n\n")
        
    #------------------------------------------------#
    
    file.write(\
        f"##################################################\n" \
        f"### PART 4                                     ###\n" \
        f"##################################################\n\n")
    
    t_f = 6
    r, dr, d2r = goal2r(np.array([0,0,0]), np.array([1,-2,3]), t_f)
    
    # 3D Plot
    # plt.xlabel("x") 
    # plt.ylabel("y")
    # ax = plt.axes(projection='3d')
    # ax.plot3D(r[0,:], r[1,:], r[2,:])
    # # ax.plot3D(r[0,:], r[1,:], np.linspace(0,10,1000), 'gray')
    # # ax.plot3D(r[0,:], np.linspace(0,10,1000), r[2,:], 'black')
    # # ax.plot3D(np.linspace(0,10,1000), r[1,:], r[2,:], 'red')
    # plt.show()
    
    # Effector position, speed, and acceleration
    simfig, plots = plt.subplots(3)
    plots[0].plot(np.linspace(0,t_f,1000), r[0,:])
    plots[0].plot(np.linspace(0,t_f,1000), r[1,:])
    plots[0].plot(np.linspace(0,t_f,1000), r[2,:])
    plots[1].plot(np.linspace(0,t_f,1000), dr[0,:])
    plots[1].plot(np.linspace(0,t_f,1000), dr[1,:])
    plots[1].plot(np.linspace(0,t_f,1000), dr[2,:])
    plots[2].plot(np.linspace(0,t_f,1000), d2r[0,:])
    plots[2].plot(np.linspace(0,t_f,1000), d2r[1,:])
    plots[2].plot(np.linspace(0,t_f,1000), d2r[2,:])
    print(r[:,-1])
    plt.show()
    
    file.close()