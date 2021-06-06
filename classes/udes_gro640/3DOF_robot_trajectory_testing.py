#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May  1 19:51:49 2020

@author: alex
"""
import numpy as np
from matplotlib import pyplot as plt 

import sys
sys.path.append("classes/udes_gro640/")

from gro640_robots import DrillingRobot
from cabg2101      import goal2r, r2q


sys = DrillingRobot()

t_f = 3
n  = 1000
# sys.x0 = np.array([0,0,0,0,0,0])
sys.compute_trajectory(t_f,n)

##################################################
# Overwrite trajectory with your q(t) data
sys.traj.t      = np.linspace(0,t_f, n)    # t
sys.traj.x[:,0] = np.linspace(0,1, n)     # q1(t)
sys.traj.x[:,1] = np.linspace(1.4,2, n)   # q2(t)
sys.traj.x[:,2] = np.linspace(-1.3,2, n)  # q3(t)
##################################################

# Compute trajectories
r, dr, d2r = goal2r(sys.forward_kinematic_effector(sys.x0), np.array([-0.5,-0.5,0]), t_f)
q, dq, d2q = r2q(r, dr, d2r, sys)

# Assign trajectories to each joint
q = q.T
sys.traj.x[:,0] = q[:,0]
sys.traj.x[:,1] = q[:,1]
sys.traj.x[:,2] = q[:,2]

# Plot angular position, speed, and acceleration
simfig, plots = plt.subplots(3)
plots[0].plot(np.linspace(0,t_f,1000), q[:,0])
plots[0].plot(np.linspace(0,t_f,1000), q[:,1])
plots[0].plot(np.linspace(0,t_f,1000), q[:,2])
plots[1].plot(np.linspace(0,t_f,1000), dq[0,:])
plots[1].plot(np.linspace(0,t_f,1000), dq[1,:])
plots[1].plot(np.linspace(0,t_f,1000), dq[2,:])
plots[2].plot(np.linspace(0,t_f,1000), d2q[0,:])
plots[2].plot(np.linspace(0,t_f,1000), d2q[1,:])
plots[2].plot(np.linspace(0,t_f,1000), d2q[2,:])
plt.show()

# Print initial and final positions
r_0_desired = list(r[:,0])
r_f_desired = list(r[:,-1])
r_0_actual  = list(sys.forward_kinematic_effector(q[0,:]))
r_f_actual  = list(sys.forward_kinematic_effector(q[-1,:]))
print("Initial position goal2r():")
print(f"x:\t{r_0_desired[0]:.4f} m\ny:\t{r_0_desired[1]:.4f} m\nz:\t{r_0_desired[2]:.4f} m\n")
print("Initial position r2q():")
print(f"x:\t{r_0_actual[0]:.4f} m\ny:\t{r_0_actual[1]:.4f} m\nz:\t{r_0_actual[2]:.4f} m\n")
print("Final position goal2r():")
print(f"x:\t{r_f_desired[0]:.4f} m\ny:\t{r_f_desired[1]:.4f} m\nz:\t{r_f_desired[2]:.4f} m\n")
print("Final position r2q():")
print(f"x:\t{r_f_actual[0]:.4f} m\ny:\t{r_f_actual[1]:.4f} m\nz:\t{r_f_actual[2]:.4f} m")

# Visualise trajectory with animation
sys.animate_simulation(is_3d=True)
plt.plot(r[0,:], r[1,:], r[2,:])