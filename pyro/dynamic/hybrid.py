#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Dec  9 22:04:53 2020

@author: alex
"""

import numpy as np

from pyro.dynamic import ContinuousDynamicSystem

from pyro.analysis import simulation


###############################################################################
class SwitchedSystem(ContinuousDynamicSystem):
    """
    
    
    """
    ############################################
    def __init__(self, n, m, p):
        
        ContinuousDynamicSystem.__init__(self, n, m, p)
        
    
    #############################################
    def f(self, x, u, t):
        
        u_mode = int(u[0])
        u_cont = u[1:]
        
        ################
        if u_mode == 0:
            
            dx = u_cont
        
        ################
        else:
            
            dx = np.zeros( self.n )

        return dx
    
    
    #############################
    def compute_trajectory(
        self, tf=10, n=10001, solver='euler'):
        """ 
        Simulation of time evolution of the system
        ------------------------------------------------
        tf : final time
        n  : time steps
        """

        sim = simulation.Simulator(self, tf, n, solver)

        self.traj = sim.compute() # save the result in the instance

        return self.traj