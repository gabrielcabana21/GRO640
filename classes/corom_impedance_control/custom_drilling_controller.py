#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 14 23:19:16 2020
@author: alex
------------------------------------
Fichier d'amorce pour le controleur de drillage
"""

import numpy as np

from pyro.control.robotcontrollers import EndEffectorPID
from pyro.control.robotcontrollers import EndEffectorKinematicController


class CustomDrillingController( EndEffectorPID ) :
    """ 

    """
    
    ############################
    def __init__(self, robot_model ):
        """ """
        
        EndEffectorPID.__init__( self , robot_model )
        
        # Label
        self.name = 'Custom Drilling Controller'
        
        """
        ###################################################
        # Vos paramètres de loi de commande ici !!
        ###################################################
        """
        
        # Target effector force
        self.rbar = np.array([0,0,0]) 
        
    
    #############################
    def c( self , y , r , t = 0 ):

        f_e = r  # Reference de force cartésienne cible
        x   = y  # Feedback from sensors (full state feedback)
        
        [ q , dq ] = self.x2q( x ) # extraction of joint position and speed
        
        """
        ##################################
        
        
        INPUTS
        f_e  : target force vector     3 x 1
        q    : joint positions vector  3 x 1
        dq   : joint velocity vector   3 x 1
        
        OUPUTS
        tau  : joint torque command    3 x 1
        
        
        # Votre loi de commande ci-dessous !!!
        
        ##################################
        """
        
        
        tau = np.zeros(self.m)  # place-holder de bonne dimension
        
        return tau
        