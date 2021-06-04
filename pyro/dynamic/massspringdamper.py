# -*- coding: utf-8 -*-
"""
Created on 4 Jun 2021

@author: Alexandre
"""

import numpy as np


from pyro.dynamic import statespace


###############################################################################

class SingleMass( statespace.StateSpaceSystem ):
    """Single Mass with linear spring and damper

    Attributes
    ----------

    """

    ############################
    def __init__(self, m=1, k=2, b=0.2):
        """ """

        # params
        self.m = m
        self.k = k
        self.b = b
        
        self.l1 = 2
        self.l2 = 1
        
        # Matrix ABCD
        self.compute_ABCD()
        
        # initialize standard params
        statespace.StateSpaceSystem.__init__(self, self.A, self.B, self.C, self.D)
        
        # Name and labels
        self.name = 'Linear-Spring-Damper'
        self.input_label = [ 'Force']
        self.input_units = [ '[N]']
        self.output_label = ['Position']
        self.output_units = ['[m]']
        self.state_label = [ 'Position','Velocity']
        self.state_units = [ '[m]', '[m/s]']
        
        self.linestyle = '-'
        
    
    ###########################################################################
    def compute_ABCD(self):
        """ 
        """
        self.A = np.array([ [ 0              ,              1 ], 
                            [ -self.k/self.m , -self.b/self.m ] ])
        self.B = np.array([ [ 0 ],
                            [ 1 ]])
        self.C = np.array([ [ 1 , 0 ]])
        self.D = np.array([ [ 0 ]])
                
        
    ###########################################################################
    # Graphical output
    ###########################################################################
    
    #############################
    def xut2q( self, x , u , t ):
        """ Compute configuration variables ( q vector ) """
        
        q = np.array([ x[0], u[0] ])

        return q
    
    ###########################################################################
    def forward_kinematic_domain(self, q ):
        """ 
        """
        l = self.l1 * 2
        
        domain  = [ (-l,l) , (-l,l) , (-l,l) ]#  
                
        return domain
    
    
    ###########################################################################
    def forward_kinematic_lines(self, q ):
        """ 
        Compute points p = [x;y;z] positions given config q 
        ----------------------------------------------------
        - points of interest for ploting
        
        Outpus:
        lines_pts = [] : a list of array (n_pts x 3) for each lines
        
        """
        
        lines_pts = [] # list of array (n_pts x 3) for each lines
        
        # ground line
        pts      = np.zeros(( 2 , 3 ))
        pts[0,:] = np.array([-self.l1,-self.l2,0])
        pts[1,:] = np.array([-self.l1,+self.l2,0])
        
        lines_pts.append( pts )
        
        # mass
        pts      = np.zeros(( 5 , 3 ))
        
        pts[0,:] =  np.array([q[0] - self.l2/2,+self.l2/2,0])
        pts[1,:] =  np.array([q[0] + self.l2/2,+self.l2/2,0])
        pts[2,:] =  np.array([q[0] + self.l2/2,-self.l2/2,0])
        pts[3,:] =  np.array([q[0] - self.l2/2,-self.l2/2,0])
        pts[4,:] =  pts[0,:]
        
        lines_pts.append( pts )
        
        # spring
        pts      = np.zeros(( 15 , 3 ))
        
        d = q[0] + self.l1 - self.l2/2
        h =  self.l2 / 3
        
        pts[0,:]  = np.array([d*0.00 - self.l1,0,0])
        pts[1,:]  = np.array([d*0.20 - self.l1,0,0])
        pts[2,:]  = np.array([d*0.25 - self.l1,+h,0])
        pts[3,:]  = np.array([d*0.30 - self.l1,-h,0])
        pts[4,:]  = np.array([d*0.35 - self.l1,+h,0])
        pts[5,:]  = np.array([d*0.40 - self.l1,-h,0])
        pts[6,:]  = np.array([d*0.45 - self.l1,+h,0])
        pts[7,:]  = np.array([d*0.50 - self.l1,-h,0])
        pts[8,:]  = np.array([d*0.55 - self.l1,+h,0])
        pts[9,:]  = np.array([d*0.60 - self.l1,-h,0])
        pts[10,:] = np.array([d*0.65 - self.l1,+h,0])
        pts[11,:] = np.array([d*0.70 - self.l1,-h,0])
        pts[12,:] = np.array([d*0.75 - self.l1,+h,0])
        pts[13,:] = np.array([d*0.80 - self.l1,0,0])
        pts[14,:] = np.array([d*1.00 - self.l1,0,0])
        
        lines_pts.append( pts )
        
        # force arrow
        pts      = np.zeros(( 5 , 3 ))
        
        pts[0,:] =  np.array([q[0] + self.l2/2,0,0])
        pts[1,:] =  np.array([q[0] + self.l2/2 + q[1],0,0])
        pts[2,:] =  np.array([q[0] + self.l2/2 + q[1] - self.l2/4*q[1],+self.l2/4*q[1],0])
        pts[3,:] =  np.array([q[0] + self.l2/2 + q[1],0,0])
        pts[4,:] =  np.array([q[0] + self.l2/2 + q[1] - self.l2/4*q[1],-self.l2/4*q[1],0])
        
        lines_pts.append( pts )
                
        return lines_pts
    
    
    
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    sys = SingleMass()
    
    def t2u(t):
        return np.array([t])
    
    sys.t2u = t2u
    sys.x0 = np.array([1,0])
    
    sys.plot_phase_plane()
    sys.bode()
    sys.plot_trajectory('xu')
    sys.animate_simulation()