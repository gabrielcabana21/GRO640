#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  2 11:17:40 2021

@author: alex
"""


import numpy as np
import scipy.signal as signal
import matplotlib
import matplotlib.pyplot as plt

from pyro.dynamic            import ContinuousDynamicSystem
from pyro.dynamic.statespace import StateSpaceSystem

# Embed font type in PDF
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype']  = 42

#################################################################
##################     In CONSTRUCTION!!!                ########
#################################################################


###############################################################################
class TransferFunction( ContinuousDynamicSystem ):
    """Time-invariant transfer function representation of a dynamic system

    Y(s) / U(s) =   [ num ] / [den]

    """
    ############################################
    def __init__(self, num, den):
        
        self.num = num
        self.den = den

        n = den.shape[0]
        
        ContinuousDynamicSystem.__init__(self, n, 1, 1)
        
        # Plot params
        self.figsize = (5,3)
        self.dpi = 300
        self.fontsize = 5
        
        
    #############################################
    def f(self, x, u, t):

        #TODO

        return dx
    
    #############################################
    def h(self, x, u, t):
        
        #TODO
        
        return y
        
    
    ############################################
    def bode_plot(self):
        """ Plot frequency response """
        
        TF = signal.TransferFunction(self.num, self.den)
        
        w, mag, phase = TF.bode()
        
        fig , plots = plt.subplots(2, sharex=True, figsize=self.figsize, 
                                  dpi=self.dpi, frameon=True)
        
        plots[0].semilogx(w, mag)
        plots[1].semilogx(w, phase)
        
        plots[0].set_ylabel(self.output_label[0] + self.output_units[0] +'\n------------\n'+ self.input_label[0] + self.input_units[0]
                 , fontsize= self.fontsize )
        plots[1].set_ylabel( 'Phase [rad]', fontsize= self.fontsize )
        plots[1].set_xlabel( 'Freq [rad/sec]', fontsize= self.fontsize )
        
        for i in [0,1]:
            plots[i].grid(True)
            plots[i].tick_params( labelsize = self.fontsize )
        
        fig.canvas.set_window_title('Bode plot of ' + self.name)
        
        plt.show()
        




#################################################################
def linearize(sys, epsilon_x, epsilon_u=None):
    """Generate linear state-space model by linearizing any system.

    The system to be linearized is assumed to be time-invariant.

    Parameters
    ----------
    sys : `pyro.dynamic.ContinuousDynamicSystem`
        The system to linearize
    xbar : array_like
        State array arround which the system will be linearized
    epsilon : float
        Step size to use for numerical gradient approximation

    Returns
    -------
    instance of `StateSpaceSystem`

    """
        

'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    """ MAIN TEST """
    
    num = np.array([1])
    den = np.array([1,0,1])
    
    #TF = signal.TransferFunction( num, den)
    
    sys = TransferFunction( num , den)
    
    sys.bode_plot()