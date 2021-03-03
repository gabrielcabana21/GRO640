# -*- coding: utf-8 -*-
"""
Created on Fri Nov 16 12:05:08 2018

@author: Alexandre
"""

#import matplotlib
#matplotlib.use('Qt4Agg')

###############################################################################
import numpy as np
###############################################################################
from pyro.analysis import costfunction
from pyro.control import controller
from pyro.dynamic import pendulum
###############################################################################


import matplotlib.pyplot as plt

from pyro.planning import discretizer, valueiteration

# Continuous dynamic system
sys = pendulum.DoublePendulum()

sys.x_ub = np.array([2,2,2,2])
sys.x_lb = np.array([-2,-2,-2,-2])

# Discrete world
grid_sys = discretizer.GridDynamicSystem( sys , ( 21, 21, 21, 21) , (3,3) , 0.05)

# Cost Function
qcf = sys.cost_function

qcf.xbar = np.array([ 0,0,0,0 ]) # target
qcf.INF  = 10000

# VI algo
vi = valueiteration.ValueIteration_ND( grid_sys , qcf )

vi.initialize()
vi.load_data('aaa')
vi.compute_steps(12,True)
#vi.load_data()
vi.assign_interpol_controller()
vi.plot_policy(0)
vi.plot_cost2go()
vi.save_data('aaa')


# Closed loop
cl_sys = vi.ctl + sys

# Simulation and animation
cl_sys.x0   = np.array([0.1,0.1,0,0])
cl_sys.compute_trajectory(2,2001,'euler')
cl_sys.plot_trajectory('xu')
cl_sys.animate_simulation()

