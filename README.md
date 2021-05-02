# Python Robotics

A object-based toolbox for robot dynamic simulation, analysis, control and planning.

# Installation #

## Dependencies ###
* numpy
* scipy
* matplotlib

## Recommanded environment ##
Anaconda distribution + spyder IDE available here: https://www.anaconda.com/products/individual

Note: If graphical animations are note working, try changing the graphics backend. In spyder this option is found in the menu at python/Preferences/IPython console/Backend. Inline does not allow animations, it is best to use Automatic of OS X (for Mac).

### Method 1: Clone repo and add to python path ###

A simple option for development is simply to clone the repo:
```bash
git clone https://github.com/SherbyRobotics/pyro.git
```
then add the pyro folder to the pythonpath variable of your environment. In spyder this option is found in the menu at python/PYTHONPATH manager.

### Method 2: Clone repo and install ###

Clone the pyro repo from git, and install the pyro library to your python
environment:

```bash

git clone https://github.com/SherbyRobotics/pyro.git
cd pyro
python setup.py install
```
or to install in develop mode (having the `pyro` module automatically updated as you edit the code in this
repository) use:

```bash

git clone https://github.com/SherbyRobotics/pyro.git
cd pyro
python setup.py develop
```

### Method 3: Using PIP ###

To install the package, use: 
```bash

pip install git+https://github.com/SherbyRobotics/pyro
```

It will install the files from the project. It will also check and install the required dependencies. Warning: don't use this option if you are using an Anaconda distribution (conflits with pip).

# Usage #






