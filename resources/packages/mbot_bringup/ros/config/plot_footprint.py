#!/usr/bin/env python
'''
plot (ROBOT env)/footprint.yaml point vector with matplotlib
for visualization (debug) purposes
'''

import matplotlib.pyplot as plt
import yaml
import sys
import os

robot = os.environ['ROBOT']
if robot == "": # check if ROBOT env is set
    print "To use this script you must set ROBOT environment variable !"
    print "Suggestion: either perform - > export ROBOT=mbot05 or edit your bashrc"
else:
    print "plotting " + str(robot) + " footprint"
    # load footprint file according to received console argument
    with open(str(robot + "/robot_footprint.yaml"), 'r') as stream:
        try:
            footprint = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    # access the element with the name 'footprint' from the footprint dictionary
    footprint_list = footprint['footprint']

    # iterate over the footprint_list and plot its elements
    plt.plot([item[0] for item in footprint_list],[item[1] for item in footprint_list])

    # set the plot title, axis labels
    plt.xlabel('distance [m]')
    plt.ylabel('distance [m]')
    plt.title('robot footprint')

    # show the plot
    plt.show()
