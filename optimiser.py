#!/usr/bin/env python
# Optimiser Script: Diferential Evolution is used to tune KP and ETA for Potential Fields
# Author: Pablo Hermoso Moreno

# Import Differtial Evolution Script from SciPy Library
from scipy.optimize import differential_evolution

# Running Bash Scripts ->
from subprocess import *
import subprocess

import csv
import rospy
import math
import sys
import time

def F(x):

    global iteration

    neu_dict = {'POT_FIELDS': []}
    neu_dict['POT_FIELDS'].append(x[0])
    neu_dict['POT_FIELDS'].append(x[1])

    print neu_dict
    print "Iteration Number: ", iteration

    # Save Current Potential Fields Parameters for Current Script:
    with open("POT_FIELDS_VAL_CURRENT.txt", "w") as f:
        # Overwrite Previous Entries
        f.write("%f %f" % (x[0],x[1]))

    # Save History of Selected Potential Field Parameters:
    with open("POT_FIELD_VAL_HISTORY.txt", "a") as f:
        f.write("%f %f \n" % (x[0],x[1]))

    subprocess.call(['./bash_training.sh'])

    # Processing Error - Total Path Length:
    with open("drone.path_length.txt", "r") as f:
        error = float(f.read())

    print "Total Path Length : ", error

    # Save History of Error:
    with open("ERROR_HISTORY.txt", "a") as f:
        f.write("%f \n" % error)

    iteration += 1

    return error


def main():

    global iteration
    iteration = 1

    # Optmisation Scheme for ETA and KP:

    bounds = [(0,5), (0,0.1)]

    # Run Differential Evolution Optimisation:
    res = differential_evolution(F, bounds, maxiter=10, popsize=5,disp=True)

    print("The Outputs from Optimizer Main are: ")
    print("Solution x= ", res.x)
    print("Success bool: ", res.success)
    #print("Status: ", res.status)
    print("Message: ", res.message)
    print("Function minimum found, error = ", res.fun)
    print("Number of evaluations: ", res.nfev)
    print("Number of iterations performed: ", res.nit)



if __name__ == '__main__':
    main()
