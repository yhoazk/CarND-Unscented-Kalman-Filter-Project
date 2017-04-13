#!/usr/bin/env python

"""
There are 2 possible arguments which can be tunned for the UKF

 * std_a_
 * std_yawdd_


"""


import subprocess
import os
import time
import numpy as np
import matplotlib.pyplot as plt

args = ["~/workspace/nd_autocar/CarND-Unscented-Kalman-Filter-Project/build/UnscentedKF",  " ./data/sample-laser-radar-measurement-data-1.txt ",  ".25", " ", ".07" , " 0.01 ", "log.log"]


rmse = {'px':[], 'py':[], 'vx':[], 'vy':[]}


def call_UKF(std_a,std_yawdd):
    global args
    args[2] = str(std_a)
    args[4] = str(std_yawdd)
    output = subprocess.check_output("".join(args), shell=True)
    output = (output.decode("utf-8")).split('\n')
    rmse['px'].append(float(output[1]))
    rmse['py'].append(float(output[2]))
    rmse['vx'].append(float(output[3]))
    rmse['vy'].append(float(output[4]))

def main():
    std_a_linspace = np.linspace(0, 2.5, 20)
    for v in std_a_linspace:
        call_UKF(0.92105263,1.71052632)


    #f, arr = plt.subplots(2,2)
    #for n, subplt in enumerate(arr.reshape(-1)):
        #subplt.plot(std_a_linspace, rmse.values()[n])
        #subplt.set_title("Error" )
    print(std_a_linspace)
    print(rmse['px'])
    plt.plot(std_a_linspace, rmse['px'])
    plt.show()


if __name__ == '__main__':
    main()

"""
import os
cmd = 'bin/bar --option --otheroption'
os.system(cmd) # returns the exit status
"""
