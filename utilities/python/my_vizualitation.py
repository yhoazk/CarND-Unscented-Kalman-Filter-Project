#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from matplotlib.legend_handler import HandlerLine2D

data = open("../../build/log.log", "r")
lines = data.readlines()
data.close()
values = {}
names = lines.pop(0).strip().split("\t")
for key in names:
    values[key] = []

for line in lines:
    #(px,py,v,yaw_angle,yaw_rate,px_measured,py_measured, px_true,py_true,vx_true,vy_true,NIS) = map(float, line.split("\t"))
    vals = list(map(float, line.strip().split("\t")))
    for x,k in zip(vals,names):
        values[k].append(x)




plt.grid('on')
kf_out  = plt.scatter(values['px'], values['py'], marker='x')
meas    = plt.scatter(values['px_measured'], values['py_measured'], color='green')
truth   = plt.scatter(values['px_true'], values['py_true'], color='red')


plt.legend((kf_out, meas, truth),
           ('UKF Out', 'Measured', 'true'),
           scatterpoints=1,
           loc='lower left',
           ncol=3,
           fontsize=8)



plt.show()



plt.plot(values['NIS'])
plt.axhline(7.8, linewidth=4, color='r')

plt.show()

'''
import plotly.offline as py
from plotly.graph_objs import *
import pandas as pd
import math
my_cols=['px_est','py_est','vx_est','vy_est','px_meas','py_meas','px_gt','py_gt','vx_gt','vy_gt']
with open('/home/porko/workspace/nd_autocar/CarND-Unscented-Kalman-Filter-Project/build/log.log') as f:
    table_ekf_output = pd.read_table(f, sep='\t', header=None, names=my_cols, lineterminator='\n')
'''
