#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

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





plt.scatter(values['px'], values['py'])
plt.scatter(values['px_true'], values['py_true'], color='red')
plt.show()
