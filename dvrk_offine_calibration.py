#!/usr/bin/env python

from numpy import genfromtxt
import rospy
import numpy as np
import pandas as pd
#import dvrk
import geometry_msgs.msg as gm
import csv
import sys
import tf
import cloudpickle as pickle
# from pytictoc import TicToc
import time

filename = sys.argv[1]
print(filename)

#rigid body 1 (RCM)
rig1_rotx = pd.read_csv(filename,usecols=[2],skiprows=6)
rig1_roty = pd.read_csv(filename,usecols=[3],skiprows=6)
rig1_rotz = pd.read_csv(filename,usecols=[4],skiprows=6)
rig1_rotw = pd.read_csv(filename,usecols=[5],skiprows=6)
rig1_posx = pd.read_csv(filename,usecols=[6],skiprows=6)
rig1_posy = pd.read_csv(filename,usecols=[7],skiprows=6)
rig1_posz = pd.read_csv(filename,usecols=[8],skiprows=6)

#rigid body 2 (rotation)
rig2_rotx = pd.read_csv(filename,usecols=[22],skiprows=6)
rig2_roty = pd.read_csv(filename,usecols=[23],skiprows=6)
rig2_rotz = pd.read_csv(filename,usecols=[24],skiprows=6)
rig2_rotw = pd.read_csv(filename,usecols=[25],skiprows=6)
rig2_posx = pd.read_csv(filename,usecols=[26],skiprows=6)
rig2_posy = pd.read_csv(filename,usecols=[27],skiprows=6)
rig2_posz = pd.read_csv(filename,usecols=[28],skiprows=6)

#create array of data
rig1_rot = np.append(rig1_rotx,rig1_roty,1)
rig1_rot = np.append(rig1_rot,rig1_rotz,1)
rig1_rot = np.append(rig1_rot,rig1_rotw,1)

rig1_pos = np.append(rig1_posx,rig1_posy,1)
rig1_pos = np.append(rig1_pos,rig1_posz,1)

rig2_rot = np.append(rig2_rotx,rig2_roty,1)
rig2_rot = np.append(rig2_rot,rig2_rotz,1)
rig2_rot = np.append(rig2_rot,rig2_rotw,1)

rig2_pos = np.append(rig2_posx,rig2_posy,1)
rig2_pos = np.append(rig2_pos,rig2_posz,1)

arm_rot = rig2_rot
# num = len(rig1_pos)
num = 10000
# print(num)

marker_data_pos = np.zeros((1,3))
marker_data_rot = np.zeros((1,4))

def _interface(self):
    if not rospy.get_node_uri():
        rospy.init_node('calibrate_test', anonymous=True, log_level=rospy.WARN)
    else:
        rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')


def _conca(positions, quaternion):
        for i in range(len(quaternion)):
            x = tf.transformations.quaternion_matrix(quaternion[i])
            if i == 0:
                matrix = np.concatenate((x[0:3, 0:3], np.identity(3)), axis=1)
                array = positions[i]
            else:
                y = np.concatenate((x[0:3, 0:3], np.identity(3)), axis=1)
                matrix = np.concatenate((matrix , y), axis=0)
                array = np.append(array, positions[i])

        return array, matrix

limit_psm=[(-0.75, 0.75), (-0.50, 0.6)]
limit_ecm=[(-0.75, 0.75), (-0.25, 0.75)]

# Calculate transformation
bpost = np.zeros((num,6))
arm_rotations = np.zeros((num,4))

tic = time.clock()

for x in range(num):
    pos = np.array([[rig1_pos[x][0],rig1_pos[x][1],rig1_pos[x][2]]])
    rot = np.array([[rig1_rot[x][0],rig1_rot[x][1],rig1_rot[x][2],rig1_rot[x][3]]])
    p, f = _conca(marker_data_pos, marker_data_rot)
    j = np.linalg.pinv(f)
    bpost[x] = np.matmul(j, -p)
    arm_rotations[x] = arm_rot[x]
    marker_data_pos = np.append(marker_data_pos,pos,axis=0)
    marker_data_rot = np.append(marker_data_rot,rot,axis=0)
    print("loop:",x)

toc = time.clock()

timer = toc - tic

print("time total:", timer)
## 7000 data points take ~ 14 mins
print("marker_data_pos")
print(marker_data_pos)
print("marker_data_rot")
print(marker_data_rot)
print("bpost")
print(bpost)
print("arm_rotations")
print(arm_rotations)

print(np.shape(bpost))
print(np.shape(arm_rotations))

rows = np.zeros((num,10))
fields = ['bpost_1','bpost_2','bpost_3','bpost_4','bpost_5','bpost_6','arm_rot_1','arm_rot_2','arm_rot_3','arm_rot_4']

## Save calculated RCM position and rotation as CSV file

for i in range(num):
    data_array = np.array([[bpost[i][0],bpost[i][1],bpost[i][2],bpost[i][3],bpost[i][4],bpost[i][5],arm_rotations[i][0],arm_rotations[i][1],arm_rotations[i][2],arm_rotations[i][3]]])

    rows[i][:] = data_array

datafile = "calibrated_data_10000.csv"

with open(datafile, 'w') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(fields)
    csvwriter.writerows(rows)
