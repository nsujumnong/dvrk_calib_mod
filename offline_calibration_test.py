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


filename = sys.argv[1]
print(filename)

time = pd.read_csv(filename,usecols=[1],skiprows=7)
#rigid body 1 (RCM)
rig1_rotx = pd.read_csv(filename,usecols=[2],skiprows=7)
rig1_roty = pd.read_csv(filename,usecols=[3],skiprows=7)
rig1_rotz = pd.read_csv(filename,usecols=[4],skiprows=7)
rig1_rotw = pd.read_csv(filename,usecols=[5],skiprows=7)
rig1_posx = pd.read_csv(filename,usecols=[6],skiprows=7)
rig1_posy = pd.read_csv(filename,usecols=[7],skiprows=7)
rig1_posz = pd.read_csv(filename,usecols=[8],skiprows=7)

#rigid body 2 (rotation)
rig2_rotx = pd.read_csv(filename,usecols=[22],skiprows=7)
rig2_roty = pd.read_csv(filename,usecols=[23],skiprows=7)
rig2_rotz = pd.read_csv(filename,usecols=[24],skiprows=7)
rig2_rotw = pd.read_csv(filename,usecols=[25],skiprows=7)
rig2_posx = pd.read_csv(filename,usecols=[26],skiprows=7)
rig2_posy = pd.read_csv(filename,usecols=[27],skiprows=7)
rig2_posz = pd.read_csv(filename,usecols=[28],skiprows=7)

#create array of data
rig1_rot = np.append(rig1_rotx,rig1_roty,1)
rig1_rot = np.append(rig1_rot,rig1_rotz,1)
rig1_rot = np.append(rig1_rot,rig1_rotw,1)

rig1_pos = np.append(rig1_posx,rig1_posy,1)
rig1_pos = np.append(rig1_pos,rig1_posz,1)

rig2_rot = np.append(rig1_rotx,rig1_roty,1)
rig2_rot = np.append(rig1_rot,rig1_rotz,1)
rig2_rot = np.append(rig1_rot,rig1_rotw,1)

rig2_pos = np.append(rig1_posx,rig1_posy,1)
rig2_pos = np.append(rig1_pos,rig1_posz,1)

arm_rot = rig1_rot
marker_data_pos = rig1_pos
marker_data_rot = rig1_rot

num = len(rig1_pos)
# print(rig1_rot)
# print(rig1_pos)
# print('1 rotation')
# print(rig1_rot[0])
# print(len(rig1_rot))

print('marker_data_pos[0]')
print(marker_data_pos[0])
#
# for i in range(len(rig1_rot)):
# 	print(rig1_rot[i])
print('rig1_pos.shape')
print(rig1_pos.shape)
print(rig1_rot.shape)
x1 = tf.transformations.quaternion_matrix(marker_data_rot[0])
# print(x1)

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

p, f = _conca(marker_data_pos, marker_data_rot)

print(p)
print(f)

a,b = np.shape(f)
# print(a)
# print(b)

limit_psm=[(-0.75, 0.75), (-0.50, 0.6)]
limit_ecm=[(-0.75, 0.75), (-0.25, 0.75)]

for i in range(a):
    for j in range(b):
        check = np.isnan(f[i,j])
        if check == True:
            print('the number at',i,j,'is nan')

f[42075,0] = (f[42072,0]+f[42078,0])/2
f[42075,1] = (f[42072,1]+f[42078,1])/2
f[42075,2] = (f[42072,2]+f[42078,2])/2
f[42076,0] = (f[42073,0]+f[42079,0])/2
f[42076,1] = (f[42073,1]+f[42079,1])/2
f[42076,2] = (f[42073,2]+f[42079,2])/2
f[42077,0] = (f[42074,0]+f[42080,0])/2
f[42077,1] = (f[42074,1]+f[42080,1])/2
f[42077,2] = (f[42074,2]+f[42080,2])/2

# print(f[42072,0],f[42072,1],f[42072,2])
# print(f[42075,0],f[42075,1],f[42075,2])
# print(f[42078,0],f[42078,1],f[42078,2])


# for i in range(len(f)):
#     print(np.isnan(f[i]))

# j = np.linalg.pinv(f)

# for i in range(len(f)):
#     print(f[i])


# Calculate transformation
bpost = np.zeros((num,6))
arm_rotations = np.zeros((num,4))

for x in range(len(marker_data_pos)):
    # p, f = _conca(marker_data_pos, marker_data_rot)
    # print(p)
    j = np.linalg.pinv(f)
    bpost[x] = np.matmul(j, -p)
    arm_rotations = arm_rot


# print(p)
# print(f)
## Run test
rate = 100
q1_num = 3
final_time = 2
tool_position = 0.15

limit_q3 = tool_position
tf = final_time
z = q1_num
z2 = rate*tf

traj_q1 = np.zeros((num, z))
traj_q2 = np.zeros((num, z2))

arm = [None]*num
# print(arm)
# for i in range(num):