import os
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches
import pandas as pd
import argparse
import glob
import pylab

##############-utils-#################
def make_box(center_x, center_y):
    line_range_x = [center_x-0.5, center_x+0.5, center_x+0.5, center_x-0.5]
    line_range_y = [center_y+0.5, center_y+0.5, center_y-0.5, center_y-0.5]
    return line_range_x, line_range_y


def read_log(csv_path):
    csv_file = pd.read_csv(csv_path, header=2)
    timestep = csv_file["timestep[s]"]
    pos_x = csv_file["pos.x"]
    pos_y = csv_file["pos.y"]
    adm = csv_file["adm"]
    safe = csv_file["safe"]
    vel_h_cost = csv_file["vel_h_cost"]
    angl_h_cost = csv_file["ang_h_cost"]
    cost = csv_file["cost"]
    cal_vel_v = csv_file["cal_vel.v"]
    cal_vel_w = csv_file["cal_val.w"]
    joy_v = csv_file["joy_v"]
    joy_w = csv_file["joy_w"]
    now_v = csv_file["now_v"]
    now_w = csv_file["now_w"]
    cal_time = csv_file["cal_time[ms]"]

    log = {"timestep": timestep,
           "pos_x": pos_x,
           "pos_y": pos_y,
           "adm": adm,
           "safe": safe,
           "vel_h_cost": vel_h_cost,
           "ang_h_cost": angl_h_cost,
           "cost": cost,
           "cal_vel_v": cal_vel_v,
           "cal_vel_w": cal_vel_w,
           "joy_v": joy_v,
           "joy_w": joy_w,
           "now_v": now_v,
           "now_w": now_w,
           "cal_time":cal_time
           }
    return log


def read_mylog(csv_path):
    csv_file = pd.read_csv(csv_path, header=2)
    timestep = csv_file["timestep[s]"]
    pos_x = csv_file["pos.x"]
    pos_y = csv_file["pos.y"]
    linadm = csv_file["linadm"]
    linsafe = csv_file["linsafe"]
    angadm = csv_file["angadm"]
    angsafe = csv_file["angsafe"]
    vel_h_cost = csv_file["vel_h_cost"]
    angl_h_cost = csv_file["ang_h_cost"]
    cost = csv_file["cost"]
    cal_vel_v = csv_file["cal_vel.v"]
    cal_vel_w = csv_file["cal_val.w"]
    joy_v = csv_file["joy_v"]
    joy_w = csv_file["joy_w"]
    lindist = csv_file["lindist"]
    angdist = csv_file["angdist"]
    now_v = csv_file["now_v"]
    now_w = csv_file["now_w"]
    cal_time = csv_file["cal_time[ms]"]

    mylog = {"timestep": timestep,
             "pos_x": pos_x,
             "pos_y": pos_y,
             "linadm": linadm,
             "linsafe": linsafe,
             "angadm": angadm,
             "angsafe": angsafe,
             "vel_h_cost": vel_h_cost,
             "ang_h_cost": angl_h_cost,
             "cost": cost,
             "cal_vel_v": cal_vel_v,
             "cal_vel_w": cal_vel_w,
             "joy_v": joy_v,
             "joy_w": joy_w,
             "lindist": lindist,
             "angdist": angdist,
             "now_v": now_v,
             "now_w": now_w,
             "cal_time":cal_time
             }
    return mylog

def distinct_log(log_dir):
    csv_paths = glob.glob(os.path.join(log_dir, "*"))
    filenames = []
    filenames_noext = []
    pro_or_pablo = []
    for logfile in csv_paths:
        name = os.path.basename(logfile)
        filenames.append(name)
        filenames_noext.append(name.split(".")[0])

        if name.split("_")[0] == "log":
            pro_or_pablo.append("pablo")
        elif name.split("_")[0] == "mylog":
            pro_or_pablo.append("pro")
    return csv_paths,filenames_noext, pro_or_pablo


def make_sparse_house():
    sparse_house_shape = [[[-0.5, -0.5], [-0.5, 10]], [[-0.5, 15], [10, 10]],
                   [[15, 15],    [10, -0.5]], [[15, -0.5], [-0.5, -0.5]],
                   [[-0.5, 5.5], [6.5, 6.5]], [[5.5, 5.5], [6.5, 10]],
                   [[6.5,6.5], [6.5, 9.0]], [[6.5,12], [9.0,9.0]],
                   [[5.0,5.0], [-0.5, 4.0]], [[5.0,10], [4.0,4.0]],
                   [[10,10],[4,-0.5]],
                   ]

    fill_list_1_x = [-0.5, 5.5, 5.5, -0.5]
    fill_list_1_y = [6.5, 6.5, 10, 10]
    fill_list_2_x = [5, 5, 10, 10]
    fill_list_2_y = [-0.5, 4, 4, -0.5]

    for i in range(len(sparse_house_shape)):
        plt.plot(sparse_house_shape[i][0], sparse_house_shape[i][1], 'k-', lw=4)

    plt.fill(fill_list_1_x, fill_list_1_y, color="gray")
    plt.fill(fill_list_2_x, fill_list_2_y, color="gray")
    pass


#- TODO -#
def draw_robot_radius(ax,r,x,y):
    for i in range(0,len(x),4):
        c = patches.Circle(xy=(x[i], y[i]), radius=r,fill=False,ec="blue")
        ax.add_patch(c)


def make_house():
    house_shape = [[[-0.5, -0.5], [-0.5, 10]], [[-0.5, 15], [10, 10]],
                   [[15, 15],    [10, -0.5]], [[15, -0.5], [-0.5, -0.5]],
                   [[1, 1],    [0.5, 9]], [[1, 3.5],      [9, 9]],
                   [[3.5, 3.5],    [9, 0.5]], [[3.5, 1], [0.5, 0.5]],
                   [[5, 5],  [10, 5]], [[5, 10],     [5, 5]],
                   [[10, 10],     [10, 5]], [[5, 5], [-0.5, 4]],
                   [[5, 10],     [4, 4]], [[10, 10],   [4, -0.5]]]

    fill_list_1_x = [1, 3.5, 3.5, 1]
    fill_list_1_y = [9, 9, 0.5, 0.5]
    fill_list_2_x = [5, 5, 10, 10]
    fill_list_2_y = [10, 5, 5, 10]
    fill_list_3_x = [5, 5, 10, 10]
    fill_list_3_y = [-0.5, 4, 4, -0.5]

    boxes_center = [[14, 8], [12.5, 5], [14.5, 3], [11, 3]]

    for i in range(len(house_shape)):
        plt.plot(house_shape[i][0], house_shape[i][1], 'k-', lw=4)

    plt.fill(fill_list_1_x, fill_list_1_y, color="gray")
    plt.fill(fill_list_2_x, fill_list_2_y, color="gray")
    plt.fill(fill_list_3_x, fill_list_3_y, color="gray")

    for i in range(len(boxes_center)):
        line_range_x, line_range_y = make_box(
            boxes_center[i][0], boxes_center[i][1])
        plt.fill(line_range_x, line_range_y, color="gray")

    plot_start_goal()


def plot_start_goal():
    goal = [14, 0]
    start = [0, 0]

    rad = np.arange(0, 2*np.pi, np.pi/100)
    gx = goal[0] + 0.2*np.cos(rad)
    gy = goal[1] + 0.2*np.sin(rad)

    sx = start[0] + 0.2*np.cos(rad)
    sy = start[1] + 0.2*np.sin(rad)

    plt.plot(gx, gy, color="red")
    plt.plot(sx, sy, color="blue")
