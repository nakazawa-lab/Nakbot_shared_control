import os
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import argparse

sns.set_style("darkgrid")

##############-utils-#################
def make_box(center_x,center_y):
    line_range_x = [center_x-0.5, center_x+0.5, center_x+0.5, center_x-0.5]
    line_range_y = [center_y+0.5, center_y+0.5, center_y-0.5, center_y-0.5]
    return line_range_x, line_range_y

def read_log(csv_path):
    csv_file = pd.read_csv(csv_path,header=2)
    timestep = csv_file["timestep"]
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
    joy_w =csv_file["joy_w"]

    log = {   "timestep":timestep,  \
              "pos_x":pos_x,\
              "pos_y":pos_y, \
              "adm"  :adm, \
              "safe" :safe, \
              "vel_h_cost":vel_h_cost, \
              "ang_h_cost":angl_h_cost, \
              "cost":cost, \
              "cal_vel_v":cal_vel_v, \
              "cal_vel_w":cal_vel_w, \
              "joy_v":joy_v, \
              "joy_w":joy_w
            }
    return log

def read_mylog(csv_path):
    csv_file = pd.read_csv(csv_path,header=2)
    timestep = csv_file["timestep"]
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
    joy_w =csv_file["joy_w"]

    mylog = { "timestep":timestep, \
              "pos_x":pos_x,\
              "pos_y":pos_y, \
              "linadm"  :linadm, \
              "linsafe" :linsafe, \
              "angadm"  :angadm, \
              "angsafe" :angsafe, \
              "vel_h_cost":vel_h_cost, \
              "ang_h_cost":angl_h_cost, \
              "cost":cost, \
              "cal_vel_v":cal_vel_v, \
              "cal_vel_w":cal_vel_w, \
              "joy_v":joy_v, \
              "joy_w":joy_w
            }
    return mylog

def make_house():
    house_shape = [ [[-0.5,-0.5],[-0.5,10]], [[-0.5,15],[10,10]  ],
                [[15,15],    [10,-0.5]], [[15,-0.5],[-0.5,-0.5]],
                [[1,1],    [0.5,9]    ], [[1,3.5],      [9,9]  ],
                [[3.5,3.5],    [9,0.5]], [[3.5,1], [0.5,0.5]],
                [[5,5],  [10,5]       ], [[5,10],     [5,5]    ],
                [[10,10],     [10,5]   ], [[5,5], [-0.5,4]    ],
                [[5,10],     [4,4]   ], [[10,10],   [4,-0.5]] ]

    fill_list_1_x = [1,3.5,3.5,1]
    fill_list_1_y = [9,9,0.5,0.5]
    fill_list_2_x = [5,5,10,10]
    fill_list_2_y = [10,5,5,10]
    fill_list_3_x = [5,5,10,10]
    fill_list_3_y = [-0.5,4,4,-0.5]

    boxes_center = [[14,8], [12.5,5], [14.5,3], [11,3]]

    for i in range(len(house_shape)):
        plt.plot(house_shape[i][0],house_shape[i][1],'k-', lw=4)
    
    plt.fill(fill_list_1_x,fill_list_1_y,color="gray")
    plt.fill(fill_list_2_x,fill_list_2_y,color="gray")
    plt.fill(fill_list_3_x,fill_list_3_y,color="gray")

    for i in range(len(boxes_center)):
        line_range_x, line_range_y = make_box(boxes_center[i][0],boxes_center[i][1])
        plt.fill(line_range_x,line_range_y,color="gray")
    
    plot_start_goal()




def plot_start_goal():
    goal = [14,0]
    start = [0,0]

    rad = np.arange(0, 2*np.pi, np.pi/100)
    gx = goal[0] + 0.2*np.cos(rad)
    gy = goal[1] + 0.2*np.sin(rad)

    sx = start[0] + 0.2*np.cos(rad)
    sy = start[1] + 0.2*np.sin(rad)

    plt.plot(gx,gy,color="red")
    plt.plot(sx,sy,color="blue")



def main():
    ############-settings-##############
    log_path = "../log"
    pro_filename = "mylog_10101950_3_3.csv"
    pro_filename_noext = (pro_filename.split("."))[0]
    pro_csv_path = os.path.join(log_path, pro_filename)
    mylog = read_mylog(pro_csv_path)

    pablo_filename = "log_10101716_take2.csv"
    pablo_filename_noext = (pablo_filename.split("."))[0]
    pablo_csv_path = os.path.join(log_path,pablo_filename)
    log = read_log(pablo_csv_path)

    ############-argparse-############
    parser = argparse.ArgumentParser()

    parser.add_argument("method",help="pablo or pro")

    args = parser.parse_args()

    #########-3D cost graph-###############
    costfig = plt.figure()
    ax = Axes3D(costfig)
    ax.set_xlabel("X[m]")
    ax.set_ylabel("Y[m]")
    ax.set_zlabel("cost")
    if args.method=="pablo":
        ax.plot(log["pos_x"],log["pos_y"],log["cost"],marker="o")
        ax.view_init(elev=30., azim=-120)
        plt.savefig('cost_figure_{}.png'.format(pablo_filename_noext)) 
    elif args.method=="pro":
        ax.plot(mylog["pos_x"],mylog["pos_y"],mylog["cost"],marker="o")
        ax.view_init(elev=30., azim=-120)
        plt.savefig('cost_figure_{}.png'.format(pro_filename_noext)) 
    plt.show()
    plt.close()

    ######-3D adm graph-############
    admfig=plt.figure()
    ax2=Axes3D(admfig)
    #ax2 = admfig.gca(projection='3d')
    ax2.set_xlabel("X[m]")
    ax2.set_ylabel("Y[m]")
    ax2.set_zlabel("danger")
    if args.method=="pablo":
        ax2.plot(log["pos_x"],log["pos_y"],log["adm"],marker="o",label="danger")
        ax2.view_init(elev=30., azim=-120)
        plt.legend()
        plt.savefig('adm_figure_{}.png'.format(pablo_filename_noext))
        
    elif args.method=="pro":
        ax2.plot(mylog["pos_x"],mylog["pos_y"],mylog["linadm"],marker="o",label="lin danger")
        ax2.plot(mylog["pos_x"],mylog["pos_y"],mylog["angadm"],marker="o",label="ang danger")
        ax2.view_init(elev=30., azim=-120)
        plt.legend()
        plt.savefig('adm_figure_{}.png'.format(pro_filename_noext)) 
    plt.show()
    plt.close()


    #########-2D path graph-###############
    pathfig = plt.figure()
    plt.xlabel("X[m]")
    plt.ylabel("Y[m]")
    make_house()
    if args.method=="pablo":
        plt.plot(log["pos_x"],log["pos_y"],label="robot path")
        plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0, fontsize=8)
        plt.savefig("path_figure_{}".format(pablo_filename_noext))
    elif args.method=="pro":
        plt.plot(mylog["pos_x"],mylog["pos_y"],label="robot path")
        plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0, fontsize=8)
        plt.savefig("path_figure_{}".format(pro_filename_noext))
    plt.show()
    plt.close()



    #########-2D linvel graph-###############
    linvelfig = plt.figure()
    plt.xlabel("Time[s]")
    plt.ylabel("linear velocity[m/s]")
    if args.method=="pablo":
        plt.plot(log["timestep"],log["joy_v"],label="joy linear vel")
        plt.plot(log["timestep"],log["cal_vel_v"],label="calculated linear vel")
        plt.legend()
        plt.savefig("linvel_figure_{}".format(pablo_filename_noext))
    elif args.method=="pro":
        plt.plot(mylog["timestep"],mylog["joy_v"],label="joy linear vel")
        plt.plot(mylog["timestep"],mylog["cal_vel_v"],label="calculated linear vel")
        plt.legend()
        plt.savefig("linvel_figure_{}".format(pro_filename_noext))
    plt.show()
    plt.close()
    

    #########-2D angvel graph-###############
    angvelfig = plt.figure()
    plt.xlabel("Time[s]")
    plt.ylabel("angular velocity[rad/s]")
    if args.method=="pablo":
        plt.plot(log["timestep"],log["joy_w"],label="joy angular vel")
        plt.plot(log["timestep"],log["cal_vel_w"],label="calculated angular vel")
        plt.legend()
        plt.savefig("angnvel_figure_{}".format(pablo_filename_noext))
    elif args.method=="pro":
        plt.plot(mylog["timestep"],mylog["joy_w"],label="joy angular vel")
        plt.plot(mylog["timestep"],mylog["cal_vel_w"],label="calculated angular vel")
        plt.legend()
        plt.savefig("angvel_figure_{}".format(pro_filename_noext))
    plt.show()
    plt.close()

    #########-2D cost graph-###############
    costfig = plt.figure()
    plt.xlabel("Time[s]")
    plt.ylabel("cost")
    if args.method=="pablo":
        plt.plot(log["timestep"],log["cost"],label="cost")
        plt.legend()
        plt.savefig("cost2d_figure_{}".format(pablo_filename_noext))
    elif args.method=="pro":
        plt.plot(mylog["timestep"],mylog["cost"],label="cost")
        plt.legend()
        plt.savefig("cost2d_figure_{}".format(pro_filename_noext))
    plt.show()
    plt.close()


if __name__ == '__main__':
    main()