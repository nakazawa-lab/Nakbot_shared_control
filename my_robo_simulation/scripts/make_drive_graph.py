import os
import matplotlib.pyplot as plt
import numpy as np
from math import ceil
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches
import pandas as pd
import argparse
import glob
import pylab
import utils

ANGULAR_MAX=1.8
LINEAR_MAX =1.2
TIME_INTERVAL=4
COST_MAX=1.1
COST_MIN=-0.1
Y_SCALE_NUM=9

def main():
    log_dir = "../log/graph_src"
    plt.rcParams["font.size"] = 14
    plt.rcParams['font.family'] ='Times New Roman'#使用するフォント
    plt.rcParams['xtick.direction'] = 'in'#x軸の目盛線が内向き('in')か外向き('out')か双方向か('inout')
    plt.rcParams['ytick.direction'] = 'in'#y軸の目盛線が内向き('in')か外向き('out')か双方向か('inout')

    ############-read logfiles in folders-##############
    csv_paths,filenames_noext,pro_or_pablo = utils.distinct_log(log_dir)

    for i, csv_path in enumerate(csv_paths):
        if pro_or_pablo[i] == "pablo":
            LOG = utils.read_log(csv_path)
        elif pro_or_pablo[i] == "pro":
            LOG = utils.read_mylog(csv_path)
        filename_noext = filenames_noext[i]
        os.makedirs("./figure/"+filename_noext,exist_ok=True)

        # 横軸の時間をもとめる
        # print(LOG["timestep"][len(LOG["timestep"])-1])
        # print(ceil(LOG["timestep"][len(LOG["timestep"])-1]))
        max_time = ceil(LOG["timestep"][len(LOG["timestep"])-1])

        #########-3D cost graph-###############

        # #costfig = plt.figure(figsize=(10, 4.8))
        # costfig = plt.figure()
        # ax = Axes3D(costfig)
        # ax.set_xlabel("X[m]")
        # ax.set_ylabel("Y[m]")
        # ax.set_zlabel("cost")

        # ax.plot(LOG["pos_x"], LOG["pos_y"], LOG["cost"], marker="o")
        # ax.view_init(elev=30., azim=-120)
        # plt.savefig('./figure/{}/{}_cost_figure.png'.format(filename_noext,filename_noext))

        # #plt.show()
        # plt.close()

        ######-3D adm graph-############
        # #admfig = plt.figure(figsize=(10, 4.8))
        # admfig = plt.figure()
        # ax2 = Axes3D(admfig)
        # #ax2 = admfig.gca(projection='3d')
        # ax2.set_xlabel("X[m]")
        # ax2.set_ylabel("Y[m]")
        # ax2.set_zlabel("danger cost")
        # if pro_or_pablo[i] == "pablo":
        #     ax2.plot(LOG["pos_x"], LOG["pos_y"],
        #             LOG["adm"], marker="o", label="danger cost")
        #     ax2.view_init(elev=30., azim=-120)
        #     plt.legend()
        #     plt.savefig('./figure/{}/{}_adm_figure.png'.format(filename_noext,filename_noext))

        # if pro_or_pablo[i] == "pro":
        #     ax2.plot(LOG["pos_x"], LOG["pos_y"], LOG["linadm"],
        #             marker="o", label="linear danger cost")
        #     ax2.plot(LOG["pos_x"], LOG["pos_y"], LOG["angadm"],
        #             marker="o", label="angular danger cost")
        #     ax2.view_init(elev=30., azim=-120)
        #     plt.legend()
        #     plt.savefig('./figure/{}/{}_adm_figure.png'.format(filename_noext,filename_noext))
        # #plt.show()
        # plt.close()

        #########-2D path graph-###############
        pathfig = plt.figure()
        ax = pathfig.add_subplot(111)
        ax.set_xlabel("x[m]")
        ax.set_ylabel("y[m]")

        #utils.make_sparse_house()
        #utils.make_house

        # 実機実験のとき
        ax.set_xlim(left=-1.0,right=4.0)                     
        ax.set_ylim(bottom=-3.0,top=1.0)         
        ax.set_xticks(np.arange(-1,4,1))
        ax.set_yticks(np.arange(-3,1,1))

        # できたグラフに応じて
        #plt.gca().yaxis.set_major_formatter(plt.FormatStrFormatter('%.1f'))#y軸小数点以下3桁表示
        
        utils.draw_robot_radius(ax,0.14,LOG["pos_x"],LOG["pos_y"])
        ax.plot(LOG["pos_x"], LOG["pos_y"], label="robot path")
        
        plt.legend(bbox_to_anchor=(1, 1.1), loc='upper right',
                borderaxespad=0, fontsize=11)

        plt.savefig("./figure/{}/{}_path_figure".format(filename_noext,filename_noext))
        plt.close()

        #########-2D linvel graph-###############
        linvelfig = plt.figure(figsize=(11, 4.8),facecolor="white")

        ax = linvelfig.add_subplot(111)
        ax.plot(LOG["timestep"], LOG["joy_v"], label="human input")
        ax.plot(LOG["timestep"], LOG["cal_vel_v"], label="modified linear velocity")
        ax.set_xlabel("Time[s]")
        ax.set_ylabel("linear velocity[m/s]")
        ax.xaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")
        ax.yaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")

        ax.set_xlim(left=0)
        ax.set_ylim(bottom=-LINEAR_MAX)
        ax.set_xticks(np.arange(0,max_time,TIME_INTERVAL))
        ax.set_yticks(np.linspace(-LINEAR_MAX,LINEAR_MAX,Y_SCALE_NUM))
        
        ax.legend()
        plt.legend(bbox_to_anchor=(1.03, 1), loc='upper left',
                borderaxespad=0, fontsize=11)
        pylab.subplots_adjust(right=0.75)

        plt.savefig("./figure/{}/{}_linvel_figure".format(filename_noext,filename_noext))
        plt.close()

        #########-2D angvel graph-###############
        angvelfig = plt.figure(figsize=(11, 4.8),facecolor="white")

        ax = angvelfig.add_subplot(111)
        ax.plot(LOG["timestep"], LOG["joy_w"], label="joy angular vel")
        ax.plot(LOG["timestep"], LOG["cal_vel_w"], label="calculated angular velocity")
        ax.set_xlabel("Time[s]")
        ax.set_ylabel("angular velocity[rad/s]")
        ax.xaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")
        ax.yaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")

        ax.set_xlim(left=0)
        ax.set_ylim(bottom=-ANGULAR_MAX)
        ax.set_xticks(np.arange(0,max_time,TIME_INTERVAL))
        ax.set_yticks(np.linspace(-ANGULAR_MAX,ANGULAR_MAX,Y_SCALE_NUM))

        ax.legend()
        plt.legend(bbox_to_anchor=(1.03, 1), loc='upper left',
                borderaxespad=0, fontsize=11)
        pylab.subplots_adjust(right=0.75)

        plt.savefig("./figure/{}/{}_angnvel_figure".format(filename_noext,filename_noext))
        plt.close()

        
        #########-2D cost graph-###############
        costfig = plt.figure(figsize=(11,4.8),facecolor="white")
        ax = costfig.add_subplot(111)
        ax.plot(LOG["timestep"], LOG["cost"], label="cost")
        ax.plot(LOG["timestep"], LOG["ang_h_cost"], label="heading difference cost")
        #ax.plot(LOG["timestep"], LOG["vel_h_cost"], label="velocity difference cost")
        
        ax.set_xlabel("Time[s]")
        ax.set_ylabel("cost")
        ax.xaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")
        ax.yaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")

        ax.set_xlim(left=0)
        ax.set_xticks(np.arange(0,max_time,TIME_INTERVAL))
        ax.legend()
        plt.legend(bbox_to_anchor=(1.07, 1), loc='upper left',
                borderaxespad=0, fontsize=11)
        pylab.subplots_adjust(right=0.75)

        plt.savefig("./figure/{}/{}_cost2d_figure".format(filename_noext,filename_noext))
        plt.close()

        #########-2D danger graph-###############
        ## mylogのみ有効 ## 
        dangerfig = plt.figure(figsize=(11,4.8),facecolor="white")
        ax = dangerfig.add_subplot(111)
        #ax.plot(LOG["timestep"], LOG["linadm"], label="linear danger")
        ax.plot(LOG["timestep"], LOG["angadm"], label="angular danger")
        
        ax.set_xlabel("Time[s]")
        ax.set_ylabel("cost")
        ax.xaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")
        ax.yaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")

        ax.set_xlim(left=0)
        ax.set_xticks(np.arange(0,max_time,TIME_INTERVAL))
        ax.legend()
        plt.legend(bbox_to_anchor=(1.07, 1), loc='upper left',
                borderaxespad=0, fontsize=11)
        pylab.subplots_adjust(right=0.75)

        plt.savefig("./figure/{}/{}_danger_figure".format(filename_noext,filename_noext))
        plt.close()
        
        #########-2D lin graph-###############
        linfig = plt.figure(figsize=(11,4.8),facecolor="white")       # default figsize = (6.4, 4.8)

        ax1 = linfig.add_subplot(111)
        ax1.plot(LOG["timestep"], LOG["joy_v"], label="human input")
        ax1.plot(LOG["timestep"], LOG["cal_vel_v"], label="modified linear velocity")
        ax1.plot(LOG["timestep"], LOG["now_v"], label="current linear velocity")
        ax1.set_xlabel("Time[s]")
        ax1.set_ylabel("linear velocity[m/s]")

        ax1.xaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")
        ax1.yaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")

        ax2 = ax1.twinx()
        ax2.grid(False)
        ax2.plot(LOG["timestep"], LOG["vel_h_cost"], color = "crimson", label="velocity difference cost")

        ax1.set_xlim(left=0)
        ax1.set_ylim(bottom=-LINEAR_MAX)
        ax2.set_ylim(bottom=-COST_MIN)
        ax1.set_yticks(np.linspace(-LINEAR_MAX,LINEAR_MAX,Y_SCALE_NUM))
        ax2.set_yticks(np.linspace(-COST_MIN,COST_MAX,Y_SCALE_NUM))
        ax1.set_xticks(np.arange(0,max_time,TIME_INTERVAL))

        h1, l1 = ax1.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()

        ax2.set_ylabel("cost")
        ax1.legend(h1+h2, l1+l2,bbox_to_anchor=(1.09, 1), loc='upper left',
                borderaxespad=0, fontsize=11)
        pylab.subplots_adjust(left= 0.09, right=0.7,bottom=0.15, top=0.95)

        plt.savefig("./figure/{}/{}_linfig_figure".format(filename_noext,filename_noext))
        plt.close()

        #########-2D head graph-###############
        angfig = plt.figure(figsize=(11,4.8))
        angfig.patch.set_facecolor('red')

        ax1 = angfig.add_subplot(111)
        ax1.plot(LOG["timestep"], LOG["joy_w"], label="human input")
        ax1.plot(LOG["timestep"], LOG["cal_vel_w"], label="modified angular velocity")
        ax1.plot(LOG["timestep"], LOG["now_w"], label="current angular velocity")
        ax1.set_xlabel("Time[s]")
        ax1.set_ylabel("angular velocity[rad/s]")
        ax1.xaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")
        ax1.yaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")

        ax2 = ax1.twinx()
        ax2.grid(False)
        ax2.plot(LOG["timestep"], LOG["ang_h_cost"], color = "crimson", label="heading difference cost")
        
        ax1.set_xlim(left=0)
        ax1.set_ylim(bottom=-ANGULAR_MAX)
        ax2.set_ylim(bottom=COST_MIN)
        ax1.set_yticks(np.linspace(-ANGULAR_MAX,ANGULAR_MAX,Y_SCALE_NUM))
        ax2.set_yticks(np.linspace(-COST_MIN,COST_MAX,Y_SCALE_NUM))
        ax1.set_xticks(np.arange(0,max_time,TIME_INTERVAL))

        h1, l1 = ax1.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()

        ax2.set_ylabel("cost")
        ax1.legend(h1+h2, l1+l2,bbox_to_anchor=(1.09, 1), loc='upper left',
                borderaxespad=0, fontsize=11)
        pylab.subplots_adjust(left= 0.09, right=0.7,bottom=0.15, top=0.95)

        plt.savefig("./figure/{}/{}_angfig_figure".format(filename_noext,filename_noext))
        plt.close()


if __name__ == '__main__':
    main()
