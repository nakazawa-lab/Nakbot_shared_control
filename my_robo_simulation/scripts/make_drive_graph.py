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
import utils

sns.set_style("darkgrid")

def main():
    log_dir = "../log/graph_src"

    ############-read logfiles in folders-##############
    csv_paths,filenames_noext,pro_or_pablo = utils.distinct_log(log_dir)

    for i, csv_path in enumerate(csv_paths):
        if pro_or_pablo[i] == "pablo":
            LOG = utils.read_log(csv_path)
        elif pro_or_pablo[i] == "pro":
            LOG = utils.read_mylog(csv_path)
        filename_noext = filenames_noext[i]
        os.makedirs("./figure/"+filename_noext,exist_ok=True)

        #########-3D cost graph-###############
        
        #costfig = plt.figure(figsize=(10, 4.8))
        costfig = plt.figure()
        ax = Axes3D(costfig)
        ax.set_xlabel("X[m]")
        ax.set_ylabel("Y[m]")
        ax.set_zlabel("cost")

        ax.plot(LOG["pos_x"], LOG["pos_y"], LOG["cost"], marker="o")
        ax.view_init(elev=30., azim=-120)
        plt.savefig('./figure/{}/{}_cost_figure.png'.format(filename_noext,filename_noext))

        #plt.show()
        plt.close()
        
        ######-3D adm graph-############
        #admfig = plt.figure(figsize=(10, 4.8))
        admfig = plt.figure()
        ax2 = Axes3D(admfig)
        #ax2 = admfig.gca(projection='3d')
        ax2.set_xlabel("X[m]")
        ax2.set_ylabel("Y[m]")
        ax2.set_zlabel("danger cost")
        if pro_or_pablo[i] == "pablo":
            ax2.plot(LOG["pos_x"], LOG["pos_y"],
                    LOG["adm"], marker="o", label="danger cost")
            ax2.view_init(elev=30., azim=-120)
            plt.legend()
            plt.savefig('./figure/{}/{}_adm_figure.png'.format(filename_noext,filename_noext))

        if pro_or_pablo[i] == "pro":
            ax2.plot(LOG["pos_x"], LOG["pos_y"], LOG["linadm"],
                    marker="o", label="linear danger cost")
            ax2.plot(LOG["pos_x"], LOG["pos_y"], LOG["angadm"],
                    marker="o", label="angular danger cost")
            ax2.view_init(elev=30., azim=-120)
            plt.legend()
            plt.savefig('./figure/{}/{}_adm_figure.png'.format(filename_noext,filename_noext))
        #plt.show()
        plt.close()

        #########-2D path graph-###############
        #pathfig = plt.figure(figsize=(10, 4.8))
        pathfig = plt.figure()
        ax = pathfig.add_subplot(111)

        ax.set_xlabel("X[m]")
        ax.set_ylabel("Y[m]")
        utils.make_sparse_house()
        utils.draw_robot_radius(ax,0.23,LOG["pos_x"],LOG["pos_y"])
        ax.plot(LOG["pos_x"], LOG["pos_y"], label="robot path")
        plt.legend(bbox_to_anchor=(1, 1), loc='upper right',
                borderaxespad=0, fontsize=8)
        plt.savefig("./figure/{}/{}_path_figure".format(filename_noext,filename_noext))

        #plt.show()
        plt.close()
        
        #########-2D linvel graph-###############
        linvelfig = plt.figure(figsize=(10, 4.8))
        ax = linvelfig.add_subplot(111)
        ax.plot(LOG["timestep"], LOG["joy_v"], label="joy linear vel")
        ax.plot(LOG["timestep"], LOG["cal_vel_v"], label="calculated linear vel")
        ax.set_xlabel("Time[s]")
        ax.set_ylabel("linear velocity[m/s]")
        #ax.set_xlim([0,30])
        ax.xaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")
        ax.yaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")
        ax.legend()
        plt.legend(bbox_to_anchor=(1.07, 1), loc='upper left',
                borderaxespad=0, fontsize=8)
        pylab.subplots_adjust(right=0.75)
        plt.savefig("./figure/{}/{}_linvel_figure".format(filename_noext,filename_noext))

        #plt.show()
        plt.close()

        #########-2D angvel graph-###############
        angvelfig = plt.figure(figsize=(10, 4.8))
        ax = angvelfig.add_subplot(111)
        ax.plot(LOG["timestep"], LOG["joy_w"], label="joy angular vel")
        ax.plot(LOG["timestep"], LOG["cal_vel_w"], label="calculated angular velocity")
        ax.set_xlabel("Time[s]")
        ax.set_ylabel("angular velocity[rad/s]")
        #ax.set_xlim([0,30])
        ax.xaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")
        ax.yaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")
        ax.legend()
        plt.legend(bbox_to_anchor=(1.07, 1), loc='upper left',
                borderaxespad=0, fontsize=8)
        pylab.subplots_adjust(right=0.75)
        plt.savefig("./figure/{}/{}_angnvel_figure".format(filename_noext,filename_noext))

        #plt.show()
        plt.close()
        """
        #########-2D cost graph-###############
        costfig = plt.figure(figsize(10,4.8))
        ax = costfig.add_subplot(111)
        ax.plot(LOG["timestep"], LOG["cost"], label="cost")
        ax.set_xlabel("Time[s]")
        ax.set_ylabel("cost")
        ax.xaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")
        ax.yaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")
        ax.legend()
        plt.legend(bbox_to_anchor=(1.07, 1), loc='upper left',
                borderaxespad=0, fontsize=8)
        pylab.subplots_adjust(right=0.75)
        plt.savefig("./figure/{}/{}_cost2d_figure".format(filename_noext,filename_noext))

        #plt.show()
        plt.close()
        """
        #########-2D lin graph-###############
        linfig = plt.figure(figsize=(10,4.8))       # default figsize = (6.4, 4.8)
        ax1 = linfig.add_subplot(111)
        ax1.plot(LOG["timestep"], LOG["joy_v"], label="human v")
        ax1.plot(LOG["timestep"], LOG["cal_vel_v"], label="calculated linear velocity")
        ax1.plot(LOG["timestep"], LOG["now_v"], label="current v")
        ax1.set_xlabel("Time[s]")
        ax1.set_ylabel("linear velocity[m/s]")
        #ax.set_xlim([0,30])
        ax1.xaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")
        ax1.yaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")

        ax2 = ax1.twinx()
        ax2.grid(False)
        ax2.plot(LOG["timestep"], LOG["vel_h_cost"], color = "crimson", label="vel difference cost")
        
        h1, l1 = ax1.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        
        ax2.set_ylabel("cost")
        ax1.legend(h1+h2, l1+l2,bbox_to_anchor=(1.07, 1), loc='upper left',
                borderaxespad=0, fontsize=8)

        pylab.subplots_adjust(right=0.75)
        plt.savefig("./figure/{}/{}_linfig_figure".format(filename_noext,filename_noext))
        #plt.show()
        plt.close()

        #########-2D head graph-###############
        angfig = plt.figure(figsize=(10,4.8)) 

        ax1 = angfig.add_subplot(111)
        ax1.plot(LOG["timestep"], LOG["joy_w"], label="human w")
        ax1.plot(LOG["timestep"], LOG["cal_vel_w"], label="calculated angular velocity")
        ax1.plot(LOG["timestep"], LOG["now_w"], label="current w")
        ax1.set_xlabel("Time[s]")
        ax1.set_ylabel("angular velocity[rad/s]")
        #ax.set_xlim([0,30])
        ax1.xaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")
        ax1.yaxis.grid(True, which= "major", linestyle ="-", color = "#CFCFCF")

        ax2 = ax1.twinx()
        ax2.grid(False)
        ax2.plot(LOG["timestep"], LOG["ang_h_cost"], color = "crimson", label="angular difference cost")
        
        h1, l1 = ax1.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        
        ax2.set_ylabel("cost")
        ax1.legend(h1+h2, l1+l2,bbox_to_anchor=(1.07, 1), loc='upper left',
                borderaxespad=0, fontsize=8)

        pylab.subplots_adjust(right=0.75)
        plt.savefig("./figure/{}/{}_angfig_figure".format(filename_noext,filename_noext))
        #plt.show()
        plt.close()


if __name__ == '__main__':
    main()
