import os
import sys
import numpy as np
import pandas as pd
import glob
from matplotlib import pyplot as plt

def read_log(path):
    csv_file = pd.read_csv(path,header=0)
    csv = csv_file[csv_file["e1_right_vel"] !=0]

    time = csv["time"]
    right_vel = csv["e1_right_vel"]
    left_vel = csv["e2_left_vel"]
    right_sum = csv["e3_right_sum"]
    left_sum = csv["e4_left_sum"]
    rot = csv["r1_rot"]
    lin = csv["r2_lin"]
    chan3 = csv["r3_chan3"]
    chan4 = csv["r4_chan4"]

    log = { "time":time,
            "right_vel":right_vel,
            "left_vel":left_vel,
            "right_sum":right_sum,
            "left_sum":left_sum,
            "rot":rot,
            "lin":lin,
            "chan3":chan3,
            "chan4":chan4
    }
    return log


def main(vel):
    csv_path = glob.glob("../log/graph_src/*")
    result = []
    for path in csv_path:
        LOG = read_log(path)

        if vel == "lin":
            x = LOG["lin"]
        elif vel == "rot":
            x = LOG["rot"]
        else:
            print("error. arg must be [lin] or [rot]")
            sys.exit(1)

        e_right = np.polyfit(x,LOG["right_vel"],1)
        e_left = np.polyfit(x,LOG["left_vel"],1)
        est_right = np.poly1d(e_right)(x)
        est_left = np.poly1d(e_left)(x)

        # ==TODO== #
        # save_fig()
        # plt.scatter(x,LOG["right_vel"],label="raw right")
        # plt.scatter(x,LOG["left_vel"],label="raw left")
        # plt.plot(x,est_right,label="est right")
        # plt.plot(x,est_left,label ="est left")
        # plt.legend()
        # plt.show()

        print(os.path.basename(path))
        print(e_right)
        print(e_left)

        result.append([os.path.basename(path),str(e_right),str(e_left)])
    
    with open("result.csv",mode="w") as f:
        f.write("path,e_right,e_left\n")
        for i in range(len(result)):
            f.write(','.join(result[i]) + '\n')

if __name__=="__main__":
    args = sys.argv
    if(len(args)!=2):
        print("usage:python rc_to_encoder.py [lin or rot]")
        sys.exit(1)
    main(sys.argv[1])
