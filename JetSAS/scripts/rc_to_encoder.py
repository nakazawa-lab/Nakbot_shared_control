import os
import sys
import numpy as np
import pandas as pd
import glob
from matplotlib import pyplot as plt

def read_log(path):
    csv_file = pd.read_csv(path,header=0)
    csv = csv_file[csv_file["r1_rot"] !=0]

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

def save_rc_enc_fig():
    # plt.scatter(x,LOG["right_vel"],label="raw right")
    # plt.scatter(x,LOG["left_vel"],label="raw left")
    # plt.plot(x,est_right,label="est right")
    # plt.plot(x,est_left,label ="est left")
    # plt.legend()
    # plt.show()
    pass

def write_result(result,velname):
    with open("result/result_"+velname+".csv",mode="w") as f:
        f.write("path,e_right,e_left\n")
        for i in range(len(result)):
            f.write(','.join(result[i]) + '\n')

def write_minmax(result,velname):
    with open("result/result_minmax_"+velname+".csv",mode="w") as f:
        f.write("max,min\n")
        for i in range(len(result)):
            f.write(','.join(result[i]) + '\n')

def lin_or_rot(velname,LOG):
    if velname == "lin":
        return LOG["lin"]
    elif velname == "rot":
        return LOG["rot"]
    else:
        print("error. arg must be [lin] or [rot]")
        sys.exit(1)

def cal_max_min(VEL_LOG):
    np_vel = VEL_LOG.to_numpy()
    max_ = np.max(np_vel)
    min_ = np.min(np_vel)
    return [str(max_),str(min_)]


def cal_rc_to_encoder(csv_path,velname):
    result_vel_enc = []
    result_minmax = []
    for path in csv_path:
        LOG = read_log(path)
        x = lin_or_rot(velname,LOG)

        e_right = np.polyfit(x,LOG["right_vel"],1)
        e_left = np.polyfit(x,LOG["left_vel"],1)
        est_right = np.poly1d(e_right)(x)
        est_left = np.poly1d(e_left)(x)

        # ==TODO== #
        plt.rcParams['font.family'] ='Times New Roman'#使用するフォント
        plt.rcParams['xtick.direction'] = 'in'#x軸の目盛線が内向き('in')か外向き('out')か双方向か('inout')
        plt.rcParams['ytick.direction'] = 'in'#y軸の目盛線が内向き('in')か外向き('out')か双方向か('inout')
        MIN_RC=1130
        MAX_RC=1250
        RC_INTERVAL=20
        MIN_ENC=4999960
        MAX_ENC=5000040
        ENC_INTERVAL=10

        #xp = np.arange(np.min(x),np.max(x),1)
        #fig = plt.figure(figsize=(10,4.8))
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(x,LOG["right_vel"],label="right wheel velocity",marker='o')
        ax.plot(x,LOG["left_vel"],label="left wheel velocity",marker='o')
        ax.plot(x,est_right,label="estimated function(right wheel)")
        ax.plot(x,est_left,label="estimated function(left wheel)")

        ax.set_xlabel("RC pulse width")
        ax.set_ylabel("Encoder pulse value")
        ax.set_xlim([MIN_RC,MAX_RC])
        ax.set_ylim([MIN_ENC,MAX_ENC])
        ax.set_xticks(np.arange(MIN_RC,MAX_RC,RC_INTERVAL))
        ax.set_yticks(np.arange(MIN_ENC,MAX_ENC,ENC_INTERVAL))
        ax.legend()
        ax.grid()
        plt.savefig("./figure/{}_regression_fig.png".format((os.path.basename(path)).split('.')[0]))
        plt.close()

        print(os.path.basename(path))
        print(e_right)
        print(e_left)

        result_vel_enc.append([os.path.basename(path),str(e_right),str(e_left)])

        result_minmax.append(cal_max_min(x))
    
    write_result(result_vel_enc,velname)
    write_minmax(result_minmax,velname)



def main(velname):
    csv_path = glob.glob("../log/graph_src/*")
    cal_rc_to_encoder(csv_path,velname)


if __name__=="__main__":
    args = sys.argv
    if(len(args)!=2):
        print("usage:python rc_to_encoder.py [lin or rot]")
        sys.exit(1)
    main(sys.argv[1])
