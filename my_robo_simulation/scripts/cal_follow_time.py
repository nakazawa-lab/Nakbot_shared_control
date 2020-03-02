import os
import os.path as osp
import numpy as np
import glob

import utils

class FollowTime():
    def __init__(self,csv_paths,pro_or_pablo):
        self.MAX_VEL = 0.8
        self.MAX_ANG = 1.5
        self.TH_VEL = 0.72
        self.TH_ANG = 1.35
        self.csv_paths = csv_paths
        self.pro_or_pablo = pro_or_pablo


    def check_format(self,LOG,key):
        thres = None
        if key == "vel":
            thres = self.MAX_VEL
        elif key == "ang":
            thres = self.MAX_ANG

        for i in range(len(LOG)-1):
            if (LOG[i]==0.0) and (LOG[i+1]!=0.0):
                if LOG[i+1] != thres:
                    print("")
                    print("invalid format in " + key)
                    print("LOG[i]: "+str(LOG[i]) + " LOG[i+1]: " + str(LOG[i+1]))
                else:
                    return i+1
        return None

        
    def cal_ang_result(self,path,LOG_VEL,LOG_TIME,LOG_NVEL):
        non0_idx = self.check_format(LOG_VEL,"ang")

        if non0_idx == None:
            print("invalid experiment: " + path)
            return None
        else:
            for i in range(non0_idx,len(LOG_VEL)):
                if LOG_NVEL[i] > self.TH_ANG:
                    dur = LOG_TIME[i]-LOG_TIME[non0_idx]
                    return dur
            return None

    def cal_vel_result(self,path,LOG_VEL,LOG_TIME,LOG_NVEL):
        non0_idx = self.check_format(LOG_VEL,"vel")

        if non0_idx == None:
            print("invalid experiment: " + path)
            return None
        else:
            for i in range(non0_idx,len(LOG_VEL)):
                if LOG_NVEL[i] > self.TH_VEL:
                    dur = LOG_TIME[i]-LOG_TIME[non0_idx]
                    return dur
            return None


    def calculate(self):
        result = []
        for i, csv_path in enumerate(self.csv_paths):
            LOG = None
            if self.pro_or_pablo[i] == "pablo":
                LOG = utils.read_log(csv_path)
            elif self.pro_or_pablo[i] == "pro":
                LOG = utils.read_mylog(csv_path)

            vel_time = self.cal_vel_result(csv_path,LOG["joy_v"],LOG["timestep"],LOG["now_v"])
            ang_time = self.cal_ang_result(csv_path,LOG["joy_w"],LOG["timestep"],LOG["now_w"])
            result.append([osp.basename(csv_path),str(vel_time),str(ang_time)])
        return result

def save_result(result,dirname):
    os.makedirs("./"+dirname,exist_ok=True)
    with open("./"+dirname +"/result.csv",mode='w') as f:
        f.write("filename,vel_follow_time,ang_follow_time\n")
        for i in range(len(result)):
            print(result[i])
            line = ",".join(result[i])+"\n"
            print(line)
            f.write(line)

def main():
    csv_paths, _, pro_or_pablo = utils.distinct_log("../log/follow_time_src")
    CalFolTime = FollowTime(csv_paths,pro_or_pablo)
    result = CalFolTime.calculate()
    save_result(result,"follow_result")

if __name__=="__main__":
    main()