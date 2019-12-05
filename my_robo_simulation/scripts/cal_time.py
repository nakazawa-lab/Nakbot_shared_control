import os
import numpy as np
import utils

def main():
    log_dir = "../log/cal_time_src"
    csv_paths, filenames_noext, pro_or_pablo = utils.distinct_log(log_dir)

    result =[]
    for i, csv_path in enumerate(csv_paths):
        if pro_or_pablo[i] == "pablo":
            LOG = utils.read_log(csv_path)
        elif pro_or_pablo[i] == "pro":
            LOG = utils.read_mylog(csv_path)
        #filename_noext = filenames_noext[i]
        #os.makedirs("./cal_time_result/"+filename_noext, exist_ok=True)

        time_all_nd = LOG["cal_time"].to_numpy()
        time_nd = time_all_nd[time_all_nd > 0]

        m_time = np.mean(time_nd)
        std_time = np.std(time_nd)
        max_time = np.max(time_nd)
        min_time = np.min(time_nd)
        print(csv_path)
        print("mean",m_time)
        print("std_time",std_time)
        print("max",max_time)
        print("min",min_time)
        print("")

        result.append([csv_path,str(m_time),str(std_time),str(max_time),str(min_time)])

    os.makedirs("./time_result",exist_ok=True)
    with open("./time_result/time_result.csv",mode='w') as f:
        f.write("path,mean,std,max,min\n")
        for i in range(len(result)):
            print(result[i])
            line = ",".join(result[i])+"\n"
            print(line)
            f.write(line)



if __name__ == '__main__':
    main()
