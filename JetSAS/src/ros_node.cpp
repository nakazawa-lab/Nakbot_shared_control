#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "JetSAS/ros_node.h"

// void JetSAS_Node::pub_lrf(){
//     lrf_pub.publish(*scan);
//     delete[] scan;
// }

void JetSAS_Node::make_scan_msgs(long* urg_data,const int scan_num){
    //std::cout << "in make scan msgs, urg_data num is " << scan_num <<std::endl;

    //std::cout << "urg_data[0] " << urg_data[0]/100.0 <<" " << urg_angle_increment <<" "<< scan.angle_increment<<std::endl;

    if (scan_num ==0){
        std::cout << "no scan msgs" <<std::endl;
    }
    else{
        for(int i=0; i<scan_num;i++){
            //std::cout << "urg data[" << i <<"] is " << urg_data[i] << " " << scan.angle_increment <<std::endl;
            scan.ranges[i] = urg_data[scan_num-i-1]/100.0;
        }
        //std::cout << "finish make sensor msgs" <<std::endl;
    }
}
