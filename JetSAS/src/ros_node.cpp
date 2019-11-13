#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "JetSAS/ros_node.h"

// void JetSAS_Node::pub_lrf(){
//     lrf_pub.publish(*scan);
//     delete[] scan;
// }

void JetSAS_Node::make_scan_msgs(long* urg_data){
    std::cout << "urg_data num is" << sizeof(urg_data)/sizeof(*urg_data) <<std::endl;

    const int scan_num = sizeof(urg_data)/sizeof(*urg_data);
    scan = new sensor_msgs::LaserScan[scan_num];

    if (scan_num ==0){
        std::cout << "no scan msgs" <<std::endl;
    }
    else{
        for(int i=0; i<scan_num;i++){
            (*scan).ranges[i] = urg_data[i];
        }
        std::cout << "finish make sensor msgs" <<std::endl;
    }
}