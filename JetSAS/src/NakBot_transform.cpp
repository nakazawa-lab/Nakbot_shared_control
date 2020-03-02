#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class NakBot_TF
{
public:
    const double BASE_FOOT_TO_BASE_LINK_Z = 0.15;

    // 面倒なので未指定 使うことがあれば他と同様にして入れる
    // const double BASE_LINK_TO_BACK_CAS_X = 0.2;
    // const double BASE_LINK_TO_BACK_CAS_Y = 0.2;
    // const double BASE_LINK_TO_BACK_CAS_Z = 0.2;

    // const double BASE_LINK_TO_FRONT_CAS_X = 0.2;
    // const double BASE_LINK_TO_FRONT_CAS_Y = 0.2;
    // const double BASE_LINK_TO_FRONT_CAS_Z = 0.2;

    const double BASE_LINK_TO_BASE_SCAN_X = 0.12;
    const double BASE_LINK_TO_BASE_SCAN_Y = 0.0;
    const double BASE_LINK_TO_BASE_SCAN_Z = 0.0;

    const double BASE_LINK_TO_BASE_CAMERA_X = 0.03;
    const double BASE_LINK_TO_BASE_CAMERA_Y = 0.0;
    const double BASE_LINK_TO_BASE_CAMERA_Z = 0.225;
    ros::Time current_time = ros::Time::now();

    tf::StampedTransform make_StampTransform(double x_q,double y_q, double z_q, double w_q, double x,double y ,double z, const char* src, const char* dist){
        return tf::StampedTransform(tf::Transform(tf::Quaternion(x_q, y_q, z_q, w_q), tf::Vector3(x, y, z)), current_time,src, dist);
    };
};


int main(int argc, char** argv){
  ros::init(argc, argv, "NakBot_transform_publisher");
  ros::NodeHandle n;
  NakBot_TF transform;
  tf::TransformBroadcaster broadcaster;

  ros::Rate r(100);
  ROS_INFO("Launch NakBot Transform Publisher");

  while(n.ok()){
    transform.current_time = ros::Time::now();
    broadcaster.sendTransform(
        std::vector<tf::StampedTransform>{
            //transform.make_StampTransform(0,0,0,1,0,0,0,"odom","base_footprint"),
            transform.make_StampTransform(0,0,0,1,0,0,transform.BASE_FOOT_TO_BASE_LINK_Z,"base_footprint","base_link"),
            transform.make_StampTransform(0,0,0,1,transform.BASE_LINK_TO_BASE_SCAN_X,transform.BASE_LINK_TO_BASE_SCAN_Y,transform.BASE_LINK_TO_BASE_SCAN_Z,"base_link","scan"),
            transform.make_StampTransform(0,0,0,1,transform.BASE_LINK_TO_BASE_CAMERA_X,transform.BASE_LINK_TO_BASE_CAMERA_Y,transform.BASE_LINK_TO_BASE_CAMERA_Z,"base_link","camera"),
        }
    );
    r.sleep();
  }
}