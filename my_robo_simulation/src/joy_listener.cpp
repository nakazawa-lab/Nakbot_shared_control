#include"my_robo_simulation/my_robo_drive.h"
/*
void my_robo::cb_joy(const sensor_msgs::Joy& joy_msg){
    // ジョイスティック左側
    // 上→axes[1]の正方向
    // 左→axes[0]の正方向
    x_vel_joy =spec.x_max_vel*joy_msg.axes[1];
    //cmd_vel.linear.y =joy_msg.axes[2];
    
    if(joy_msg.axes[1]>=0) z_ang_joy=spec.z_max_ang*joy_msg.axes[0];
    else z_ang_joy=-1*spec.z_max_ang*joy_msg.axes[0];

    ROS_INFO("x_joy: %f",x_vel_joy);
    ROS_INFO("z_ang: %f\n",z_ang_joy);
}
*/
/*
int main(int argc, char **argv){
    ros::init(argc, argv, "basic_twist_publisher");
    ros::NodeHandle n;

    n.getParam("/my_robo/diff_drive_controller/linear/x/max_velocity",x_max_vel);
    n.getParam("my_robo/diff_drive_controller/angular/z/max_velocity",z_max_ang);

    //publish
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    //subscriibe
    ros::Subscriber joy_sub   = n.subscribe("joy", 10, joy_callback);

    ros::Rate loop_rate(10);
    while (ros::ok()){
        cmd_pub.publish(cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
*/
