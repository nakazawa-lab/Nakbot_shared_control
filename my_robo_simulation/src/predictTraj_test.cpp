#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include<geometry_msgs/Pose.h>
#include<tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <math.h>
#include<vector>

namespace test_marker
{

#pragma region クオータニオンの計算のための試行錯誤したゴミ
// クオータニオン x+y+z+wjからz軸方向への回転角psiを求める
double QuaternionToEuler(double q0, double q1, double q2, double q3){
    double q0q0 = q0 * q0;
    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    double q0q3 = q0 * q3;
    double q1q1 = q1 * q1;
    double q1q2 = q1 * q2;
    double q1q3 = q1 * q3;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q3q3 = q3 * q3;

    return atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
}

void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat){
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}
#pragma endregion

double add_theorem_sin(double sin_a, double sin_b, double cos_a, double cos_b){
    //ROS_INFO("sin_a:%f,sin_b:%f,cos_a:%f,cos_b%f",sin_a,sin_b,cos_a,cos_b);
    double a=(sin_a * cos_b + cos_a * sin_b);
    return a;

}

double add_theorem_cos(double sin_a, double sin_b, double cos_a, double cos_b){
    double a= (cos_a*cos_b - sin_a * sin_b);
    return a;
}

// 次の位置を格納する箱
struct position
{
    double x;
    double y;
    double cos_th;
    double sin_th;
};

class PredictTraj_test
{
private:
    ros::Subscriber sub_joy;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_cmd_vel;
    ros::Publisher pub_mark;
    ros::Publisher pub_mark_arr;
    ros::NodeHandle n;
    geometry_msgs::Twist vel;

    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markerarray;
    nav_msgs::Odometry odom;
    geometry_msgs::Twist twist;

    position now_p;
    std::vector<std::vector<double>> PredictTraj;

    int frameid;

public:
    PredictTraj_test()
    {
        //sub_joy = n.subscribe("joy", 10, &PredictTraj_test::joy_callback,this);
        pub_mark = n.advertise<visualization_msgs::Marker>("/marker", 1);
        pub_mark_arr = n.advertise<visualization_msgs::MarkerArray>("/marker_array", 1);
        sub_odom = n.subscribe("/odom", 10, &PredictTraj_test::odom_callback, this);
        sub_cmd_vel = n.subscribe("/cmd_vel", 10, &PredictTraj_test::twist_callback, this);
        now_p ={0,0,0};

        frameid=0;
    };

    void joy_callback(const sensor_msgs::Joy &joy)
    {
        vel.linear.x = joy.axes[1];

        if (joy.axes[1] >= 0)
            vel.angular.z = joy.axes[0];
        else
            vel.angular.z = -1 * joy.axes[0];

        ROS_INFO("listen joy.");
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // ROS_INFO("odom vel %f", msg->twist.twist.linear.x);
        odom = *msg;

        now_p.x=odom.pose.pose.position.x;
        now_p.y=odom.pose.pose.position.y;

        now_p.sin_th = 2 * odom.pose.pose.orientation.z * odom.pose.pose.orientation.w;
        now_p.cos_th = odom.pose.pose.orientation.w * odom.pose.pose.orientation.w
                            - odom.pose.pose.orientation.z * odom.pose.pose.orientation.z;

    }

    void twist_callback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        twist = *msg;
        //ROS_INFO("listen cmd_vel.");
    }

    void make_marker(float green)
    {
        // visualization_msgs::Marker marker;
        marker.header.frame_id = "/odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = frameid;

        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = (ros::Duration)3;

        marker.scale.x = 0.5;
        marker.scale.y = 0.3;
        marker.scale.z = 0.7;
        marker.pose.position.x = now_p.x;
        marker.pose.position.y = now_p.y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = odom.pose.pose.orientation.z;
        marker.pose.orientation.w = odom.pose.pose.orientation.w;
        marker.color.r = 0.0f;
        //marker.color.g = 1.0f;
        marker.color.g = green;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        frameid++;

        //ROS_INFO("make marker.");
    }

    void make_marker_array(float green){
        //float GREEN = (double)rand() / RAND_MAX;
        // int k=0;
        markerarray.markers.resize(PredictTraj.size() );
      
        //ROS_INFO("marker size %d",PredictTraj.size());
        //予測時刻の数だけループ
        for (int j = 0; j < PredictTraj.size(); j++)
        {   
            //ROS_INFO("get loop %d",j);
            // ROS_INFO("marker size %d",PredictTraj.size());
            markerarray.markers[j].header.frame_id = "odom";
            markerarray.markers[j].header.stamp = ros::Time::now();
            markerarray.markers[j].ns = "cmd_vel_display";
            markerarray.markers[j].id = frameid;
            markerarray.markers[j].lifetime = (ros::Duration)2.0;

            // marker_array.markers[j].type = visualization_msgs::Marker::CUBE;
            markerarray.markers[j].type = visualization_msgs::Marker::CUBE;
            markerarray.markers[j].action = visualization_msgs::Marker::ADD;
            markerarray.markers[j].scale.x = 0.1;
            markerarray.markers[j].scale.y = 0.05;
            markerarray.markers[j].scale.z = 0.1;
            markerarray.markers[j].pose.position.x = PredictTraj[j][1];
            markerarray.markers[j].pose.position.y = PredictTraj[j][2];
            markerarray.markers[j].pose.position.z = 0;
            markerarray.markers[j].pose.orientation.x = 0;
            markerarray.markers[j].pose.orientation.y = 0;
            markerarray.markers[j].pose.orientation.z = odom.pose.pose.orientation.z;
            // 半角の公式によりクオータニオンwを求める(符号が区別できていない)
            double w = sqrt((1+PredictTraj[j][4])/2);
           //markerarray.markers[j].pose.orientation.w = odom.pose.pose.orientation.w;
            markerarray.markers[j].pose.orientation.w = w;
            markerarray.markers[j].color.r = 0.0f;
            markerarray.markers[j].color.g = green;
            markerarray.markers[j].color.b = 0.0f;
            markerarray.markers[j].color.a = 1.0f;
            frameid++;

            // visualization_msgs::Marker markers;
            // markers.header.frame_id = "/base_link";
            // markers.header.stamp = ros::Time::now();
            // markers.ns = "cmd_vel_display";
            // markers.id = frameid;
            // markers.lifetime = (ros::Duration)2.0;

            // // marker_array.markers[j].type = visualization_msgs::Marker::CUBE;
            // markers.type = visualization_msgs::Marker::SPHERE;
            // markers.action = visualization_msgs::Marker::ADD;
            // markers.scale.x = 0.05;
            // markers.scale.y = 0.05;
            // markers.scale.z = 0.05;
            // markers.pose.position.x = PredictTraj[j][1];
            // markers.pose.position.y = PredictTraj[j][2];
            // markers.pose.position.z = 0;
            // markers.pose.orientation.x = 0;
            // markers.pose.orientation.y = 0;
            // markers.pose.orientation.z = 0;
            // markers.pose.orientation.w = PredictTraj[j][3];

            // markers.color.r = 0.0f;
            // markers.color.g = green;
            // markers.color.b = 0.0f;
            // markers.color.a = 1.0f;

            // markerarray.markers.push_back(marker);
        }
            //ROS_INFO("finish loop");
    }

    void publish_marker()
    {
        pub_mark.publish(marker);
        ROS_INFO("pub mark");
    }

    void publish_markerarray()
    {
        pub_mark_arr.publish(markerarray);
    }

    void say_nowp(){
        ROS_INFO("nowp x:%f,y:%f,sin:%f,cos:%f",now_p.x,now_p.y,now_p.sin_th,now_p.cos_th);
    }

    // 次の時刻のロボットの位置を計算する関数
    position robot_model(position p_now, double cand_v, double cand_w, double dt)
    {
        position p_next;
        // float radian = 2 * acos(p_now.th);
        
        //cos(th+(wt)/2),sin(th+(wt)/2)を求める
        //double sin_wtdiv2 = add_theorem_sin(p_now.sin_th,sin(cand_w*dt/2),p_now.cos_th, cos(cand_w*dt/2));
        //double cos_wtdiv2 = add_theorem_cos(p_now.sin_th,sin(cand_w*dt/2),p_now.cos_th, cos(cand_w*dt/2));

        // p_next.x = p_now.x + cand_v * p_now.cos_th * dt * cos_wtdiv2;
        // p_next.y = p_now.y + cand_v * p_now.sin_th * dt * sin_wtdiv2;
        
        p_next.x = p_now.x + cand_v * p_now.cos_th * dt ;
        p_next.y = p_now.y + cand_v * p_now.sin_th * dt ;

        // cos(th+wt),sin(th+wt)を求める
        p_next.cos_th = add_theorem_cos(p_now.sin_th, sin(cand_w*dt), p_now.cos_th, cos(cand_w*dt));
        p_next.sin_th = add_theorem_sin(p_now.sin_th, sin(cand_w*dt), p_now.cos_th, cos(cand_w*dt));
        
        //ROS_INFO("sin:%f,cos:%f",p_next.sin_th,p_next.cos_th);        
        // ROS_INFO("sin:%f,cos:%f,traj: x=%f,y=%f",p_next.sin_th,p_next.cos_th,p_next.x, p_next.y);
        return p_next;
    }

    void cal_traj(){
        double dt=0.1;
        double PredictTime=5.0f;
        double time = dt;
        int i=0;


        // 今の位置にもどす
        position p=now_p;

        // double sin = 2 * odom.pose.pose.orientation.z * odom.pose.pose.orientation.w;
        // double cos = odom.pose.pose.orientation.w * odom.pose.pose.orientation.w
        //                     - odom.pose.pose.orientation.z * odom.pose.pose.orientation.z;


        // double roll,yaw,pitch;
        // geometry_quat_to_rpy(roll,yaw,pitch,odom.pose.pose.orientation);
        // double radian=yaw;

        // if(radian >M_PI) radian = radian - 2*M_PI;
        // double deg = radian * 180 / M_PI;
        //ROS_INFO("sin:%f, cos:%f",sin,cos);

        //ROS_INFO("push empty vector");

        
        while (time <= PredictTime)
        {
            // 位置の更新
            position np;
            np = robot_model(p,twist.linear.x,twist.angular.z,dt);

             // 並進速度0.8,角速度0.5のときの軌道
            //ROS_INFO("p_x:%f",p.x);
            //np = robot_model(p,1.0,0.5,dt);
            //ROS_INFO("np_x:%f",np.x);
            //ROS_INFO("push1");
            // timeとpをPredictTrajに格納する処理
            PredictTraj.push_back(std::vector<double>());
            PredictTraj[i].push_back(time);
            PredictTraj[i].push_back(np.x);
            PredictTraj[i].push_back(np.y);
            PredictTraj[i].push_back(np.sin_th);
            PredictTraj[i].push_back(np.cos_th);
            p = np;
            //ROS_INFO("p_x:%f",p.x);
            // ROS_INFO("i=%d,time:%f,traj: x=%f,y=%f", i,time, np.x, np.y);
            
            i++;
            
            if (i == 4)
            {
                //ROS_INFO("i=4,traj: x=%f,y=%f", np.x, np.y);
            }

            time += dt;
        }

    }

    void clearvector(){
        PredictTraj.clear();
    }
};

} // namespace test_marker

int main(int argc, char **argv)
{
    ros::init(argc, argv, "info_marker_publisher1");

    test_marker::PredictTraj_test test;
    float green = 0.0f;

    // ros::NodeHandle nh;

    // //publisher
    // ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 1);

    ros::Rate loop_rate(2);
    while (ros::ok())
    {
        if (green > 1)green = 0;

        
        // test.make_marker(green);
        // test.publish_marker();
        test.cal_traj();

        test.make_marker_array(green);
        test.publish_markerarray();

        green += 0.1;
        // marker_pub.publish(marker);

        test.say_nowp();

        ros::spinOnce();
        loop_rate.sleep();

        test.clearvector();
    }
    return 0;
}