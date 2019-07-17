#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <math.h>
#include<vector>

namespace test_marker
{
// 次の位置を格納する箱
struct position
{
    float x;
    float y;
    float th;
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
    std::vector<std::vector<float>> PredictTraj;

public:
    PredictTraj_test()
    {
        //sub_joy = n.subscribe("joy", 10, &PredictTraj_test::joy_callback,this);
        pub_mark = n.advertise<visualization_msgs::Marker>("marker", 1);
        pub_mark_arr = n.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
        sub_odom = n.subscribe("/odom", 10, &PredictTraj_test::odom_callback, this);
        sub_cmd_vel = n.subscribe("/cmd_vel", 10, &PredictTraj_test::twist_callback, this);
        now_p ={0,0,0};
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
        now_p.th=odom.pose.pose.orientation.w;

    }

    void twist_callback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        twist = *msg;
        //ROS_INFO("listen cmd_vel.");
    }

    void make_marker(int frameid, float green)
    {
        // visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_footprint";
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
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = now_p.th;
        marker.color.r = 0.0f;
        //marker.color.g = 1.0f;
        marker.color.g = green;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        //ROS_INFO("make marker.");
    }

    void make_marker_array(int frameid, float green){
        //float GREEN = (double)rand() / RAND_MAX;
        // int k=0;
        //markerarray.markers.resize(PredictTraj.size() );
      
        ROS_INFO("marker size %d",PredictTraj.size());
        //予測時刻の数だけループ
        for (int j = 0; j < PredictTraj.size(); j++)
        {   
            
            
            // ROS_INFO("get loop %d",j);
            // ROS_INFO("marker size %d",PredictTraj.size());
            // markerarray.markers[j].header.frame_id = "base_link";
            // markerarray.markers[j].header.stamp = ros::Time::now();
            // markerarray.markers[j].ns = "cmd_vel_display";
            // markerarray.markers[j].id = j;
            // markerarray.markers[j].lifetime = (ros::Duration)2.0;

            // // marker_array.markers[j].type = visualization_msgs::Marker::CUBE;
            // markerarray.markers[j].type = visualization_msgs::Marker::SPHERE;
            // markerarray.markers[j].action = visualization_msgs::Marker::ADD;
            // markerarray.markers[j].scale.x = 0.05;
            // markerarray.markers[j].scale.y = 0.05;
            // markerarray.markers[j].scale.z = 0.05;
            // markerarray.markers[j].pose.position.x = PredictTraj[j][1];
            // markerarray.markers[j].pose.position.y = PredictTraj[j][2];
            // markerarray.markers[j].pose.position.z = 0;
            // markerarray.markers[j].pose.orientation.x = 0;
            // markerarray.markers[j].pose.orientation.y = 0;
            // markerarray.markers[j].pose.orientation.z = 0;
            // markerarray.markers[j].pose.orientation.w = PredictTraj[j][3];

            // markerarray.markers[j].color.r = 0.0f;
            // markerarray.markers[j].color.g = GREEN;
            // markerarray.markers[j].color.b = 0.0f;
            // markerarray.markers[j].color.a = 1.0f;
            // k++;

            visualization_msgs::Marker markers;
            markers.header.frame_id = "base_link";
            markers.header.stamp = ros::Time::now();
            markers.ns = "cmd_vel_display";
            markers.id = frameid;
            markers.lifetime = (ros::Duration)2.0;

            // marker_array.markers[j].type = visualization_msgs::Marker::CUBE;
            markers.type = visualization_msgs::Marker::SPHERE;
            markers.action = visualization_msgs::Marker::ADD;
            markers.scale.x = 0.05;
            markers.scale.y = 0.05;
            markers.scale.z = 0.05;
            markers.pose.position.x = PredictTraj[j][1];
            markers.pose.position.y = PredictTraj[j][2];
            markers.pose.position.z = 0;
            markers.pose.orientation.x = 0;
            markers.pose.orientation.y = 0;
            markers.pose.orientation.z = 0;
            markers.pose.orientation.w = PredictTraj[j][3];

            markers.color.r = 0.0f;
            markers.color.g = green;
            markers.color.b = 0.0f;
            markers.color.a = 1.0f;

            markerarray.markers.push_back(marker);
        }
            ROS_INFO("finish loop");
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
        ROS_INFO("nowp x:%f,y:%f,th:%f",now_p.x,now_p.y,now_p.th);
    }

    // 次の時刻のロボットの位置を計算する関数
    position robot_model(position p_now, float cand_v, float cand_w, float dt)
    {
        position p_next;
        p_next.x = p_now.x + cand_v * cos(p_now.th) * dt * cos(p_now.th + dt / 2);
        p_next.y = p_now.y + cand_v * sin(p_now.th) * dt * sin(p_now.th + dt / 2);
        
        p_next.th = p_now.th + cand_w * dt;
        return p_next;
    }

    void cal_traj(){
        float dt=0.1;
        float PredictTime=3.0f;
        float time = dt;
        int i=0;


        // 今の位置にもどす
        position p=now_p;

        //ROS_INFO("push empty vector");

        while (time <= PredictTime)
        {
            // 位置の更新
            position np;
            np = robot_model(p,twist.linear.x,twist.angular.z,dt);
            //ROS_INFO("push1");
            // timeとpをPredictTrajに格納する処理
            PredictTraj.push_back(std::vector<float>());
            PredictTraj[i].push_back(time);
            PredictTraj[i].push_back(np.x);
            PredictTraj[i].push_back(np.y);
            PredictTraj[i].push_back(np.th);

            p = np;

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
    int i = 0;
    float green = 0.0f;

    // ros::NodeHandle nh;

    // //publisher
    // ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 1);

    ros::Rate loop_rate(2);
    while (ros::ok())
    {
        if (green > 1)
            green = 0;

        
        //test.make_marker(i, green);
        //test.publish_marker();
        test.cal_traj();

        test.make_marker_array(i,green);
        test.publish_markerarray();

        i++;
        green += 0.1;
        // marker_pub.publish(marker);

        test.say_nowp();

        ros::spinOnce();
        loop_rate.sleep();

        test.clearvector();
    }
    return 0;
}