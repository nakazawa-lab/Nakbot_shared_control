#include"my_robo_simulation/my_robo_drive.h"
#include<vector>

my_robo::my_robo(){
    spec.get_spec_param(n,spec);
    sensor.count = 0;
    sensor.countj = 0;

    sub_odom = n.subscribe("/odom", 10, &my_robo_sensor::cb_odom, &(this->sensor));
    sub_lrf = n.subscribe("/laserscan", 10, &my_robo_sensor::cb_lrf,&(this->sensor));
    sub_joy = n.subscribe("/joy", 10, &my_robo_sensor::cb_joy,&(this->sensor));
    pub_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    // pubする速度司令の初期化
    vel.linear.x=0;
    vel.angular.z=0;
    //sub_lrf=n.subscribe("/laserscan", 10, chatterCallback);

    // //ros::Publisher cmd_pub=n.advertise<geometry_msgs::Twist>();
    // count=0;

    // cmd_vel.linear.x=0;
    // cmd_vel.angular.z=0;

    // sub_lrf=n.subscribe("/laserscan", 10, &my_robo::cb_lrf,this);
    // sub_odom=n.subscribe("/odom",10,&my_robo::cb_odom,this);
    // sub_joy=n.subscribe("/joy",10,&my_robo::cb_joy,this);
    // pub_cmd=n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    // // 速度、加速度に関するパラメータの取得
    // get_vel_acc_Param();

    // 引用"odomの中で方位を表す、クオータニオンを初期化"
    //odom.pose.pose.orientation.w = 1.0;

}



void my_robo::controlloop(){
    ROS_INFO("control loop start.");

    ros::Rate rate(2);

    // // センサデータを処理するための変数群
    int range_num,center_point,point_70;



    while(ros::ok()){
        ros::spinOnce();
        ROS_INFO("get loop.");
        

        if(sensor.countj != 0){
            // joyからの速度司令の計算
            // sensor.joy_cmd_vel[0] 速度
            // sensor.joy_cmd_vel[1] 角速度

            // ジョイスティック左側
            // 上→axes[1]の正方向
            // 左→axes[0]の正方向
            sensor.joy_cmd_vel[0] = spec.x_max_vel * sensor.joy.axes[1];
            //cmd_vel.linear.y =joy_msg.axes[2];

            if(sensor.joy.axes[1]>=0) sensor.joy_cmd_vel[1] = spec.z_max_ang * sensor.joy.axes[0];
            else sensor.joy_cmd_vel[1] = -1 * spec.z_max_ang * sensor.joy.axes[0];

            ROS_INFO("spec.x: %f",spec.x_max_vel);
            ROS_INFO("spec.z: %f\n",spec.z_max_ang);

            ROS_INFO("x_joy: %f",sensor.joy_cmd_vel[0]);
            ROS_INFO("z_ang: %f\n",sensor.joy_cmd_vel[1]);

            vel.linear.x = sensor.joy_cmd_vel[0];
            vel.angular.z = sensor.joy_cmd_vel[1];
        }

        // LaserScanメッセージをすでに受け取っている場合
        if(sensor.latest_scan.ranges.size()>0){

            // lrfデータに関する初期化
            if(sensor.count==0){   
                range_num=static_cast<int>(sensor.latest_scan.ranges.size());                // 取得した点の数
                center_point=range_num/2;                                                   // 正面のスキャン点番号
                point_70=(M_PI/(2*sensor.latest_scan.angle_increment))*(7/9);               //80度となる点の数
                sensor.count++;
                ROS_INFO("lrf initialize commplete.");
            }


            // もし、正面方向に0.7m以下の距離に障害物があり、joyから前に進む司令があったとき、joyの司令を無視して停止する
            //for(int i = center_point-point_70; i < center_point + point_70; i += 10){
                if(sensor.latest_scan.ranges[center_point]<0.7 && sensor.joy_cmd_vel[0]>0){
                    ROS_INFO("too close. distance:%f",sensor.latest_scan.ranges[center_point]);
                    vel.linear.x=0.0;
                    ROS_INFO("stopped.");
                }
            //}

            //ROS_INFO("I heard: [%f]", static_cast<float>(latest_scan.ranges[range_num/2]));
            pub_cmd.publish(vel);
            ROS_INFO("pub vel.");
        }


        rate.sleep();
    }

    // ループの最後にはpredicttrajectoryやcmdcandidateなどを消去する

}

void my_robo::cal_DWA(my_robo_spec& spec, my_robo_sensor& sensor, DWA_var& DWA){
    ROS_INFO("Calculate DWA start.");
    // 現在の速度(odonm)から刻み時間後に到達可能な最大、最小速度を求める。それを一時保存しておく
    float max_dwa_vel = sensor.odom.twist.twist.linear.x  + spec.x_max_acc * DWA.dt;
    float min_dwa_vel = sensor.odom.twist.twist.linear.x  - spec.x_max_acc * DWA.dt;
    float max_dwa_ang = sensor.odom.twist.twist.angular.z  + spec.z_max_acc * DWA.dt;
    float min_dwa_ang = sensor.odom.twist.twist.angular.z  - spec.z_max_acc * DWA.dt;

    // それらがスペックを超えていたら、スペックの限度の値に保税する
    if(max_dwa_vel > spec.x_max_vel)    max_dwa_vel = spec.x_max_vel;
    if(min_dwa_vel > spec.x_min_vel)    min_dwa_vel = spec.x_min_vel;
    if(max_dwa_ang > spec.z_max_ang)    max_dwa_ang = spec.z_max_ang;
    if(min_dwa_ang > spec.z_min_ang)    min_dwa_ang = spec.z_min_ang;
    
    // 幅に関して、解像度だけ繰り返して組み合わせを作る
    float f = min_dwa_vel;
    float g = min_dwa_ang;
    int i=0;

    ROS_INFO("puch back candidates.");
    while(true){
        float g = min_dwa_ang;

        // ウインドウを角速度方向に捜査
        while(true){
            DWA.CandVel.push_back(std::vector<float>());
            DWA.CandVel[i].push_back(f);        //[i][0]に速度要素
            DWA.CandVel[i].push_back(g);        //[i][1]に角速度要素
            
            i++;
            g += spec.ang_res;
            if(g>max_dwa_ang)   break;
        }
            //　速度方向に1つ増やし、次のループでまた角速度方向に捜査
            f += spec.vel_res;
            if(f>max_dwa_vel)   break;
    }
}



void my_robo::cal_predict_position(DWA_var& DWA, my_robo_spec spec, std::vector<std::vector<float>> CandVel,
      std::vector<std::vector<float>>&  PredictTraj, my_robo_sensor& sensor){
        float time = DWA.dt;
        
        // 候補の数
        int num_cand = CandVel.size();
        ROS_INFO("candidate size is %d",num_cand);
        position p{ sensor.odom.pose.pose.position.x, sensor.odom.pose.pose.position.y, sensor.odom.pose.pose.orientation.w};
        ROS_INFO("x=%f,y=%f,th=%f",p.x,p.y,p.th);
        int i = 0;
        while(time < DWA.PredictTime){
            position p = robot_model(p, CandVel[i][0] ,CandVel[i][1] , DWA.dt,  sensor);
            PredictTraj.push_back(std::vector<float>());
            // timeとpをPredictTrajに格納する処理
            // 予測軌道 [時刻][x][y][theta]
            PredictTraj[i].push_back(time);
            PredictTraj[i].push_back(p.x);
            PredictTraj[i].push_back(p.y);
            PredictTraj[i].push_back(p.th);
            i++;
            time += DWA.dt;
        }
}


int main(int argc, char **argv){
  ros::init(argc, argv, "my_robo_drive");

  // ros::NodeHandle n;
  //ros::Subscriber sub = n.subscribe("/laserscan", 10, chatterCallback);

  my_robo robot;
  robot.controlloop();

  //ros::Rate loop_rate(1);
  //ros::spin();

  return 0;
}
