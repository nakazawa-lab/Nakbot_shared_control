#include"my_robo_simulation/my_robo_drive.h"
#include<vector>
#include<algorithm>

my_robo::my_robo(){
    spec.get_spec_param(n,spec);
    sensor.count = 0;
    sensor.countj = 0;

    // 最後の引数はメッセージを受け取ったときに呼び出す関数がクラスの中にある際に、その【実体】を指定する
    sub_odom = n.subscribe("/odom", 10, &my_robo_sensor::cb_odom, &(this->sensor));
    sub_lrf = n.subscribe("/scan", 10, &my_robo_sensor::cb_lrf,&(this->sensor));
    sub_joy = n.subscribe("/joy", 10, &my_robo_sensor::cb_joy,&(this->sensor));
    pub_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    // pubする速度指令の初期化
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

    ros::Rate rate(5);

    // // センサデータを処理するための変数群
    int range_num,center_point,point_70;

    const float DegToPoint = 1024 / 360 ;
    int deg10point =10 * DegToPoint;


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

            // ROS_INFO("spec.x: %f",spec.x_max_vel);
            // ROS_INFO("spec.z: %f\n",spec.z_max_ang);

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

            ROS_INFO("distance:%f\n",sensor.latest_scan.ranges[center_point]);

            // もし、正面方向に0.8m以下の距離に障害物があり、joyから前に進む司令があったとき、joyの司令を0.6倍する
            // もし、正面方向に0.4m以下の距離に障害物があり、joyから前に進む司令があったとき、joyの司令を無視して停止する

            for(int i = center_point-deg10point * 7; i <= center_point + deg10point * 7; i += deg10point){
                if(sensor.latest_scan.ranges[i]<0.8 && sensor.latest_scan.ranges[i]>0.4 
                        && sensor.joy_cmd_vel[0]>0)
                {
                    ROS_INFO("close.");
                    vel.linear.x *= 0.6;
                    break;
                }
                else if(sensor.latest_scan.ranges[i] <= 0.4 && sensor.joy_cmd_vel[0]>0)
                {
                    ROS_INFO("too close.stopped.");
                    vel.linear.x=0.0;
                    break;
                }
            }

            //ROS_INFO("I heard: [%f]", static_cast<float>(latest_scan.ranges[range_num/2]));
            pub_cmd.publish(vel);
            ROS_INFO("pub vel.");
        }


        rate.sleep();
    }

    // ループの最後にはpredicttrajectoryやcmdcandidateなどを消去する

}

// スペック上の最大加速度と今の速度からDynamicWindowを求める vector<vector<float>>型のDWA.CanVelに格納される
// CandVel[i][0] = v [i][1]=w
void my_robo::cal_DWA(){
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


// 予測軌道を計算する関数 内部でmy_robo.robot_modelを呼びだす
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

    // 速度コストのsaturation係数を求める
double my_robo::cal_vel_sat( std::vector<std::vector<float>>& CandVel,  std::vector<std::vector<float>>& PredictTraj ){
    int num = CandVel.size();

    // 予測時刻あとに衝突しないような軌道を探す

    // PredictTrajから衝突判定？
    // PredictTrajの最後の座標がレーザのd,thetaから計算される位置より大きかったら衝突

    double sum = 0;
    double clearance;
    for(int i = 0; i<num; i++){
    int j = PredictTraj.size();

    // 予測時刻あとの座標はPredictTraj[j-1][x][y][th]
    // クリアランス計算
    // クリアランスをどうやって求めるか？
    //clearance= ;
    sum += clearance;
    } 
}

// 障害物との入力相当距離(SharedDWAのd_Uにあたる)を求め,
// CandVelの3番目の要素にd_Uをプッシュバックする
void my_robo::cal_Dist(std::vector<std::vector<float>>& CandVel, DWA_var& DWA, sensor_msgs::LaserScan& scan, my_robo_spec& spec){
    float adm;
    float t=0.05,time=0;;
    float v,w,r;            // rは曲率半径
    double d_lin,d_ang;     // sharedDWAの論文における定義
    float d,th;             // レーザセンサに対応した距離と角度
    float d_U;

    // <<必要な情報>>
    // 候補速度ベクトル

    // センサデータから計算される,障害物の位置(scan.data)

    // <<やること>>
    // 1 (v,w)を一つ取り出す
    int candSize = CandVel.size();

    // 5 1から4をすべての候補に対して繰り返す
    //   ただ、(v,w)がともに0のときは距離1となる
    for(int i=0; i<candSize; i++){
        v = CandVel[i][0];
        w = CandVel[i][1];
        
         // 4-b-1 予測時刻を増やしても衝突せず、センサ範囲を超えたら、その(v,w)でのd_uは1とする
        if(v == 0 && w == 0){
            d_U = 1;
            CandVel[i].push_back(d_U);
        }
        else{
            r =  v / w;
            // 進んだ距離(d_lin,d_ang)を計算し、dとthに変換してレーザセンサの値と比較、衝突判定する
            // 2 予測時刻をどんどん増やしていってその時のdとthetaを求める(センサの値と比較するため)
            //   予測時刻は衝突するのがわかればいいのでざっくり増やしていく
            //   ↑d_linとd_angを求める必要があるので細かいほうが良い
            while(true){
            time += t;
            d_lin = v * time;
            d_ang = w * time;
            d = sqrt(2*r*r * (1 - cos(d_ang)));     // 余弦定理
            th = acos((v * sin(d_ang)) / (d * w) );
            
            // 3 thetaを挟むようなLRFの計測値を2つ取り出す
            // indexとindex+1
            int index = 0;
            float deg = index * scan.angle_increment;
            while(abs(deg-th)<scan.angle_increment){
                index++;
                if(index > scan.ranges.size() - 2) break;
            }

            // 4 その2つの計測値のdの値と計算された時のdの値を求める
            // 4-a-1 計測値のどちらかより計算値が大きければ、衝突と判定、その時のd_lとd_aを求める
            if(d > scan.ranges[index] || d > scan.ranges[index + 1]){
                // 4-a-2 v_inevとw_inevを求める
                float v_inev = sqrt(2 * spec.x_min_acc * d_lin);
                float w_inev = sqrt(2 * spec.z_min_acc * d_ang);

                // 4-a-3 d_uを求める
                if(v >= v_inev || w >= w_inev){
                    d_U = 0;
                } 
                else{
                    d_U = std::min((v_inev-v)/v_inev ,(w_inev - w) / w_inev);
                }
                CandVel[i].push_back(d_U);
            }
            else{

            }
            }
        }
    }
}

// 評価関数を計算する Dはsat係数.あらかじめ計算しておく
double my_robo::cal_J_sharedDWA(double adm, float u[2], float ug[2], double D, double vmax){ 

    // 受け取るもの
    // 候補速度ベクトル
    // ジョイからの速度ベクトル
    // 最大速度
    // adm
    // 速度コストのsat係数

    // 角度コストの計算
    double head = abs(atan2(u[0],u[1]) - atan2(ug[0], ug[1])) / M_PI;

    // 速度コストの計算
    double velocity = (1 - D) * (abs(u[0]-ug[0])) / vmax + D * abs(u[0]) / vmax;

    // 最終的なコストの計算
    double cost = adm + (1- adm) * ( DWA.k_heading * head + DWA.k_velocity * velocity);
    // return cost;
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
