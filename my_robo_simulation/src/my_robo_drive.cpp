#include"my_robo_simulation/my_robo_drive.h"
#include<vector>
#include<algorithm>

#define DEG2RAD M_PI/180
#define RAD2DEG 180/M_PI

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

    // joyの初期化
    sensor.joy_cmd_vel[0]=0;
    sensor.joy_cmd_vel[1]=0;

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


float cal_euclid(float x0,float y0, float x1,float y1){
    return sqrt( (x0-x1)*(x0-x1) + (y0-y1)*(y0-y1) );
}


void my_robo::controlloop(){
    ROS_INFO("control loop start.");

    ros::Rate rate(5);

    // // センサデータを処理するための変数群
    int range_num,center_point,point_70;

    const float DegToPoint = 1024 / 360 ;
    int deg10point =10 * DegToPoint;

    int flag =0;


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
            if(flag==0){   
                range_num=sensor.latest_scan.ranges.size();                // 取得した点の数
                center_point=range_num/2;                                                   // 正面のスキャン点番号
                point_70=(M_PI/(2*sensor.latest_scan.angle_increment))*(7/9);               //80度となる点の数
                sensor.count++;
                ROS_INFO("lrf initialize commplete.");
                ROS_INFO("center_point:%d",center_point);
                flag++;
            }

            ROS_INFO("distance:%f\n",sensor.latest_scan.ranges[center_point]);

            // もし、正面方向に0.8m以下の距離に障害物があり、joyから前に進む司令があったとき、joyの司令を0.6倍する
            // もし、正面方向に0.4m以下の距離に障害物があり、joyから前に進む司令があったとき、joyの司令を無視して停止する

            for(int i = center_point-deg10point * 6; i <= center_point + deg10point * 6; i += deg10point){
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
                    ROS_INFO("range:%f",sensor.latest_scan.ranges[i]);
                    // ROS_INFO("angle:%f",);
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
    //ROS_INFO("Calculate DWA start.");
    // 現在の速度(odonm)から刻み時間後に到達可能な最大、最小速度を求める。それを一時保存しておく
    float max_dwa_vel = sensor.odom.twist.twist.linear.x  + spec.x_max_acc * DWA.dt;
    float min_dwa_vel = sensor.odom.twist.twist.linear.x  + spec.x_min_acc * DWA.dt;
    float max_dwa_ang = sensor.odom.twist.twist.angular.z  + spec.z_max_acc * DWA.dt;
    float min_dwa_ang = sensor.odom.twist.twist.angular.z  + spec.z_min_acc * DWA.dt;

    // それらがスペックを超えていたら、スペックの限度の値に保税する
    if(max_dwa_vel > spec.x_max_vel)    max_dwa_vel = spec.x_max_vel;
    if(min_dwa_vel < spec.x_min_vel)    min_dwa_vel = spec.x_min_vel;
    if(max_dwa_ang > spec.z_max_ang)    max_dwa_ang = spec.z_max_ang;
    if(min_dwa_ang < spec.z_min_ang)    min_dwa_ang = spec.z_min_ang;
    //ROS_INFO("DWA:%f,%f,%f,%f.",max_dwa_vel,max_dwa_ang,min_dwa_vel,min_dwa_ang);

    // 幅に関して、解像度だけ繰り返して組み合わせを作る
    float f = min_dwa_vel;
    float g = min_dwa_ang;
    int i=0;

    //ROS_INFO("puch back candidates.");

    // はじめに、現在の速度を入れる
    DWA.CandVel.push_back(std::vector<float>());
    DWA.CandVel[i].push_back(sensor.odom.twist.twist.linear.x);        //[i][0]に速度要素
    DWA.CandVel[i].push_back(sensor.odom.twist.twist.angular.z);        //[i][1]に角速度要素
    //ROS_INFO("CandVel:%f,%f",DWA.CandVel[i][0],DWA.CandVel[i][1]);
    i++;
    while(true){
        float g = min_dwa_ang;

        // ウインドウを角速度方向に捜査
        while(true){
            

            // 後ろ方向に進む候補に対しては考えない
            if (f >= 0)
            {
                DWA.CandVel.push_back(std::vector<float>());
                DWA.CandVel[i].push_back(f); //[i][0]に速度要素
                DWA.CandVel[i].push_back(g); //[i][1]に角速度要素
                i++;
            }
            //ROS_INFO("CandVel:%f,%f",DWA.CandVel[i][0],DWA.CandVel[i][1]);

            
            g += spec.ang_res;
            if(g>max_dwa_ang)   break;
        }
            //　速度方向に1つ増やし、次のループでまた角速度方向に捜査
            f += spec.vel_res;
            if(f>max_dwa_vel)   break;
    }
    //ROS_INFO("candSize:%d.",DWA.CandVel.size());
}


// 予測軌道を計算する関数 内部でmy_robo.robot_modelを呼びだす
void my_robo::cal_predict_position(){
    float time = DWA.dt;

    ROS_INFO("candidate size is %d", DWA.CandVel.size());

    // すべての候補に対して
    for (int i = 0; i < DWA.CandVel.size(); i++)
    {
        //ROS_INFO("loop %d / %d", i, DWA.CandVel.size());
        //初期化
        time = DWA.dt;

        // 今の位置にもどす
        position p{sensor.odom.pose.pose.position.x, sensor.odom.pose.pose.position.y, sensor.odom.pose.pose.orientation.w};
        //ROS_INFO("x=%f,y=%f,th=%f",p.x,p.y,p.th);

        // indexの挿入
        DWA.PredictTraj.push_back(std::vector<std::vector<float>>());
        //ROS_INFO("push empty vector");


        while (time < DWA.PredictTime)
        {
            // 位置の更新
            position np;
            np = robot_model(p, DWA.CandVel[i][0], DWA.CandVel[i][1], DWA.dt, sensor);
            //ROS_INFO("push1");
            // timeとpをPredictTrajに格納する処理
            // 予測軌道 [index][時刻][x,y,theta]
            // [0][0][0]時刻,[0][0][1]x,[0][0][2]y...
            DWA.PredictTraj.back().push_back(std::vector<float>());
            DWA.PredictTraj.back().back().push_back(time);
            DWA.PredictTraj.back().back().push_back(np.x);
            DWA.PredictTraj.back().back().push_back(np.y);
            DWA.PredictTraj.back().back().push_back(np.th);

            p = np;

            if (i == 4)
            {
                ROS_INFO("i=4,traj: x=%f,y=%f", np.x, np.y);
            }

            time += DWA.dt;
        }
        //ROS_INFO("TrajSize%d", DWA.PredictTraj.back().size());
    }
    //ROS_INFO("candidate size is %d",DWA.CandVel.size());
    //ROS_INFO("finish cal traj\n");
}

    // 速度コストのsaturation係数を求める
double my_robo::cal_vel_sat(){
    int num = DWA.CandVel.size();
    //ROS_INFO("start cal sat.");
    // 予測時刻あとに衝突しないような軌道を探す
    // obsからクリアランスを求める
    double clearance;
    // 予測自国内に衝突しない候補の数を求める
    int count =0;

    //ROS_INFO("num:%d",num);
    if(!DWA.isCollision.empty()){

    // 候補速度に対して
    for(int i=0;i<num;i++){
        //候補速度の中で、衝突しない速度を選ぶ

            if(!DWA.isCollision[i]){
                count++;
                float dist=10000000000;
                //ROS_INFO("i=%d",i);

                // Dの計算
                // predictTreajの一番最後と、obsの距離を比べる
                // DWA.predictTraj[j-1]とセンサ障害物の最大の距離を求める
                //ROS_INFO("sensor_size:%d",sensor.obs.size());
                if(sensor.obs.size()==0){
                    ROS_INFO("no obs");
                    dist =8;
                }else
                {
                    for(int k=0; k < sensor.obs.size();k++){
                        int a = DWA.PredictTraj[i][0].size();
                        
                        //ROS_INFO("trajSize:%d",a);
                        //ROS_INFO("euclid");
                        // ROS_INFO("dist:%f",DWA.PredictTraj[i][a-1][0]);
                        // ROS_INFO("dist:%f",DWA.PredictTraj[i][a-1][1]);
                        // ROS_INFO("dist:%f",sensor.obs[k][0]);
                        // ROS_INFO("dist:%f",sensor.obs[k][1]);
                        float temp =cal_euclid(DWA.PredictTraj[i][a-1][0],sensor.obs[k][0],DWA.PredictTraj[i][a-1][1],sensor.obs[k][1]);
                        //ROS_INFO("finish euclid. dist:%f",temp);
                        if(temp < dist){
                            dist = temp;
                        }
                    }
                }
                //ROS_INFO("final dist:%f",dist);
                clearance +=dist;
            }
       // }
        
        }
    }
    ROS_INFO("number not collision:%d",count);

    // 予測時刻あとの座標はPredictTraj[j-1][x][y][th]
    // クリアランス計算
    // クリアランスをどうやって求めるか？
    double D = clearance / count ;
    ROS_INFO("D:%lf\n",D);
    return D;
}

// 障害物との入力相当距離(SharedDWAのd_Uにあたる)を求め,
// CandVelの3番目の要素にd_Uをプッシュバックする
void my_robo::cal_Dist(){
    float adm;
    float t=0.05,time;
    float v,w,r;            // rは曲率半径
    double d_lin,d_ang;     // sharedDWAの論文における定義
    float d,th;             // レーザセンサに対応した距離と角度
    float d_U;

    // <<必要な情報>>
    // 候補速度ベクトル

    // センサデータから計算される,障害物の位置(scan.data)

    // <<やること>>
    //ROS_INFO("start cal dist.");
    int candSize = DWA.CandVel.size();
    // 1 (v,w)を一つ取り出す
    // 5 1から4をすべての候補に対して繰り返す
    //   ただ、(v,w)がともに0のときは距離1となる
    //ROS_INFO("candsize%d",candSize);
    for(int i=0; i<candSize; i++){
        v = DWA.CandVel[i][0];
        w = DWA.CandVel[i][1];
        //ROS_INFO("i=%d",i);
        time =0;
         // 4-b-1 予測時刻を増やしても衝突せず、センサ範囲を超えたら、その(v,w)でのd_uは1とする
        if(v == 0 && w == 0){
            d_U = 1;
            DWA.CandVel[i].push_back(d_U);
        }
        else{
            r =  v / w;
            // 進んだ距離(d_lin,d_ang)を計算し、dとthに変換してレーザセンサの値と比較、衝突判定する
            // 2 予測時刻をどんどん増やしていってその時のdとthetaを求める(センサの値と比較するため)
            //   予測時刻は衝突するのがわかればいいのでざっくり増やしていく
            //   ↑d_linとd_angを求める必要があるので細かいほうが良い
            while(true){
            time += t;

            if(time>5){
                DWA.isCollision.push_back(false);
                d_U = 1;
                DWA.CandVel[i].push_back(d_U);
                ROS_INFO("i:%d", i);
                ROS_INFO("timeout while cal D_U.break\n");
                break;
            }

            d_lin = v * time;
            d_ang = w * time;
            d = sqrt(2*r*r * (1 - cos(d_ang)));     // 余弦定理
            th = acos((v * sin(d_ang)) / (d * w) );

            // ROS_INFO("c2.");
            // ROS_INFO("d:%f.",d);
            // ROS_INFO("th:%f.",th);

            // 3 thetaを挟むようなLRFの計測値を2つ取り出す
            // indexとindex+1
            int index = 0;
            float index_th = index * sensor.latest_scan.angle_increment;
            // indexが数えやすいよう、最小測定角を0度としたときの角度に直す
            float th_l = (sensor.latest_scan.angle_max - sensor.latest_scan.angle_min)/2 +th;

            while(abs(index_th-th_l)>sensor.latest_scan.angle_increment){
                //ROS_INFO("index:%d.",index);
                index++;
                index_th = index * sensor.latest_scan.angle_increment;
                //ROS_INFO("c4.");
                if(index > sensor.latest_scan.ranges.size() - 2) break;
            }

            //ROS_INFO("range_size:%d",sensor.latest_scan.ranges.size());   //683

            // ROS_INFO("time%f",time);
             //ROS_INFO("index:%d",index);
             //ROS_INFO("d:%f",d);
             //ROS_INFO("laser:%f",sensor.latest_scan.ranges[index]);

            // 4 その2つの計測値のdの値と計算された時のdの値を求める
            // 4-a-1 計測値のどちらかより計算値が大きければ、衝突と判定、その時のd_lとd_aを求める
            if(d > sensor.latest_scan.ranges[index] || d > sensor.latest_scan.ranges[index + 1]){
                //ROS_INFO("spec x_min_acc z_min_acc:%f,%f",spec.x_min_acc,spec.z_min_acc);
                // 4-a-2 v_inevとw_inevを求める
                float v_inev = sqrt(abs(2 * spec.x_min_acc * d_lin));
                float w_inev = sqrt(abs(2 * spec.z_min_acc * d_ang));

                ROS_INFO("v_inev%f",v_inev);
                ROS_INFO("w_inev%f",w_inev);
                ROS_INFO("v%f",v);
                ROS_INFO("w%f",w);
                // ROS_INFO("d:%f.",d);
                // ROS_INFO("th:%f.",th);
                // ROS_INFO("time:%f.",time);
                // 4-a-3 d_uを求める
                if(v >= v_inev || w >= w_inev)d_U = 0; 
                else d_U = std::min(abs(v_inev- v / v_inev) ,abs( w_inev - w / w_inev));

                DWA.CandVel[i].push_back(d_U);
                ROS_INFO("i:%d", i);
                ROS_INFO("collision.");
                ROS_INFO("d_U%f",d_U);

                // Dの計算に使うので、もし予測時刻内の衝突であればフラグを立てる
                if(time < DWA.PredictTime){
                    ROS_INFO("collision in predict time.");
                    ROS_INFO("break\n.");
                    DWA.isCollision.push_back(true);
                }else{
                    DWA.isCollision.push_back(false);
                    ROS_INFO("collision out of predict time.");
                    ROS_INFO("break\n.");
                }
                break;
            }
            else{
                // ループを続ける
                //ROS_INFO("no collision.");
            }
            }
        }
    }
}

// 評価関数を計算する Dはsat係数.あらかじめ計算しておく
// 最小の評価関数の添字を返す
int my_robo::cal_J_sharedDWA(double D){ 
    float adm;
    double cost=1000;
    int index=0;

    for(int i=0;i<DWA.CandVel.size();i++){
        adm = 1 - DWA.CandVel[i][2];
        double head;
        // 角度コストの計算
        if(sensor.joy_cmd_vel[1]==0){      
            head = abs (DWA.CandVel[i][1]);
        }
        else{
            head = abs(atan2(DWA.CandVel[i][0],DWA.CandVel[i][1]) - atan2(sensor.joy_cmd_vel[0], sensor.joy_cmd_vel[1])) / M_PI;
        }
        // 速度コストの計算
        // double velocity = (1 - D) * (abs(DWA.CandVel[i][0]-sensor.joy_cmd_vel[0])) / spec.x_max_vel + D * abs(DWA.CandVel[i][0]) / spec.x_max_vel;
        double velocity = (abs(DWA.CandVel[i][0]-sensor.joy_cmd_vel[0])) / spec.x_max_vel ;

        // 最終的なコストの計算
        double temp = adm + (1- adm) * ( DWA.k_heading * head + DWA.k_velocity * velocity);

        // ROS_INFO("CandVel:%f,%f",DWA.CandVel[i][0],DWA.CandVel[i][1]);
        // ROS_INFO("joy:%f",sensor.joy_cmd_vel[0],sensor.joy_cmd_vel[1]);
        // ROS_INFO("D:%lf",D);


         if(temp<cost){
            //ROS_INFO(co)
            cost=temp;
            index=i;

            ROS_INFO("i:%d", i);
            // ROS_INFO("now vel:%f, %f", sensor.odom.twist.twist.linear.x, sensor.odom.twist.twist.angular.z); 
            // ROS_INFO("joy vel:%f, %f", sensor.joy_cmd_vel[0], sensor.joy_cmd_vel[1]);
            // ROS_INFO("cand vel:%f, %f", DWA.CandVel[i][0], DWA.CandVel[i][1]);
            // ROS_INFO("joy atan2:%f", atan2(sensor.joy_cmd_vel[0], sensor.joy_cmd_vel[1]));
            // ROS_INFO("cand atan2:%f ", atan2(DWA.CandVel[i][0],DWA.CandVel[i][1]));

            ROS_INFO("adm:%lf", adm);
            ROS_INFO("head:%lf", head);
            ROS_INFO("velocity:%lf", velocity);
            ROS_INFO("cost:%lf\n", temp);
        }
    }
    // ROS_INFO("finish cal_J");
    ROS_INFO("index:%d\n",index);
    ROS_INFO("pubvel:%f,%f",DWA.CandVel[index][0],DWA.CandVel[index][1]);
    return index;
}


void my_robo::DWAloop(){
  ROS_INFO("control loop start.");

    ros::Rate rate(DWA.looprate);


    while(ros::ok()){
        ros::spinOnce();
        ROS_INFO("get DWA loop.");


        if(sensor.latest_scan.ranges.size()==0){
            ROS_INFO("no LRF data.waiting...");
            usleep(1000);
        }
        else{
            check_joy();
            ROS_INFO("finish check joy\n");

            cal_DWA();
            //ROS_INFO("finish cal DWA\n");

            cal_predict_position();
            //ROS_INFO("finish cal predict pos\n");

            sensor.cal_obs(sensor.latest_scan,10,70,sensor.odom.pose.pose.position.x,sensor.odom.pose.pose.position.y);
            ROS_INFO("candidate size is %d",DWA.CandVel.size());   
            //ROS_INFO("finish cal obs\n");

            cal_Dist();
            //ROS_INFO("finish cal Dist\n");

            double D = cal_vel_sat();
            //ROS_INFO("finish cal val_sat\n");

            int index = cal_J_sharedDWA(D);

            if( sensor.joy_cmd_vel[0] >= 0){
            vel.linear.x=DWA.CandVel[index][0];
            vel.angular.z=DWA.CandVel[index][1];
            }
        }


        pub_cmd.publish(vel);
        ROS_INFO("vel:%f,%f.",vel.linear.x,vel.angular.z);
        ROS_INFO("pub vel.\n");


        clear_vector();
        //ROS_INFO("clear vector");

        rate.sleep();
    }
   
}

int main(int argc, char **argv){
  ros::init(argc, argv, "my_robo_drive");

  // ros::NodeHandle n;
  // ros::Subscriber sub = n.subscribe("/laserscan", 10, chatterCallback);

  my_robo robot;
  //robot.controlloop();
  robot.DWAloop();

  //ros::Rate loop_rate(1);
  //ros::spin();

  return 0;
}
