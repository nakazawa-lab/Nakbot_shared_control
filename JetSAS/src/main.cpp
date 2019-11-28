/*************************************************************** K-MAIL ***
* File Name : main.cpp
* Contents : JetSAS main program 1.00
* Use Module:
* Master CPU1 : Jetson Xavier NVidia
* Slave  CPU2 : R5f72165ADF RENESAS
* Compiler : gcc
* history : 2018.10.08 ver.1.00.00
* Revised : 2019.07.18 ver.1.10.00
***************************** Copyright NAKAZAWA Lab Dept. of SD.keio.jp ***/


#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include <pthread.h>///

#include <iostream>
#include <string>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <urg_c/urg_sensor.h>
#include <urg_c/urg_utils.h>

#include "JetSAS/SimpleGPIO.h"
//#include <GL/glut.h>
#include "JetSAS/jetsas.h"
#include "JetSAS/open_urg_sensor.h"

/// ros header file
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "JetSAS/ros_node.h"
#include <random>

using namespace std;
void* thread1(void* pParam); ///
pthread_mutex_t mutex; ///
int count1=0;///
urg_t urg;  ///for URG sensor
long *urg_data;
const double JetSAS::robot_width = 0.28;    // 実測値
///extern int open_urg_sensor(urg_t *urg, int argc, char *argv[]);

/*************************usage serial.cpp jetsas(int,int,int)*************************
 * jetsas('m',0001,0000);  // コマンド(m,0001,0000) でmortor_on = 1
 * jetsas('m',0000,0000);  // コマンド(m,0001,0000) でmortor_on = 0
 * jetsas('v',5010,5010);  // コマンドvで速度指令 vel1=5010-5000,vel2=5010-5000 値の範囲は4000から5999
 * jetsas('L',5100,5100);  // コマンドLで速度制限地設定指令 v_max=10 2つの引数が一致しなければ変更しない
 * jetsas('e',0001,0001);  // コマンドeで エンコーダの値を100で割って送信, 時間, 左, 右, 時間の順←この1年で仕様が変わったかも?
 * received from nakbot a b c d  → 順番に右の速度指令値, 左の速度指令値, 右の実行開始からの進行距離, 左の実行開始からの進行距離　前進方向がエンコーダの値が大きくなる向き
 * jetsas('r',0001,0001);  // コマンドrでラジコンからの値を取得して表示する　左から回転量(時計回りが正), 並進量(後退方向が正), チャンネルのスイッチ(小さいと入力を受け付けて車輪を動かす), アンテナ右のスイッチの回転量
/*************************usage serial.cpp jetsas(int,int,int))*************************

/******************************************************** JetsonTK1_init ***/
int JetsonXavier_init()
{
    char *dummy[1];
    LCD_init();
    gpio_open();
///    spi_open();
    UART_init();
    I2C1_init();
///    AQUE_init(); ///need not init
    if (open_urg_sensor(&urg, 1, dummy) < 0) return 1;
    urg_data = (long *)malloc(urg_max_data_size(&urg) * sizeof(urg_data[0]));

    return 0;
}/****************************************************************** END ***/

/****************************************************************** main ***/
int main(int argc, char *argv[])
{
    int fd,i,ret;
    int addr = 0x3e;
    unsigned char buf[20];
    char buf2[30] = "JetSAS on NakBot";
    JET_TIMER jt;
    int t_s, t_ns;
    float g[20], y, z;

    long max_distance;///for URG sensor
    long min_distance;
    long time_stamp;
    int n;

    /// ros
    ros::init(argc, argv, "JetSAS_node");
    JetSAS_Node node;
    /// end ros

    pthread_t tid1, tid2, tid3;
    pthread_mutex_init(&mutex, NULL);


    printf("Start JetSAS on NakBot\n");

    JetsonXavier_init();

    LCD_printf(0, 0,"JetSAS-X on NakBot");
    printf("Print LCD\n");

    jt.reset();
    pthread_create(&tid1, NULL, th_receive, NULL);

    printf("jt time0 %d[sec] %d[nsec]\n",jt.get_sec(),jt.get_nsec());

    jetsas('m',0001,0000);  // コマンド(m,0001,0000) でmortor_on = 1
    usleep(1000000);         // on for 1s

//    jetsas('v',5100,5100);      // コマンドvで速度指令 vel1=5010-5000,vel2=5010-5000 値の範囲は4000から5999

    // Wait for the push button to be pressed
    cout << "Please press the button!" << endl;


    unsigned int value = LOW;
    int j=0;
    int vel_i =0;
    
    jetsas('L',5200,5200);
    //jetsas('v',4800,4900);
    jetsas('e',0001,0001);

    do
    {
///        jetsas('v',node.temp,node.temp);

           //jetsas('v',5200,5200);
///        jetsas('r',0001,0001);


///        jetsas('v',5070,4970);
///        jetsas('r',0001,0001);



        /************/

        /// ros addedby kitajima
        //node.lrf.make_scan_msgs(urg_data,n);
        //node.lrf.pub_lrf();
        node.controlloop(jt);
        //std::cout << "right" << ros_serial.encoder.r_sum - ros_serial.encoder.r_init  << std::endl;
        //std::cout << "left" << ros_serial.encoder.l_sum - ros_serial.encoder.l_init  << std::endl;
        ///end ros added by kitajima

        value=gpio_sw(SW1);
    }
    while (value==HIGH);

    cout << endl <<  "Button was just pressed!" << endl;
    cout << "Finished Testing the GPIO Pins" << endl;
    gpio_close();

    printf("time2 %d[sec] %d[nsec]\n",jt.get_sec(),jt.get_nsec());
    pthread_mutex_destroy(&mutex);


    return 0;
}/****************************************************************** END ***/

