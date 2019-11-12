/*************************************************************** K-MAIL ***
* File Name : i2c1_sensor.cpp
* Contents : JetSAS i2c-1 sensor control program 1.00
* Use Module:
* Master CPU1 : Jetson Xavier NVidia
* Slave  CPU2 : R5f72165ADF RENESAS
* Compiler : gcc
* history : 2015.04.12 ver.1.00.00
* Revised : ver.1.00.00
* Revised : 2019.07.18 ver.1.10.00
***************************** Copyright NAKAZAWA Lab Dept. of SD.keio.jp ***/
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include "JetSAS/jetsas.h"

extern int fd_i2c; ///declere at lcd.cpp
/*************************************************************** I2C1_init ***/
int I2C1_init(void)
{
    int i,ret;
//    int addr = 0x69;
    unsigned char buf[20], buf2[20];

///    fd_i2c = open("/dev/i2c-1", O_RDWR);
///    if(fd_i2c < 0) printf("i2c failed to open\n");

    printf("i2c-0=%d for sensors \n",fd_i2c);

///MPU6050 wake up
    if(ioctl(fd_i2c, I2C_SLAVE, 0x68) < 0)printf("failed to acquire bus\n");
    buf[0] = 0x6b;
    buf[1] = 0x0;
    if((ret=write(fd_i2c,buf,2)) < 0)printf("i2c write failed at init1\n");
    usleep(1000);

///SHT31 <== HDC1000 set up
    if(ioctl(fd_i2c, I2C_SLAVE, 0x45) < 0)printf("failed to acquire bus\n");
    buf[0] = 0x24;
    buf[1] = 0x0b;
    if((ret=write(fd_i2c,buf,2)) < 0)printf("i2c write failed at init2\n");
    usleep(1000);

///LPS25H wake up
    if(ioctl(fd_i2c, I2C_SLAVE, 0x5c) < 0)printf("failed to acquire bus\n");
    buf[0] = 0x20;
    buf[1] = 0x90;

    if((ret=write(fd_i2c,buf,2)) < 0)printf("i2c write failed at init3\n");

///S11059 wake up

    return 0;
}/****************************************************************** END ***/


/************************************************************* get_data1 ***/
int get_data1(int adr)
{
    int gx;
    unsigned char buf[10];

    buf[0] = adr;
    if(write(fd_i2c,buf,1) != 1) printf("i2c write failed\n");
    usleep(1000);
    if(read(fd_i2c,buf, 2) != 2) printf("i2c read failed\n");
    usleep(1000);

    if((buf[0]&0x80)==0x80)  gx=-((65535-buf[0]*0x100+buf[1])+1);
    else  gx=buf[0]*0x100+buf[1];

    return  gx;
}/****************************************************************** END ***/

/*************************************************************** sensor ***/
int sensor(int TYPE,float g[])
{
    switch(TYPE)
    {
    case MPU6050:
        mpu6050(g);
        break;
    case S11059:
        s11059(g);
        break;
    case LPS25H:
        lps25h(g);
        break;
    case SHT31:
        sht31(g);
        break;
    default:
        break;
    }

    return 0;
}/****************************************************************** END ***/


/*************************************************************** mpu6050 ***/
int mpu6050(float g[])
{
    int i,ret;
    int gx;
    int addr = 0x68;
    unsigned char buf[20], buf2[20];
///static int cnt=0;
///printf("MPU6050 %d \n",cnt++);
    if(ioctl(fd_i2c, I2C_SLAVE, addr) < 0)printf("failed to acquire bus\n");

    g[0] = get_data1(0x3b)/16384.0;
    g[1] = get_data1(0x3d)/16384.0;
    g[2] = get_data1(0x3f)/16384.0;
    g[3] = get_data1(0x43)/1000.0;
    g[4] = get_data1(0x45)/1000.0;
    g[5] = get_data1(0x47)/1000.0;

    return ret;
}/****************************************************************** END ***/

/*************************************************************** sht31 ***/
int sht31(float g[])
{
    int i,ret;
    int gx;
    int addr = 0x45;///0x40;
    unsigned char buf[20], buf2[20];

    if(ioctl(fd_i2c, I2C_SLAVE, addr) < 0)printf("failed to acquire bus\n");

/****/
    buf[0] = 0x00; ///get temperature & humidity

    buf[0] = 0x24;
    buf[1] = 0x0b;
    if((ret=write(fd_i2c,buf,2)) != 2)
        printf("i2c write failed *1 ret=%d \n",ret);
    usleep(6000);
/*******/

    if((ret=read(fd_i2c,buf, 7)) != 7)printf("i2c read failed *1 ret=%d \n",ret);
    g[0] = (buf[0]*0x100+buf[1])*175.0/65535.0-45;
///    printf("%x %x %x %f ",buf[0],buf[1],buf[2],g[0]);

    g[1] = (buf[3]*0x100+buf[4])*100.0/65535.0;
///    printf("%x %x %x %f\n",buf[3],buf[4],buf[5],g[1]);

    return ret;
}/****************************************************************** END ***/

/*************************************************************** lps25h ***/
int lps25h(float g[])
{
    int i,ret;
    int gx;
    int addr = 0x5c;
    unsigned char buf[20], buf2[20];

    if(ioctl(fd_i2c, I2C_SLAVE, addr) < 0)printf("failed to acquire bus\n");

    buf[0] = 0x28; ///get temperature
    if((ret=write(fd_i2c,buf,1)) != 1)printf("i2c write failed ret=%d \n",ret);
    if((ret=read(fd_i2c,buf, 1)) != 1)printf("i2c read failed 1 ret=%d \n",ret);
    buf2[0]=buf[0];

    buf[0] = 0x29; ///get temperature
    if((ret=write(fd_i2c,buf,1)) != 1)printf("i2c write failed ret=%d \n",ret);
    if((ret=read(fd_i2c,buf, 1)) != 1)printf("i2c read failed 1 ret=%d \n",ret);
    buf2[1]=buf[0];

    buf[0] = 0x2a; ///get temperature
    if((ret=write(fd_i2c,buf,1)) != 1)printf("i2c write failed ret=%d \n",ret);
    if((ret=read(fd_i2c,buf, 1)) != 1)printf("i2c read failed 1 ret=%d \n",ret);
    buf2[2]=buf[0];

    g[0] = (buf2[2]*0x10000+buf2[1]*0x100+buf2[0])/4096.0;
    printf("%x %x %x %f \n",buf2[2],buf2[1],buf2[0],g[0]);

    return ret;
}/****************************************************************** END ***/

/*************************************************************** s11059 ***/
int s11059(float g[])
{
    int i,ret;
    int wt=1000;
    unsigned char buf[20];

///S11059 wake up
    if(ioctl(fd_i2c, I2C_SLAVE, 0x2a) < 0)printf("failed to acquire bus\n");
    buf[0] = 0x0;
    buf[1] = 0x8a;
    if((ret=write(fd_i2c,buf,2)) < 0)
    {
        usleep(wt);
        if((ret=write(fd_i2c,buf,2)) < 0)printf("i2c write failed at init54\n");
    }
    usleep(wt);
//      if(ioctl(fd_i2c, I2C_SLAVE, 0x2a) < 0)printf("failed to acquire bus\n");
    buf[0] = 0x0;
    buf[1] = 0x0a;
    if((ret=write(fd_i2c,buf,2)) < 0)
    {
        usleep(wt);
        if((ret=write(fd_i2c,buf,2)) < 0)
            if((ret=write(fd_i2c,buf,2)) < 0)printf("i2c write failed at init55\n");
    }

    usleep(90000);///waiting for integral

    buf[0] = 0x03; ///get temperature & humidity
    if((ret=write(fd_i2c,buf,1)) != 1)
    {
        usleep(wt);
        if((ret=write(fd_i2c,buf,1)) != 1)printf("i2c write failed 56 ret=%d \n",ret);
    }
    if((ret=read(fd_i2c,buf, 8)) != 8)
    {
        usleep(wt);
        if((ret=read(fd_i2c,buf, 8)) != 8)printf("i2c read failed 57 ret=%d \n",ret);
    }
    printf("%x %x %x %x %x %x %x %x \n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
    g[0]=buf[0]*100.0+buf[1];
    g[1]=buf[2]*100.0+buf[3];
    g[2]=buf[4]*100.0+buf[5];
    g[3]=buf[6]*100.0+buf[7];
    return ret;
}/****************************************************************** END ***/
