/*************************************************************** K-MAIL ***
* File Name : aquestalk.cpp
* Contents : JetSAS Aques Talk control program 1.00
* Use Module:
* Master CPU1 : Jetson Xavier NVidia
* Slave  CPU2 : R5f72165ADF RENESAS
* Compiler : gcc
* history : 2016.11.07 ver.1.00.00
* Revised : ver.1.00.00
* Revised : 2019.07.18 ver.1.10.00
***************************** Copyright NAKAZAWA Lab Dept. of SD.keio.jp ***/
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include "JetSAS/jetsas.h"

int fda;
int er_aque=0;
extern int fd_i2c; ///declere at lcd.cpp

/*************************************************************** LCD_init ***/
int AQUE_init(void)
{
    int i,ret=0;
    int addr = 0x2e;

 ///   fda = open("/dev/i2c-1", O_RDWR);
 ///   if(fda < 0) printf("aques talk failed to open\n");
 ///   printf("aque-1=%d \n",fda);
   printf("aque=%d \n",fd_i2c);

    if(ioctl(fd_i2c, I2C_SLAVE, addr) < 0)
        printf("failed to acquire bus for aques talk\n");

    return ret;
}/****************************************************************** END ***/

/******************************************************* LCD_DrawString2 ***/
int AQUES_Talk(char *buf)
{
    int ret, len, i;
    int addr = 0x2e;
    char bu[128];

    if(ioctl(fd_i2c, I2C_SLAVE, addr) < 0) {
        printf("failed to acquire bus for aques talk\n");
        return -1;
    }
    while(1)
    {
        ret=read(fd_i2c,bu, 1);
///        printf(" %2x",bu[0]);
///        if((bu[0]=='*')||(bu[0]==0xff)) continue;
        if(bu[0]==0x3e)break;
        usleep(1000);

    }
    i=0;
    while(1)   /// copy from buf[] to bu[]
    {
        if(buf[i]==0) break;
        bu[i]=buf[i];
///        printf("i=%d bu=%c \n",i,bu[i]);
        i++;
    }
    bu[i]=0x0d;
    len=i+1;

    if((ret=write(fd_i2c,bu,len)) < 0) er_aque++;

///    printf("er_aque=%d len=%d\n",er_aque,len);
    usleep(10000);

    return ret;
}/****************************************************************** END ***/

