/*************************************************************** K-MAIL ***
* File Name : lcd.cpp
* Contents : JetSAS lcd control program 1.00
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
#include <stdarg.h>
int fd_i2c;
int er_i2c=0;

int LCD_DrawString(int x, int y, char *buf);
int LCD_DrawString2(int x, int y, char *buf);

/*************************************************************** LCD_init ***/
int LCD_init(void)
{
    int i,ret;
    int addr = 0x3e;
    unsigned char buf[] = {0x0,0x38,0x39,0x14,0x72,0x56,0x6c};

    fd_i2c = open("/dev/i2c-8", O_RDWR);
    if(fd_i2c < 0) printf("i2c failed to open\n");
    printf("i2c-8=%d for LCD\n",fd_i2c);

    if(ioctl(fd_i2c, I2C_SLAVE, addr) < 0)printf("failed to acquire bus\n");

    if((ret=write(fd_i2c,buf,7)) < 0)printf("i2c write failed at LCD_init1\n");
    usleep(100);      // sleep

    buf[2] = 0x0d;
    buf[3] = 0x01;
    if((ret=write(fd_i2c,buf,4)) < 0)printf("i2c write failed at LCD_init2\n");
    usleep(10000);      // sleep


    return ret;
}/****************************************************************** END ***/

/******************************************************** LCD_DrawString ***/
int LCD_printf(int x, int y, const char *fmt, ...)
{
    int ret;
    va_list args;
    char buff[20];
    int addr = 0x3e;

    va_start(args,fmt);
    vsprintf(buff,fmt,args);

if(ioctl(fd_i2c, I2C_SLAVE, addr) < 0)printf("failed to acquire bus\n");

    ret = LCD_DrawString2(x, y, buff);
    if(ret<0){ret = LCD_DrawString2(x, y, buff);ret=0;}

    return ret;
}/****************************************************************** END ***/


/******************************************************** LCD_DrawString ***/
int LCD_DrawString(int x, int y, char *buf)
{
    int ret;
    ret = LCD_DrawString2(x, y, buf);
    if(ret<0)ret = LCD_DrawString2(x, y, buf);
    return ret;
}/****************************************************************** END ***/

/******************************************************* LCD_DrawString2 ***/
int LCD_DrawString2(int x, int y, char *buf)
{
    int ret, len, i;
    char bu[21];

    bu[0]=0x0;
    if (x>12)x=12;
    if (y==0)bu[1]=0x80+x;
    else bu[1]=0xc0+x; // set position
    if((ret=write(fd_i2c,bu,2)) < 0)
    {
        printf("i2c(LCD) write 1 failed ret=%d\n",ret);
    }
    usleep(1000);

    bu[0]='@';
    i=0;
    while(1)   // copy from buf[] to bu[]
    {
        if(buf[i]==0) break;
        bu[i+1]=buf[i];
        i++;
    }
    len=i+1;

    if((ret=write(fd_i2c,bu,len)) < 0)  //write to LCD
    {
        er_i2c++;
    }
    usleep(1000);

    return ret;
}/****************************************************************** END ***/

