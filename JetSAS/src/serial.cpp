/*************************************************************** K-MAIL ***
* File Name : serial.cpp
* Contents : JetSAS RS232C control program 1.00
* Use Module:
* Master CPU1 : Jetson Xavier NVidia
* Slave  CPU2 : R5f72165ADF RENESAS
* Compiler : gcc
* history : 2015.04.12 ver.1.00.00
* Revised : ver.1.00.00
* Revised : 2019.07.18 ver.1.10.00
* avairable buad rate  110, 300, 600, 1200, 2400, 4800, 9600, 14400,
* 19200, 38400, 57600, 115200, 230400, 460800, 921600
***************************** Copyright NAKAZAWA Lab Dept. of SD.keio.jp ***/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h> // needed for memset

#include "JetSAS/jetsas.h"

#include "JetSAS/ros_node.h"       // addded by kitajima

int tty_fd0, tty_fd1 ;

JetSAS::Serial_sh ros_serial;       // added by kitajima
extern void save_serial(const char &RS_cmd, const int (&RS_prm)[4]);

/*************************************************************** UART_init ***/
int UART_init(void)
{
    struct termios tio;
    char tty0[]="/dev/ttyTHS0";
///    char tty0[]="/dev/ttyTHS2";
///    char tty1[]="/dev/ttyTHS1";
//    int tty_fd;
    int count=0;
    fd_set rdset;

    unsigned char c='D', r='R';

    memset(&tio,0,sizeof(tio));
    tio.c_iflag=0;
    tio.c_oflag=0;
    tio.c_cflag=CS8|CREAD|CLOCAL;
    // 8n1, see termios.h for more information
    tio.c_lflag=0;
    tio.c_cc[VMIN]=1;

    tio.c_cc[VTIME]=5;

    tty_fd0=open(tty0, O_RDWR | O_NONBLOCK);
    printf("tty_fd0=%d \n",tty_fd0);
    if(tty_fd0<0) return 0;
    cfsetospeed(&tio,B230400);            // 921600 baud
    cfsetispeed(&tio,B230400);            // 921600 baud
    tcsetattr(tty_fd0,TCSANOW,&tio);
    /***
        tty_fd1=open(tty1, O_RDWR | O_NONBLOCK);
        printf("tty_fd1=%d \n",tty_fd1);
        if(tty_fd1<0) return 0;
        cfsetospeed(&tio,B115200);            /// 921600 baud
        cfsetispeed(&tio,B115200);            /// 921600 baud
        tcsetattr(tty_fd1,TCSANOW,&tio);
    ***/
    return 0;
}/****************************************************************** END ***/

/*************************************************************** jetsas ***/
int jetsas(char cmd,int prm1, int prm2)
{
    static int c=0x45, ret;
///    int dat[]= {'v',',','5','0','1','0',',','5','0','1','0',0x0d};
///    int dat[]= {'m',',','0','0','0','1',',','0','0','0','0',0x0d};
///    char dat[]= {'m',',','0','0','0','1',',','0','0','0','0',0x0d};
///    char dt1[]= {'m',',','0','0','0','1',',','0','0','0','0',0x0d};
///    char dt2[]= {'v',',','5','0','1','0',',','4','9','9','0',0x0d};
    char dt1[20];

//    char dat[]= "m,0001,0000";//{'m',',','0','0','0','1',',','0','0','0','0',0x0d};
//    printf("jetsas cmd=%c prm1=%d prm2=%d\n",cmd,prm1,prm2);
    sprintf(dt1,"%c,%4d,%4d%c",cmd,prm1,prm2,0x0d);
///    printf("dt1=%s\n",dt1) ;

    send(dt1,12);
    usleep(1000);
///    send(dt2,12);
//    printf("%d ret=%d\n",c,ret);
//    if(c>100)c=32;

    return 0;
}/****************************************************************** END ***/

/*************************************************************** jetsas ***/
int send(char dat[], int n)
{
    int i, ret;
    for(i=0; i<n; i++)
    {
///        printf("%d \n",i);
///        ret=write(tty_fd0,&dat[i],1);
        ret=write(tty_fd0,&dat[i],1);
///     printf("ret=%d",ret);
        usleep(5);
    }
    return 0;
}/****************************************************************** END ***/
/*************************************************************** jetsas ***/
int send_tocos(int d)
{
    int i, ret;
    char buf = '0x55';

///        ret=write(tty_fd0,&dat[i],1);
    buf= 0x41+d;
    ret=write(tty_fd1,&buf,1);
    usleep(5);
    if (d==25)
    {
        buf= 0x0d;
        ret=write(tty_fd1,&buf,1);
    }
    usleep(5);
    return 0;
}/****************************************************************** END ***/
/***************************************************************************/
/*    Check dat in 0x30 ~ 0x39                                             */
/***************************************************************************/
int deci(int dat){

    dat=dat-0x30;
	if(dat<0)dat=0;
	if(dat>9)dat=0;

	return dat;

}/************************************************************ End of BT_init */
/*************************************************************** th_receive ***/
void* th_receive(void* pParam)
{
    unsigned char r='R';
    char RS_cmd,dat[100];
    int a=1,ct=0;
    int RS_prm[4],b;
    static int num=0;

    while(1)
    {
        while(read(tty_fd0,&r,1)>0)
        {
            //printf("received from NakBot %x %c\n",r,r);
            gpio_led(LED_RED,LED_ON);

            dat[num]=r;
            num++;
            if(r=='\r')
            {
                dat[num]=0;
                //printf("received from NakBot %s \n",dat);

                RS_cmd=dat[0];

                b=2;
                RS_prm[0]=deci(dat[b])*1000000+deci(dat[b+1])*100000
                  +deci(dat[b+2])*10000+deci(dat[b+3])*1000
                  +deci(dat[b+4])*100+deci(dat[b+5])*10+deci(dat[b+6]);

                b=10;
                RS_prm[1]=deci(dat[b])*1000000+deci(dat[b+1])*100000
                  +deci(dat[b+2])*10000+deci(dat[b+3])*1000
                  +deci(dat[b+4])*100+deci(dat[b+5])*10+deci(dat[b+6]);

                b=18;
                RS_prm[2]=deci(dat[b])*1000000+deci(dat[b+1])*100000
                  +deci(dat[b+2])*10000+deci(dat[b+3])*1000
                  +deci(dat[b+4])*100+deci(dat[b+5])*10+deci(dat[b+6]);

                b=26;
                RS_prm[3]=deci(dat[b])*1000000+deci(dat[b+1])*100000
                  +deci(dat[b+2])*10000+deci(dat[b+3])*1000
                  +deci(dat[b+4])*100+deci(dat[b+5])*10+deci(dat[b+6]);

                 printf("Decoded data %c %8d %8d %8d %8d\n",
                        RS_cmd,RS_prm[0],RS_prm[1],RS_prm[2],RS_prm[3]);

                save_serial(RS_cmd, RS_prm);        /// added by kitajima 
                num=0;
            }
            gpio_led(LED_RED,LED_OFF);

        }

        if(ct>500)
        {
            gpio_led(LED_BLUE,a^=1);
            ct=0;
        }
        ct++;
        usleep(1000);
//       printf("pass ");
    }
}/****************************************************************** END ***/


/****
c=32;
        while ((count++)<100)
        {
		c++;
		write(tty_fd,&c,1);
//		printf("data=%c /n\n",c);
		usleep(10);
//		pauseNanoSec(10000000);


        }

	while(read(tty_fd,&r,1)>0) printf("       buffered data = %c \n",r);

        close(tty_fd);
}
*****/
