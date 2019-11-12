/*************************************************************** K-MAIL ***
* File Name : timer.cpp
* Contents : JetSAS time get program 1.00
* Use Module:
* Master CPU1 : Jetson TK1 NVidia
* Slave  CPU2 : R5f72165ADF RENESAS
* Compiler : gcc
* history : 2015.05.10 ver.1.00.00
* Revised : ver.1.00.00
***************************** Copyright NAKAZAWA Lab Dept. of SD.keio.jp ***/
#include <stdio.h>
#include <time.h>
#include "JetSAS/timer.h"
timespec diff(timespec start, timespec end);

/******
class JET_TIMER
{
public:
    JET_TIMER();
    ~JET_TIMER();
    int start(void);
    int stop(void);
    int reset(void);
    int get(void);
    int t_usec;
    int t_sec;
};
*****/
JET_TIMER::JET_TIMER()
{
    t_usec=0;
    t_sec=0;
    printf("JET_TIMER Constructor \n");
}
JET_TIMER::~JET_TIMER()
{
    printf("JET_TIMER Destructor \n");

}
int JET_TIMER::reset()
{
    timespec startTime, endTime, endTime2;

    clock_gettime(CLOCK_REALTIME, &startTime);
    printf("%d %d \n",startTime.tv_sec,startTime.tv_nsec);
    t.tv_sec=startTime.tv_sec;
    t.tv_nsec=startTime.tv_nsec;

}
int JET_TIMER::get_sec()
{
    timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    return (int)diff(t,now).tv_sec;
//    return now.tv_sec-t.tv_sec;
}
int JET_TIMER::get_nsec()
{
    timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
///return 0;
    return (int)diff(t,now).tv_nsec;
/*    timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    return now.tv_nsec-t.tv_nsec;
*/
}

