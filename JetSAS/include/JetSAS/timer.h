
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
timespec diff(timespec start, timespec end);

class JET_TIMER
{
public:
    JET_TIMER();
    ~JET_TIMER();
    int start(void);
    int stop(void);
    int reset(void);
    int get_sec(void);
    int get_nsec(void);
    int t_usec;
    int t_sec;
    timespec t;
};
/****
JET_TIMER::JET_TIMER() {
    t_usec=0;
    t_sec=0;
    printf("JET_TIMER Constructor \n");
}
JET_TIMER::~JET_TIMER(){
    printf("JET_TIMER Destructor \n");

}
****/
