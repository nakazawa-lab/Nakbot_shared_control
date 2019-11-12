/*************************************************************** K-MAIL ***
* File Name : gpio.cpp
* Contents : JetSAS gpio program 1.00
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

#include <iostream>
#include <string>
#include <unistd.h>

#include "JetSAS/jetsas.h"
/*
#include "SimpleGPIO.h"

#define LED_RED 0
#define LED_WHITE 1
#define LED_YELLOW 2
#define LED_GREEN 3
#define LED_BLUE 4
*/
///int fd388, fd298, fd480, fd486; ///, fd164, fd165, fd166;
int fd422, fd351, fd424, fd393, fd256; ///, fd164, fd165, fd166;

/************************************************************ gpio_open ***/
int gpio_open(void)
{
    gpio_export(422);//SW1 LED RED
    gpio_export(351);//LED WHITE
    gpio_export(424);//LED YELLOW
    gpio_export(393);//LED GREEN
    gpio_export(256);//LED BLUE
///    gpio_export(165);//SW 1
///   gpio_export(166);//SW 2
    gpio_set_dir(422, OUTPUT_PIN);
    gpio_set_dir(351, OUTPUT_PIN);
    gpio_set_dir(424, OUTPUT_PIN);
    gpio_set_dir(393, INPUT_PIN);
    gpio_set_dir(256, INPUT_PIN);
///    gpio_set_dir(164, OUTPUT_PIN);
///    gpio_set_dir(165, INPUT_PIN);
///    gpio_set_dir(166, INPUT_PIN);

    int er = 0;
    char buf[64];

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", 422);
    if ((fd422 = open(buf, O_WRONLY)) < 0)er--;

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", 351);
    if ((fd351 = open(buf, O_WRONLY)) < 0)er--;

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", 424);
    if ((fd424 = open(buf, O_WRONLY)) < 0)er--;

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", 393);
    if ((fd393 = open(buf, O_RDONLY)) < 0)er--;

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", 256);
    if ((fd256 = open(buf, O_RDONLY)) < 0)er--;

///    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", 164);
///    if ((fd164 = open(buf, O_WRONLY)) < 0)er--;

//	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", 165);
//	if ((fd165 = open(buf, O_RDONLY)) < 0)er--;

//	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", 166);
//	if ((fd166 = open(buf, O_RDONLY)) < 0)er--;

    if (er<0) printf("gpio open error %d \n",er);

    return er;
}/****************************************************************** END ***/

/************************************************************ gpio_led ***/
void gpio_led(int led, int on)
{
    int fd;
    switch (led)
    {
    case LED_RED:
        fd = fd422;
        break;
    case LED_GREEN:
        fd = fd351;
        break;
    case LED_BLUE:
        fd = fd424;
        break;
///     case LED_WHITE:
///       fd = fd486;
        break;
 ///   case LED_BLUE:
 ///       fd = fd164;
 ///       break;
    }
    if (on == 1)write(fd, "1", 2);
    else write(fd, "0", 2);
}/****************************************************************** END ***/

/*************************************************************** gpio_sw ***/
int gpio_sw(int sw)
{
    unsigned int value = 0;
    char c;

    switch (sw)
    {
    case SW1:
        gpio_get_value(393, &value);
        break;
    case SW2:
        gpio_get_value(256, &value);
        break;
//    defualt:
    }

    return value;
}
/************************************************************ gpio_init ***/
void gpio_close()
{

    close(fd422);
    close(fd351);
    close(fd424);
    close(fd393);
    close(fd256);
///    close(fd164);
///    close(fd165);
///    close(fd166);

}/****************************************************************** END ***/




