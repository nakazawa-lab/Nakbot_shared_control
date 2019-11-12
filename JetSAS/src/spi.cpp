/*************************************************************** K-MAIL ***
* File Name : spi.cpp
* Contents : JetSAS spi control program 1.00
* Use Module:
* Master CPU1 : Jetson TK1 NVidia
* Slave  CPU2 : R5f72165ADF RENESAS
* Compiler : gcc
* history : 2015.04.24 ver.1.00.00
* Revised : ver.1.00.00
* original refer
* https://github.com/NeuroRoboticTech/JetduinoDrivers/blob/master/Jetson/ArduinoSPI/main.cpp
***************************** Copyright NAKAZAWA Lab Dept. of SD.keio.jp ***/
#include <stdio.h>
#include <string.h>
//#include <sys/types.h>
//#include <sys/stat.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include <unistd.h> ///for close()

static void dumpstat(const char *name, int fd);

int deviceHandle;
int readBytes;
#define BUFF_SIZE 10

//Uncomment out this line to switch to software cs control
//#define CS_SOFTWARE_CONTROL 1

unsigned char inBuffer[BUFF_SIZE];
unsigned char outBuffer[BUFF_SIZE];

timespec diff(timespec start, timespec end)
{
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

int spi_open(void){// open device on /dev/spidev0.0

	if ((deviceHandle = open("/dev/spidev0.0", O_RDWR)) < 0) {
		printf("Error: Couldn't open device 0! %d\n", deviceHandle);
		return 1;
	}
    printf("spidev0.0=%d \n",deviceHandle);
    dumpstat("spidev0.0", deviceHandle);

}
int spi_close(void){// open device on /dev/spidev0.0
	close(deviceHandle);
}
static void dumpstat(const char *name, int fd)
{
	__u8	mode, lsb, bitsr=64, bitsw;
	__u32	speed=12000000;//25000000

	if (ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0) {
		perror("SPI rd_mode");
		return;
	}
	if (ioctl(fd, SPI_IOC_RD_LSB_FIRST, &lsb) < 0) {
		perror("SPI rd_lsb_fist");
		return;
	}
	if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bitsr) < 0) {
		perror("SPI bits_per_word");
		return;
	}
	if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bitsr) < 0) {
		perror("SPI bits_per_word");
		return;
	}
	if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bitsw) < 0) {
		perror("SPI bits_per_word");
		return;
	}
	if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) > 0) {
		perror("SPI write max_speed_hz");
		return;
    }
	if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
		perror("SPI max_speed_hz");
		return;
	}

	printf("%s: spi mode %d, %d bitsr, %d bitsw, %sper word, %d Hz max\n",
		name, mode, bitsr, bitsw, lsb ? "(lsb first) " : "", speed);
}


void do_msg(int fd)
{
	struct spi_ioc_transfer	xfer[1];
	unsigned char txbuf[32], rxbuf[32], *bp = NULL;
	int			status;
	int len = BUFF_SIZE;

	memset(xfer, 0, sizeof xfer);
	memset(txbuf, 0, sizeof txbuf);
	memset(rxbuf, 0, sizeof rxbuf);

	for(int i=0; i<len; i++)
        txbuf[i] = 65+i; //65+i;

	xfer[0].tx_buf = (unsigned long) txbuf;
	xfer[0].rx_buf = (unsigned long) rxbuf;
	xfer[0].len = len;

	status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSAGE");
		return;
	}

	printf("response(%2d, %2d): ", len, status);
	for (bp = rxbuf; len; len--)
		printf(" %02x", *bp++);
	printf("\n");
}

