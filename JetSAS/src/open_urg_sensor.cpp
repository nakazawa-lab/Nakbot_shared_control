/*!
  \brief URG ‚Æ‚ÌÚ‘±
  \author Satofumi KAMIMURA

  $Id$
*/

//#include "urg_sensor.h"
//#include "urg_utils.h"
#include <urg_c/urg_sensor.h>
#include <urg_c/urg_utils.h>
#include <urg_c/urg_detect_os.h>
#include <string.h>
#include <stdio.h>


int open_urg_sensor(urg_t *urg, int argc, char *argv[])
{
    const char *device = "/dev/ttyACM0";

    urg_connection_type_t connection_type = URG_SERIAL;
    long baudrate_or_port = 115200;
///    const char *ip_address = "192.168.0.10";
    //const char *ip_address = "localhost";
///    int i;

    // Ú‘±ƒ^ƒCƒv‚ÌØ‘Ö‚¦
/*****
    for (i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-e")) {
            connection_type = URG_ETHERNET;
            baudrate_or_port = 10940;
            device = ip_address;
        }
        if (!strcmp(argv[i], "-e") || !strcmp(argv[i], "-s")) {
            if (argc > i + 1) {
                ++i;
                device = argv[i];
            }
        }
    }
*****/
    // Ú‘±
    if (urg_open(urg, connection_type, device, baudrate_or_port) < 0) {
        printf("urg_open: %s, %ld: %s\n",
            device, baudrate_or_port, urg_error(urg));
        return -1;
    }

    return 0;
}
