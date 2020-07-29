#include "auto_driving/System.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "NakBot_auto_driving");
    JetSAS::System system;

    system.mainloop();

    return 0;
}