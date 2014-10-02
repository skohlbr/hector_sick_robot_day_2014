#include "ros/ros.h"
#include "sickrobot_2014_msgs/SetLedState.h"

#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

int fd_arduino;

void setup_serial(){
    fd_arduino = open( "/dev/ttyACM0", O_RDWR| O_NONBLOCK | O_NDELAY );
    if(fd_arduino <= 0){
        ROS_ERROR("Port not found");
    }
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    /* Set Baud Rate */
    cfsetospeed (&tty, B9600);
    tty.c_cflag     &=  ~PARENB;        // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;
    tty.c_cflag     &=  ~CRTSCTS;       // no flow control
    tty.c_lflag     =   0;          // no signaling chars, no echo, no canonical processing
    tty.c_oflag     =   0;                  // no remapping, no delays

    /* Flush Port, then applies attributes */
    tcflush( fd_arduino, TCIFLUSH );

    if ( tcsetattr ( fd_arduino, TCSANOW, &tty ) != 0)
    {
        ROS_ERROR("Error %d from tcsetattr",errno);
    }
}

bool set_state(sickrobot_2014_msgs::SetLedStateRequest  &req,sickrobot_2014_msgs::SetLedStateResponse  &res){

    unsigned char cmd[] = {'a'};
    if(req.state){
        cmd[0] = 'a';
        ROS_INFO("LED: on");
    }else{
        cmd[0] = 'b';
        ROS_INFO("LED: off");
    }
    write( fd_arduino, cmd , sizeof(cmd) );
    tcflush( fd_arduino, TCOFLUSH );

    return true;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "blinky_node");

    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("set_led_state", set_state);

    setup_serial();

    ros::spin();

    close(fd_arduino);

    return 0;
}
