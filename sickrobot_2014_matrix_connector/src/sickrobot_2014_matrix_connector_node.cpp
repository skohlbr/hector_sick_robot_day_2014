#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <ros/console.h>

char matrix_trigger[] = "R\n";

static int read_cnt;
static char *read_ptr;
static char read_buf[100];

static ssize_t
my_read(int fd, char *ptr)
{
    if (read_cnt <= 0) {
again:
        if ( (read_cnt = read(fd, read_buf, sizeof(read_buf))) < 0) {
            if (errno == EINTR)
                goto again;
            return (-1);
        } else if (read_cnt == 0)
            return (0);
        read_ptr = read_buf;
    }

    read_cnt--;
    *ptr = *read_ptr++;
    return (1);
}

ssize_t
readline(int fd, void *vptr, size_t maxlen)
{
    ssize_t n, rc;
    char    c, *ptr;
    ptr = (char *) vptr;
    for (n = 1; n < maxlen; n++) {
        if ( (rc = my_read(fd, &c)) == 1) {
            *ptr++ = c;
            if (c  == '\n')
                break;          /* newline is stored, like fgets() */
        } else if (rc == 0) {
            *ptr = 0;
            return (n - 1);     /* EOF, n - 1 bytes were read */
        } else
            return (-1);        /* error, errno set by read() */
    }

    *ptr  = 0;                  /* null terminate like fgets() */
    return (n);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_node_name");
    ros::NodeHandle nh;

    std::string ip_address;
    int portno;
    int rate;

    nh.param<std::string>("ip_address", ip_address, "192.168.0.1");

    nh.param<int>("port", portno, 5801);

    nh.param<int>("rate", rate, 10);

    int sockfd;
    struct sockaddr_in serv_addr;

    portno = 5801;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0) {
        ROS_ERROR("cannot open socket!");
        return -1;
    }
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(ip_address.c_str());

    serv_addr.sin_port = htons(portno);

    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) {
        ROS_ERROR("Cannot connect to matrix: ip=%s, port=%d", ip_address.c_str(), portno);
    }
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("holzklotz_number", 5);
    std_msgs::Int32 msg;


    ros::Rate r(rate);

    char buffer[256];

    while(ros::ok()){

        ROS_INFO("reading");
        write(sockfd, matrix_trigger, strlen(matrix_trigger));
        bzero(buffer,256);
        int n = readline(sockfd, buffer, sizeof(buffer));
        int station = -1;
        if(n != 4){
            ROS_ERROR("Invalid MSG from scanner recieved, Length should be 4 but was %d", n);
        }else{
            ROS_DEBUG("MSG from Scanner recieved");
            char number = buffer[1];

            if(number == 'X'){
                ROS_INFO("NOREAD");
            }else{
                station = number - '0';
                ROS_INFO("READ: %c -> %d", number, station);
            }
            msg.data = station;
            pub.publish(msg);

        }

        r.sleep();
    }

    close(sockfd);
    return 0;
}
