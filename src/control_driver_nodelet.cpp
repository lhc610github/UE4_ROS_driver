#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <iostream>
#include <arpa/inet.h>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <string>
#include "ue4_ros_drivers/config.h"

#include <nodelet/nodelet.h>
#include <ros/ros.h>

class ControlDriver : public nodelet::Nodelet {
    public:
        void onInit();
    private:
        void cmdCB(const mavros_msgs::AttitudeTarget::ConstPtr &msg);
        ros::Subscriber CmdSubscriber_;
        std::string IP_ADRR_;
        std::string topic_name_;
        int IP_PORT_;

        int sockfd, opt, valread;
        struct sockaddr_in servaddr;

};

void ControlDriver::onInit() {
    ros::NodeHandle priv_nh(getPrivateNodeHandle());
    priv_nh.param<std::string>("address", IP_ADRR_, "192.168.10.15");
    priv_nh.param<int>("port", IP_PORT_, 6766);
    priv_nh.param<std::string>("topic",  topic_name_, "controller");
    CmdSubscriber_ = priv_nh.subscribe(topic_name_, 10, &ControlDriver::cmdCB, this);

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
        std::cout << "ERROR open socket" << std::endl;

    opt = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        std::cout << "Socket set wrong" << std::endl;
    }

    servaddr.sin_family    = AF_INET;
    servaddr.sin_port = htons(IP_PORT_); 
    servaddr.sin_addr.s_addr = inet_addr(IP_ADRR_.c_str());
    if ( bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)  {
        std::cout << "Bind error" << std::endl;
    }
    if ( connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)  {
        std::cout << "connect error" << std::endl;
    }
}

void ControlDriver::cmdCB(const mavros_msgs::AttitudeTarget::ConstPtr &msg) {
    std::vector<unsigned char> data = control_cmd_encode(*msg);
    send(sockfd, data.data(), data.size(), MSG_CONFIRM);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ControlDriver, nodelet::Nodelet);