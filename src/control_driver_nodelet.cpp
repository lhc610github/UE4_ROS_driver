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
#include "ue4_ros_drivers/udp_handler.h"

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
        std::shared_ptr<UDPClientHandler> udp_handler_ptr;
        int IP_PORT_;
};

void ControlDriver::onInit() {
    ros::NodeHandle priv_nh(getPrivateNodeHandle());
    priv_nh.param<std::string>("address", IP_ADRR_, "192.168.10.15");
    priv_nh.param<int>("port", IP_PORT_, 6766);
    priv_nh.param<std::string>("topic",  topic_name_, "controller");
    udp_handler_ptr.reset(new UDPClientHandler(IP_PORT_, false, IP_ADRR_));
    CmdSubscriber_ = priv_nh.subscribe(topic_name_, 10, &ControlDriver::cmdCB, this);
}

void ControlDriver::cmdCB(const mavros_msgs::AttitudeTarget::ConstPtr &msg) {
    std::vector<unsigned char> data = control_cmd_encode(*msg);
    udp_handler_ptr->udp_sendto(data.data(), data.size());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ControlDriver, nodelet::Nodelet);