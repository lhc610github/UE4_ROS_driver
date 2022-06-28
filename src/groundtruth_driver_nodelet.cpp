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

class GroundTruthDriver : public nodelet::Nodelet {
    public:
        void onInit();
    private:
        // void threadCB(const ros::TimerEvent&);
        void threadCB();
        ros::Publisher OdomPublisher_;
        // ros::Timer RunOnceTimer_;
        std::thread recv_thread_;
        std::string IP_ADRR_;
        std::string topic_name_;
        std::shared_ptr<UDPServerHandler> udp_handler_ptr;
        int IP_PORT_;
};

void GroundTruthDriver::onInit() {
    ros::NodeHandle priv_nh(getPrivateNodeHandle());
    priv_nh.param<std::string>("address", IP_ADRR_, "192.168.10.15");
    priv_nh.param<int>("port", IP_PORT_, 6766);
    priv_nh.param<std::string>("topic",  topic_name_, "groundtruth_odom");
    OdomPublisher_ = priv_nh.advertise<nav_msgs::Odometry>(topic_name_, 10);
    recv_thread_ = std::thread(std::bind(&GroundTruthDriver::threadCB, this));
}

void GroundTruthDriver::threadCB() {
    udp_handler_ptr.reset(new UDPServerHandler(IP_PORT_, false, IP_ADRR_));
    unsigned char tmp_buffer[2*MB_LEN];
    std::vector<unsigned char> msg;
    msg.clear();

    while (ros::ok()) {
        std::size_t valread = udp_handler_ptr->udp_recvfrom(tmp_buffer, sizeof(groudtruth_state_t));
        if (valread <= 0) {
            if (valread < 0) {
                ROS_ERROR_STREAM("[groundtruth] Connect error ");
                msg.clear();
                udp_handler_ptr.reset(new UDPServerHandler(IP_PORT_, false, IP_ADRR_));
                continue;
            } else {
                ROS_ERROR_STREAM("[groundtruth] Client Disconnect ");
                msg.clear();
                continue;
            }
        }
        std::vector<unsigned char> read_vec(tmp_buffer, tmp_buffer+valread);
        msg.insert(msg.end(), read_vec.begin(), read_vec.end());
        if (msg.size() >= sizeof(groudtruth_state_t)/sizeof(uint8_t)) {
            auto ros_msg = groudtruth_state_decode(msg.data());
            OdomPublisher_.publish(ros_msg);
            std::vector<unsigned char> rest_vec(msg.data()+sizeof(groudtruth_state_t)/sizeof(uint8_t), msg.data()+msg.size());
            msg.swap(rest_vec);
        }
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(GroundTruthDriver, nodelet::Nodelet);