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

class ImuDriver : public nodelet::Nodelet {
    public:
        void onInit();
    private:
        // void threadCB(const ros::TimerEvent&);
        void threadCB();
        ros::Publisher ImuPublisher_;
        // ros::Timer RunOnceTimer_;
        std::thread recv_thread_;
        std::string IP_ADRR_;
        std::string topic_name_;
        std::shared_ptr<UDPHandler> udp_handler_ptr;
        int IP_PORT_;
};

void ImuDriver::onInit() {
    ros::NodeHandle priv_nh(getPrivateNodeHandle());
    // ros::NodeHandle nh;
    priv_nh.param<std::string>("address", IP_ADRR_, "192.168.10.15");
    priv_nh.param<int>("port", IP_PORT_, 6766);
    priv_nh.param<std::string>("topic",  topic_name_, "imu");
    ImuPublisher_ = priv_nh.advertise<sensor_msgs::Imu>(topic_name_, 10);
    // RunOnceTimer_ =  nh.createTimer(ros::Duration(0.01), &ImuDriver::threadCB, this, true);
    // RunOnceTimer_ =  priv_nh.createTimer(ros::Duration(0.01), &ImuDriver::threadCB, this, true);
    // RunOnceTimer_.start();
    recv_thread_ = std::thread(std::bind(&ImuDriver::threadCB, this));
}

// void ImuDriver::threadCB(const ros::TimerEvent&) {
void ImuDriver::threadCB() {
    udp_handler_ptr.reset(new UDPHandler(IP_PORT_, false, IP_ADRR_));
    unsigned char tmp_buffer[2*MB_LEN];
    std::vector<unsigned char> msg;
    msg.clear();

    while (ros::ok()) {
        std::size_t valread = udp_handler_ptr->udp_recvfrom(tmp_buffer, sizeof(imu_data_t));
        if (valread <= 0) {
            if (valread < 0) {
                ROS_ERROR_STREAM("[imu] Connect error ");
                msg.clear();
                udp_handler_ptr.reset(new UDPHandler(IP_PORT_, false, IP_ADRR_));
                continue;
            } else {
                ROS_ERROR_STREAM("[imu] Client Disconnect ");
                msg.clear();
                continue;
            }
        }
        std::vector<unsigned char> read_vec(tmp_buffer, tmp_buffer+valread);
        msg.insert(msg.end(), read_vec.begin(), read_vec.end());
        if (msg.size() >= sizeof(imu_data_t)/sizeof(uint8_t)) {
            auto ros_msg = imu_data_decode(msg.data());
            ImuPublisher_.publish(ros_msg);
            std::vector<unsigned char> rest_vec(msg.data()+sizeof(imu_data_t)/sizeof(uint8_t), msg.data()+msg.size());
            msg.swap(rest_vec);
        }
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ImuDriver, nodelet::Nodelet);