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

class ImuDriver : public nodelet::Nodelet {
    public:
        void onInit();
    private:
        void threadCB(const ros::TimerEvent&);
        ros::Publisher ImuPublisher_;
        ros::Timer RunOnceTimer_;
        std::string IP_ADRR_;
        std::string topic_name_;
        int IP_PORT_;
};

void ImuDriver::onInit() {
    ros::NodeHandle priv_nh(getPrivateNodeHandle());
    priv_nh.param<std::string>("address", IP_ADRR_, "192.168.10.15");
    priv_nh.param<int>("port", IP_PORT_, 6766);
    priv_nh.param<std::string>("topic",  topic_name_, "imu");
    ImuPublisher_ = priv_nh.advertise<sensor_msgs::Imu>(topic_name_, 10);
    RunOnceTimer_ =  priv_nh.createTimer(ros::Duration(0.01), &ImuDriver::threadCB, this, true);
    RunOnceTimer_.start();
}

void ImuDriver::threadCB(const ros::TimerEvent&) {
    std::cout << "Start receiving imu data from " << IP_ADRR_ << ":" << IP_PORT_ << std::endl;
    int sockfd, valread;
    struct sockaddr_in servaddr;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
        std::cout << "ERROR open socket" << std::endl;

    servaddr.sin_family    = AF_INET;
    servaddr.sin_port = htons(IP_PORT_); 
    servaddr.sin_addr.s_addr = inet_addr(IP_ADRR_.c_str());
    if ( bind(sockfd, (struct sockaddr *)&servaddr, sizeof(sockaddr_in)) < 0)  {
        std::cout << "Bind error" << IP_PORT_<<  std::endl;
    }
    // if ( connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)  {
    //     std::cout << "connect error" << std::endl;
    // }

    unsigned char tmp_buffer[2*MB_LEN];
    std::vector<unsigned char> msg;
    msg.clear();
    // sockaddr src_addr;
    // socklen_t src_addrlen = sizeof(src_addr);

    while (ros::ok()) {
        // valread = read(sockfd, tmp_buffer, MB_LEN);
        int len =  sizeof(sockaddr_in);
        valread = recvfrom(sockfd, tmp_buffer, MB_LEN, MSG_WAITALL, (struct sockaddr *) &servaddr,  (socklen_t*) &len);
        if (valread <= 0) {
            if (valread < 0) {
                ROS_ERROR_STREAM("[imu] Connect error ");
                return;
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