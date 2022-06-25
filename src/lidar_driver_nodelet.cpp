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

class LidarDriver : public nodelet::Nodelet {
    public:
        void onInit();
    private:
        void threadCB(const ros::TimerEvent&);
        ros::Publisher LidarPublisher_;
        ros::Timer RunOnceTimer_;
        std::string IP_ADRR_;
        std::string topic_name_;
        int IP_PORT_;
};

void LidarDriver::onInit() {
    ros::NodeHandle priv_nh(getPrivateNodeHandle());
    priv_nh.param<std::string>("address", IP_ADRR_, "192.168.10.15");
    priv_nh.param<int>("port", IP_PORT_, 6766);
    priv_nh.param<std::string>("topic",  topic_name_, "imu");
    LidarPublisher_ = priv_nh.advertise<sensor_msgs::PointCloud2>(topic_name_, 10);
    RunOnceTimer_ =  priv_nh.createTimer(ros::Duration(0.01), &LidarDriver::threadCB, this, true);
    RunOnceTimer_.start();
}

void LidarDriver::threadCB(const ros::TimerEvent&) {
    std::cout << "Start receiving lidar data from " << IP_ADRR_ << ":" << IP_PORT_ << std::endl;
    int sockfd, new_socket, portno, opt, valread, buf_len;
    struct sockaddr_in serv_addr;
    int addrlen = sizeof(serv_addr);

    portno = IP_PORT_;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        std::cout << "ERROR open socket" << std::endl;

    opt = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        std::cout << "Socket set wrong" << std::endl;
    }
    buf_len = MB_LEN;//BUF_SIZE;// 2457664;//1048576;
    int len = sizeof(buf_len);
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &buf_len, sizeof(buf_len))) {
        std::cout << "Socket set wrong" << std::endl;
    }
    if (setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &buf_len, sizeof(buf_len))) {
        std::cout << "Socket set wrong" << std::endl;
    }
    getsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &buf_len, (socklen_t*)&len);
	printf("the receive buffer size after setting is %d\n", buf_len);
    getsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &buf_len, (socklen_t*)&len);
	printf("the send buffer size after setting is %d\n", buf_len);
	
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(portno);
    serv_addr.sin_addr.s_addr = inet_addr(IP_ADRR_.c_str());

    if (bind(sockfd, (struct sockaddr*)& serv_addr, sizeof(serv_addr))) {
        std::cout << "Bind error" << std::endl;
    }
    if (listen(sockfd, 3) < 0) {
        std::cout << "listen error" << std::endl;
    }
    std::cout << "listen sucess !" << std::endl;
    if ((new_socket = accept(sockfd, (struct sockaddr *)&serv_addr, (socklen_t*)&addrlen))<0) {
        std::cout << "accept error" << std::endl;
    }
    std::cout << "connect success !" << std::endl;

    unsigned char tmp_buffer[2*MB_LEN];
    std::vector<unsigned char> msg;
    msg.clear();

    while (ros::ok()) {
        valread = read(new_socket, tmp_buffer, MB_LEN);
        if (valread <= 0) {
            if (valread < 0) {
                ROS_ERROR_STREAM("[lidar] Connect error ");
                return;
            } else {
                ROS_ERROR_STREAM("[lidar] Client Disconnect ");
                close(new_socket);
                msg.clear();
                std::cout << " wait for reconnect " << std::endl;
                if ((new_socket = accept(sockfd, (struct sockaddr *)&serv_addr, (socklen_t*)&addrlen))<0) {
                    std::cout << "accept error" << std::endl;
                }
                continue;
            }
        }
        /* scan */ 
        std::vector<unsigned char> read_vec(tmp_buffer, tmp_buffer+valread);
        msg.insert(msg.end(), read_vec.begin(), read_vec.end());
        if (msg.size() >= sizeof(lidar_data_t)/sizeof(uint8_t)) {
            auto ros_msg = lidar_data_decode(msg.data());
            LidarPublisher_.publish(ros_msg);
            std::vector<unsigned char> rest_vec(msg.data()+sizeof(lidar_data_t)/sizeof(uint8_t), msg.data()+msg.size());
            msg.swap(rest_vec);
        }
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(LidarDriver, nodelet::Nodelet);