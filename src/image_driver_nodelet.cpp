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
#include "ue4_ros_drivers/tcp_handler.h"

#include <nodelet/nodelet.h>
#include <ros/ros.h>

class ImageDriver : public nodelet::Nodelet {
    public:
        void onInit();
    private:
        // void threadCB(const ros::TimerEvent&);
        void threadCB();
        ros::Publisher ImgPublisher_;
        ros::Publisher ImgInfoPublisher_;
        // ros::Timer RunOnceTimer_;
        std::thread recv_thread_;
        std::thread receive_thread_;
        std::string IP_ADRR_;
        std::string topic_name_;
        std::shared_ptr<TCPServerHandler> tcp_handler_ptr;
        std::string frame_id_;
        int IP_PORT_;
        bool is_depth_img_;
        double fx_, fy_, cx_, cy_;
};

void ImageDriver::onInit() {
    ros::NodeHandle priv_nh(getPrivateNodeHandle());
    frame_id_ = priv_nh.getNamespace();
    frame_id_.erase(0,1);
    priv_nh.param<std::string>("address", IP_ADRR_, "192.168.10.15");
    priv_nh.param<int>("port", IP_PORT_, 6766);
    priv_nh.param<std::string>("topic",  topic_name_, "camera");
    priv_nh.param<bool>("is_depth_img", is_depth_img_, false);
    priv_nh.param<double>("fx", fx_, 320.0);
    priv_nh.param<double>("fy", fy_, 320.0);
    priv_nh.param<double>("cx", cx_, 320.0);
    priv_nh.param<double>("cy", cy_, 240.0);
    ImgPublisher_ = priv_nh.advertise<sensor_msgs::Image>(topic_name_, 10);
    ImgInfoPublisher_ = priv_nh.advertise<sensor_msgs::CameraInfo>(topic_name_+"/camera_info", 10);
    // RunOnceTimer_ = priv_nh.createTimer(ros::Duration(0.01), &ImageDriver::threadCB, this, true);
    // RunOnceTimer_.start();
    recv_thread_ = std::thread(std::bind(&ImageDriver::threadCB, this));
}

// void ImageDriver::threadCB(const ros::TimerEvent&) {
void ImageDriver::threadCB() {
    tcp_handler_ptr.reset(new TCPServerHandler(IP_PORT_, MB_LEN, IP_ADRR_));
    unsigned char tmp_buffer[2*MB_LEN];
    std::vector<unsigned char> msg;
    msg.clear();

    while (ros::ok()) {
        std::size_t valread = tcp_handler_ptr->tcp_read(tmp_buffer, MB_LEN);
        if (valread <= 0) {
            if (valread < 0) {
                std::cout << " Connect error " << std::endl;
                tcp_handler_ptr.reset(new TCPServerHandler(IP_PORT_, MB_LEN, IP_ADRR_));
                continue;
            } else {
                ROS_ERROR_STREAM("[camera] Client Disconnect ");
                tcp_handler_ptr->reconnect();
                msg.clear();
                continue;
            }
        }
        /* scan */ 
        std::vector<unsigned char> read_vec(tmp_buffer, tmp_buffer+valread);
        msg.insert(msg.end(), read_vec.begin(), read_vec.end());
        if (!is_depth_img_) {
            if (msg.size() >= sizeof(img_data_t)/sizeof(uint8_t)) {
                auto ros_msg = imge_data_decode(msg.data());
                ros_msg.header.frame_id = frame_id_;
                ImgPublisher_.publish(ros_msg);
                if (ImgInfoPublisher_.getNumSubscribers() > 0) {
                    sensor_msgs::CameraInfo camera_info;
                    camera_info.header = ros_msg.header;
                    camera_info.header.frame_id = ros_msg.header.frame_id + "_optical_frame";
                    camera_info.height = ros_msg.height;
                    camera_info.width = ros_msg.width;
                    camera_info.distortion_model = "plumb_bob";
                    camera_info.D = std::vector<double>(5, 0.0);
                    camera_info.K = {fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0};
                    camera_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
                    camera_info.P = {fx_, 0.0, cx_, 0.0, 0.0, fy_, cy_, 0.0, 0.0, 0.0, 1.0, 0.0};
                    camera_info.binning_x = 0;
                    camera_info.binning_y = 0;
                    camera_info.roi.x_offset = 0;
                    camera_info.roi.y_offset = 0;
                    camera_info.roi.height = 0;
                    camera_info.roi.width= 0;
                    camera_info.roi.do_rectify = false;
                    ImgInfoPublisher_.publish(camera_info);
                }
                std::vector<unsigned char> rest_vec(msg.data()+sizeof(img_data_t)/sizeof(uint8_t), msg.data()+msg.size());
                msg.swap(rest_vec);
            }
        } else {
            if (msg.size() >= sizeof(depth_img_data_t)/sizeof(uint8_t)) {
                auto ros_msg = depth_imge_data_decode(msg.data());
                ros_msg.header.frame_id = frame_id_;
                ImgPublisher_.publish(ros_msg);
                std::vector<unsigned char> rest_vec(msg.data()+sizeof(depth_img_data_t)/sizeof(uint8_t), msg.data()+msg.size());
                msg.swap(rest_vec);
            }
        }
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ImageDriver, nodelet::Nodelet);