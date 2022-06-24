#ifndef CONFIG_H
#define CONFIG_H
#include <chrono>
#include <ctime>
#include <iostream>
#include <unordered_map>
#include <cstdio>
#include <queue>
#include <vector>
#include <cstring>

/* ros msg type */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>

#define ROW 480
#define COL 640
#define MB_LEN 1048576

typedef struct {
    uint64_t timestamp;
    uint8_t data[ROW*COL];
} img_data_t;

typedef struct {
    uint64_t timestamp;
    float data[ROW*COL];
} depth_img_data_t;

typedef struct {
    uint64_t timestamp;
    float orientation[4]; // x y z w
    float angular_velocity[3];
    float linear_acceleration[3];
} imu_data_t;


sensor_msgs::Image imge_data_decode(unsigned char* data) {
    img_data_t* decode_data = (img_data_t*)(data);
    sensor_msgs::Image image_msg;
    image_msg.header.stamp = ros::Time((uint32_t)(decode_data->timestamp/1000000000),
                                        (uint32_t)(decode_data->timestamp%1000000000));
    image_msg.data.resize(ROW*COL);
    image_msg.height = ROW;
    image_msg.width = COL;
    image_msg.encoding = sensor_msgs::image_encodings::MONO8;
    image_msg.step = COL;
    image_msg.is_bigendian = 0;

    memcpy(image_msg.data.data(), &(decode_data->data), sizeof(decode_data->data));
    return image_msg;
}

sensor_msgs::Image depth_imge_data_decode(unsigned char* data) {
    depth_img_data_t* decode_data = (depth_img_data_t*)(data);
    sensor_msgs::Image image_msg;
    image_msg.header.stamp = ros::Time((uint32_t)(decode_data->timestamp/1000000000),
                                        (uint32_t)(decode_data->timestamp%1000000000));
    image_msg.data.resize(ROW*COL*4);
    image_msg.height = ROW;
    image_msg.width = COL;
    image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    image_msg.step = COL*4;
    image_msg.is_bigendian = 0;

    memcpy(image_msg.data.data(), &(decode_data->data), sizeof(decode_data->data));
    return image_msg;
}

sensor_msgs::Imu imu_data_decode(unsigned char* data) {
    imu_data_t* decode_data = (imu_data_t*)(data);
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time((uint32_t)(decode_data->timestamp/1000000000), 
                                        (uint32_t)(decode_data->timestamp%1000000000));
    imu_msg.orientation.x = decode_data->orientation[0];
    imu_msg.orientation.y = decode_data->orientation[1];
    imu_msg.orientation.z = decode_data->orientation[2];
    imu_msg.orientation.w = decode_data->orientation[3];
    imu_msg.angular_velocity.x = decode_data->angular_velocity[0];
    imu_msg.angular_velocity.y = decode_data->angular_velocity[1];
    imu_msg.angular_velocity.z = decode_data->angular_velocity[2];
    imu_msg.linear_acceleration.x = decode_data->linear_acceleration[0];
    imu_msg.linear_acceleration.y = decode_data->linear_acceleration[1];
    imu_msg.linear_acceleration.z = decode_data->linear_acceleration[2];
    return imu_msg;
}

#endif