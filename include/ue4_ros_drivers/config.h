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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <mavros_msgs/AttitudeTarget.h>

#include "geometry_math_type.h"

#define ROW 480
#define COL 640
#define MB_LEN 1048576
#define LIDAR_BUF_SIZE 16*1800//480000

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

typedef struct {
    float point[3];
    uint16_t ring;
} point_t;

typedef struct {
    uint64_t timestamp;
    point_t point_cloud[LIDAR_BUF_SIZE];
} lidar_data_t;

typedef struct {
    uint64_t timestamp;
    float attitude_setpoint[3]; // roll pitch yaw
    float throttle_setpoint; // 0-1
} control_cmd_t;

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

sensor_msgs::PointCloud2 lidar_data_decode(unsigned char* data) {
    lidar_data_t* decode_data = (lidar_data_t*)(data);
    sensor_msgs::PointCloud2 lidar_msg;
    lidar_msg.header.stamp = ros::Time((uint32_t)(decode_data->timestamp/1000000000), 
                                        (uint32_t)(decode_data->timestamp%1000000000));
    lidar_msg.height = 1;
    lidar_msg.width = LIDAR_BUF_SIZE;
    lidar_msg.fields.resize(4);
    lidar_msg.fields[0].name = "x";
    lidar_msg.fields[1].name = "y";
    lidar_msg.fields[2].name = "z";
    lidar_msg.fields[3].name = "ring";
    
    int offset = 0;

    for (size_t d = 0; d < lidar_msg.fields.size()-1; ++d, offset += 4) {
        lidar_msg.fields[d].offset = offset;
        lidar_msg.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
        lidar_msg.fields[d].count = 1;
    }
    lidar_msg.fields[3].offset = offset; // 12
    lidar_msg.fields[3].datatype = sensor_msgs::PointField::UINT16;
    lidar_msg.fields[3].count = 1;
    lidar_msg.is_bigendian = false;
    lidar_msg.point_step = offset+2; // 14
    lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;

    lidar_msg.is_dense = true; 
    std::vector<point_t> data_std(decode_data->point_cloud, decode_data->point_cloud + sizeof(point_t) * LIDAR_BUF_SIZE);
    const unsigned char* bytes = reinterpret_cast<const unsigned char*>(data_std.data());
    std::vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(point_t) * LIDAR_BUF_SIZE);
    lidar_msg.data = std::move(lidar_msg_data);
    return lidar_msg;
}

std::vector<unsigned char> control_cmd_encode(const mavros_msgs::AttitudeTarget& msg) {
    control_cmd_t encode_msg;
    encode_msg.timestamp = (uint64_t)(msg.header.stamp.toNSec());
    // encode_msg.attitude_setpoint[0] = msg.orientation.x;
    // encode_msg.attitude_setpoint[1] = msg.orientation.y;
    // encode_msg.attitude_setpoint[2] = msg.orientation.z;
    // encode_msg.attitude_setpoint[3] = msg.orientation.w;
    Eigen::Quaternionf q;
    q.w() = msg.orientation.w;
    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    Eigen::Vector3f euler;
    get_euler_from_q(euler, q);
    encode_msg.attitude_setpoint[0] = euler(0);
    encode_msg.attitude_setpoint[1] = euler(1);
    encode_msg.attitude_setpoint[2] = euler(2);
    encode_msg.throttle_setpoint = std::min(std::max(msg.thrust, 0.0f), 1.0f);
    const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&encode_msg);
    std::vector<unsigned char> encode_bytes(bytes, bytes + sizeof(control_cmd_t));
    return encode_bytes;
}

#endif