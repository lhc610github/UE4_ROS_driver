#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class TFPublisher: public nodelet::Nodelet {
    public:
        void onInit();
    private:
        void OdomCB(const nav_msgs::Odometry::ConstPtr &msg);
        // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_ptr_;
        ros::Subscriber odom_sub_;
        std::string base_frame_id;
};

void TFPublisher::onInit() {
    ros::NodeHandle priv_nh(getPrivateNodeHandle());
    ros::NodeHandle nh;
    base_frame_id = nh.getNamespace();
    base_frame_id.erase(0, 1);
    // tf_br_ptr_.reset(new tf2_ros::TransformBroadcaster());
    odom_sub_ = priv_nh.subscribe("odom", 10, &TFPublisher::OdomCB, this);
}

void TFPublisher::OdomCB(const nav_msgs::Odometry::ConstPtr &msg) {
    static tf2_ros::TransformBroadcaster tf_br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header = msg->header;
    transformStamped.child_frame_id = base_frame_id;
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;
    transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;
    // tf_br_ptr_->sendTransform(transformStamped);
    tf_br.sendTransform(transformStamped);
    ROS_WARN_ONCE("publish tf from [%s] to [%s]",
                transformStamped.header.frame_id.c_str(),
                transformStamped.child_frame_id.c_str());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TFPublisher, nodelet::Nodelet);