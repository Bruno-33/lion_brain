#ifndef _LION_BRAIN_NODE_H_
#define _LION_BRAIN_NODE_H_

// ROS
#include <ros/ros.h>

// ROS standard msgs
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>

// Custom msgs
// #include <lion_brain/target_position.h>
#include <lion_brain/virtual_rc.h>
#include <lion_brain/rc.h>
#include <lion_brain/chassis_control.h>

// SDK library
#include <lion_brain.h>

namespace lion
{

class LionBrainNode
{
public:
    LionBrainNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~LionBrainNode();

private:
    bool init_robot();
    bool init_subscribers(ros::NodeHandle& nh);
    bool init_publishers(ros::NodeHandle& nh);

    // subscribe callbacks
    void get_target_image_position_callback(const geometry_msgs::PointStampedConstPtr& msg);

    void get_virtual_rc_callback(const lion_brain::virtual_rc::ConstPtr& msg);

    void chassis_control_callback(const lion_brain::chassis_control::ConstPtr& msg);

public:
    // publish callbacks
    void publish_rc_msg(robot_rc_t rc);

private:
    // core
    Robot*      robot;

    // subscribers
    ros::Subscriber target_image_position_subscriber;   // recommand input method
    ros::Subscriber rgb_vision_subscriber;              // image input
    ros::Subscriber depth_vision_subscriber;            // for enginer
    ros::Subscriber virtual_rc_subscriber;              // keyboard and mouse input method
    ros::Subscriber chassis_control_subscriber;         // speed control method

    // publishers
    ros::Publisher  rc_publisher;

    // configurations
    std::string     serial_portname;
    int             baud_rate;

    // enums


}; // class lion brain node

}; // namespace

#endif
