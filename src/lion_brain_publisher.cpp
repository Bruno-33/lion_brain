#include "lion_brain_node.h"
#include "core/lion_type.h"

using namespace lion;

// publish callback

// void publish_imu_msg(ros::Publisher publisher, imu_msg_t imu)
// {
//     // sensor_msgs::Imu msg;
//     // msg.header.stamp = ros::Time::now();
//     // msg.header.frame_id = "base_link";

//     // msg.orientation.x = imu.q.q0;
//     // msg.orientation.y = imu.q.q1;
//     // msg.orientation.z = imu.q.q2;
//     // msg.orientation.w = imu.q.q3;

//     // msg.linear_acceleration.x = imu.acc.x;
//     // msg.linear_acceleration.y = imu.acc.y;
//     // msg.linear_acceleration.z = imu.acc.z;

//     // msg.angular_velocity.x = imu.gyro.x;
//     // msg.angular_velocity.y = imu.gyro.y;
//     // msg.angular_velocity.z = imu.gyro.z;

//     // publisher.publish(msg);
// }

void LionBrainNode::publish_rc_msg(robot_rc_t rc)
{
    //ROS_WARN("publishing rc msg");
    lion_brain::rc msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";

    msg.x = rc.x;
    msg.y = rc.y;
    msg.z = rc.z;

    msg.yaw = rc.yaw;
    msg.pitch = rc.pitch;

    msg.input_mode = rc.input_mode;
    msg.assist_mode = rc.assist_mode;

    rc_publisher.publish(msg);
}
