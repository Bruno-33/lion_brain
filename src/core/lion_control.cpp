#include <core/lion_control.h>
#include <ros/ros.h>
#include <lion_brain.h>
#include <mutex>

using namespace lion;

Control::Control(Robot* robot)
    : robot(robot)
{
    init_control();

    virtual_rc_command = new uint8_t[VIRTUAL_RC_COMMAND_LENGTH];
}

Control::~Control()
{
    if (virtual_rc_command)
    {
        delete virtual_rc_command;
        virtual_rc_command = NULL;
    }

    robot = NULL;
}

bool Control::init_control()
{
    return true;
}

void Control::fire(float x, float y)
{
    vector3f_t target;
    
    target.x = x;
    target.y = y;
    target.z = 0;

    robot->protocol->send((vector3f_t* )&target, SERIAL_CUSTOM_DATA_CMD_ID, sizeof(target));
}

void Control::track(float x, float y)
{
    robot_pose_t pose;

    pose.chassis.speed = {0, 0, 0};

    pose.gimbal.yaw.angle = -0.15 * x;

    pose.gimbal.pitch.angle = -0.15 * y;

    pose.control_mode = 1;
    
     // ROS_WARN("track");
    robot->protocol->send((robot_pose_t* )&pose, SERIAL_CONTROL_CMD_ID, sizeof(robot_pose_t));
}


void Control::set_virtual_rc_command(const uint8_t* msg)
{
    std::lock_guard<std::mutex> lock(virtual_rc_mutex);
    
    memcpy(virtual_rc_command, msg, VIRTUAL_RC_COMMAND_LENGTH);
}


void Control::send_virtual_rc_command()
{
    std::lock_guard<std::mutex> lock(virtual_rc_mutex);
    
    robot->protocol->send(virtual_rc_command, SERIAL_CONTROL_CMD_ID, VIRTUAL_RC_COMMAND_LENGTH);
}
