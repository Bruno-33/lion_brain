#include "lion_brain_node.h"
#include "core/lion_type.h"

using namespace lion;

// subscribe callback
void LionBrainNode::get_target_image_position_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
    ROS_INFO("get target: %f, %f", msg->point.x-400, msg->point.y-200);
    robot->control->track(msg->point.x-400, msg->point.y-200);
}


void LionBrainNode::get_virtual_rc_callback(const lion_brain::virtual_rc::ConstPtr& msg)
{
    uint8_t code[VIRTUAL_RC_COMMAND_LENGTH];
    code[14] = msg->code & 0xFF;
    code[15] = (msg->code >> 8) & 0xFF;
    // //线程版本
    // robot->control->set_virtual_rc_command(code);

    // // rise up a thread to send virtual rc commands in each 10ms
    // robot->thread_manager->start(VIRTUAL_RC_THREAD);


    //不启用线程版本
    robot->protocol->send(code, SERIAL_RC_DOWNLOAD_CMD_ID, VIRTUAL_RC_COMMAND_LENGTH);
}

void LionBrainNode::chassis_control_callback(const lion_brain::chassis_control::ConstPtr& msg)
{
    uint8_t code[VIRTUAL_RC_COMMAND_LENGTH];

    int x, y; // mm/s

    // m/s to mm/s
    x = msg->x * 1000;
    y = msg->y * 1000;

    // make sure the max expected speed is no large than 3m/s
    if (x >  3000) x =  3000;
    if (x < -3000) x = -3000;
    if (y >  3000) y =  3000;
    if (y < -3000) y = -3000;

    // turn x,y to range (-660, 660)+660
    x = x * 660 / 3000 + 1024;
    y = y * 660 / 3000 + 1024;

    code[0] = x & 0xFF;
    code[1] = ((x >> 8) & 0x07) | (y << 3);
    code[2] = (y >> 5) & 0x3F;

    robot->protocol->send(code, SERIAL_RC_DOWNLOAD_CMD_ID, VIRTUAL_RC_COMMAND_LENGTH);
}

