#include <lion_brain.h>
#include <lion_brain_node.h>
#include <unistd.h>

using namespace lion;

Robot::Robot(std::string portname, int baudrate, LionBrainNode* ros_node)
    : ros_node(ros_node)
{
    serial = new Serial(portname, baudrate);

    protocol = new Protocol(this);
    control = new Control(this);
    
    thread_manager = new Thread(this);

    thread_manager->start(SERIAL_READ_THREAD);
}

Robot::~Robot()
{
    if (control)
    {
        delete control;
        control = NULL;
    }

    if (protocol)
    {
        delete protocol;
        protocol = NULL;
    }

    if (thread_manager)
    {
        delete thread_manager;
        thread_manager = NULL;
    }

    if (serial)
    {
        delete serial;
        serial = NULL;
    }
    
    ros_node = NULL;
}

LionBrainNode* Robot::getNode()
{
    return ros_node;
}
