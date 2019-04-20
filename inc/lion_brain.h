#ifndef _LION_BRAIN_H_
#define _LION_BRAIN_H_

#include <core/lion_serial.h>
#include <core/lion_protocol.h>
#include <core/lion_control.h>
#include <core/lion_thread.h>
#include <core/lion_type.h>

#include <lion_brain_node.h>

namespace lion
{

class LionBrainNode;

class Robot
{
public:
    Robot(std::string portname, int baudrate, LionBrainNode* ros_node);
    ~Robot();

public:
    Serial*   serial;
    Protocol* protocol;
    Control*  control;

    Thread*   thread_manager;

    LionBrainNode* getNode();

private:
    LionBrainNode* ros_node;
};

}; // namespace 

#endif