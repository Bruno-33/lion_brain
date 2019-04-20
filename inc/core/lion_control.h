#ifndef _LION_CONTROL_H_
#define _LION_CONTROL_H_

#include <core/lion_type.h>
#include <mutex>

namespace lion
{
class Robot;

class Control
{
public:
    Control(Robot* robot);
    ~Control();

    void fire(float x = 0, float y = 0);
    void track(float x, float y);

    void set_virtual_rc_command(const uint8_t* msg);
    void send_virtual_rc_command(void);

private:
    bool init_control();

private:
    Robot* robot;
    uint8_t* virtual_rc_command;

    std::mutex virtual_rc_mutex; // a lock to make sure multi-thread-operations on virtual_rc_command is safe

}; // class control

}; // namespace 

#endif
