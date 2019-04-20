#ifndef _LION_THREAD_H_
#define _LION_THREAD_H_

#include <thread>
#include <vector>
#include <functional>

namespace lion
{
class Robot;

class Thread
{
public:
    Thread(Robot* robot);
    ~Thread();

    bool start(int type);

    bool stop(int type);

    bool restart(int type);

private:

    void serial_read_callback(Robot* robot);

    void virtual_rc_callback(Robot* robot);

private:
    Robot* robot;

    bool* running; // flag to discribe whether lion_brain is terminal(to stop threads)

    int* delay;

    std::vector<std::_Bind_helper<false, void (lion::Thread::*)(lion::Robot*), lion::Thread*, lion::Robot*&>::type> callbacks;
    
}; // class thread

}; // namespace 

#endif
