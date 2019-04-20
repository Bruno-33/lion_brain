#include <core/lion_thread.h>
#include <lion_brain.h>
#include <unistd.h>

using namespace lion;
using std::thread;

Thread::Thread(Robot* robot)
    : robot(robot)
{
    running = new bool[MAX_THREAD_TYPE_NUM];
    delay   = new int[MAX_THREAD_TYPE_NUM];

    for (int i = 0; i < MAX_THREAD_TYPE_NUM; i++)
        running[i] = false;

    delay[SERIAL_READ_THREAD] = 10;     // 10us
    delay[VIRTUAL_RC_THREAD]  = 10000;  // 10ms

    callbacks.clear();
    callbacks.push_back(std::bind(&Thread::serial_read_callback, this, robot));
    callbacks.push_back(std::bind(&Thread::virtual_rc_callback, this, robot));
}

Thread::~Thread()
{
    for (int i = 0; i < MAX_THREAD_TYPE_NUM; i++)
        stop(i);

    if (running)
    {
        delete running;
        running = NULL;
    }
    
    if (delay)
    {
        delete delay;
        delay = NULL;
    }
    
    robot = NULL;
}

bool Thread::start(int type)
{
    if (!running[type])
    {
        try
        {
            thread th(callbacks[type]);
            th.detach();

            running[type] = true;
        }
        catch (...)
        {
            return false;
        }

        return true;
    }
    else
    {
        return true;
    }
}

bool Thread::stop(int type)
{
    running[type] = false;

    // wait for stop
    usleep(2.0 * delay[type]);
    
    return true;
}

// unsafe
bool Thread::restart(int type)
{
    if (running[type])
        stop(type);

    start(type);

    return true;
}

void Thread::serial_read_callback(Robot* robot)
{
    while(running[SERIAL_READ_THREAD])
    {
        robot->protocol->read();
        usleep(delay[SERIAL_READ_THREAD]);
    }
}

void Thread::virtual_rc_callback(Robot* robot)
{
    while(running[VIRTUAL_RC_THREAD])
    {
        robot->control->send_virtual_rc_command();
        usleep(delay[VIRTUAL_RC_THREAD]); // 10ms
    }
}
