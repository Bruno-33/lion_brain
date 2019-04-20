#include <ros/ros.h>
#include <lion_brain_node.h>

using namespace lion;

LionBrainNode::LionBrainNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
    nh_private.param("serial_name", serial_portname, std::string("/dev/ttyUSB0"));
    nh_private.param("baud_rate",   baud_rate,       115200);

    robot = NULL;

	if(!init_subscribers(nh))
	    ROS_ERROR("Initialize subscribers failed.");

	if(!init_publishers(nh))
	    ROS_ERROR("Initialize publishers failed.");

	if (!init_robot())
	{
	    ROS_ERROR("Initialize robot failed.");
	}
	else
	{
            ROS_INFO("Successful to access robot.");
	}
}

LionBrainNode::~LionBrainNode()
{
    if (robot)
    {
        delete robot;
        robot = NULL;
    }
}

bool LionBrainNode::init_robot()
{
    try
    {
        robot = new Robot(serial_portname, baud_rate, this);
    } 
    catch (IOException& e)
    {
        ROS_ERROR_STREAM("Unable to connect lion_bot via " << serial_portname << ".\n\n"
                         
                         << "Reason: " << e.what() << "\n\n"

                         << "If you are going to run lion_brain within a USB-TTL module:\n"
                         << "  1. Please check if any USB device is existed by using command $ls /dev/ttyUSB*\n"
                         << "  2. Correct the value of parameter \"serial_name\" in launch file\n"
                         << "  3. If you have no permission to access device " << serial_portname << ","
                         << " run the commands below and then re-plug your USB-TTL module to fix this problem:\n"
                         << "    3.1. $roscd lion_brain\n"
                         << "    3.2. $sudo cp support/ch340.rules /etc/udev/rules.d/\n"
                         << "    3.2. $sudo service udev reload\n"
                         << "    3.3. $sudo service udev restart\n"
                         << "  4. Try again :-)\n\n"

                         << "If you are going to run lion_brain on TX2 within its GPIO serial port:\n"
                         << "  1. Make sure the wire connection is correct: J1-5(TX) & J1-6(RX) & J1-19/20(GND)\n"
                         << "  2. Check if /dev/ttyTHS2 is existed by using command $ls /dev/ttyTHS*\n"
                         << "  3. Make sure the value of parameter \"serial_name\" is \"/dev/ttyTHS2\"\n"
                         << "  4. Try again :-)\n"
                         
                        );

        return false;
    }
    catch (...)
    {
        ROS_ERROR_STREAM("Ooops, an unknown error occurs.\n\n"
                         << "Connect teammates to look for help please. :-)\n");
        return false;
    }
    return true;
}

bool LionBrainNode::init_subscribers(ros::NodeHandle& nh)
{
    target_image_position_subscriber = nh.subscribe<geometry_msgs::PointStamped>(
        "/lion_brain/target_position", 10, &LionBrainNode::get_target_image_position_callback, this);

    virtual_rc_subscriber = nh.subscribe<lion_brain::virtual_rc>(
        "/lion_brain/virtual_rc", 10, &LionBrainNode::get_virtual_rc_callback, this);

    chassis_control_subscriber = nh.subscribe<lion_brain::chassis_control>(
        "lion_brain/chassis_control", 10, &LionBrainNode::chassis_control_callback, this);

    return true;
}

bool LionBrainNode::init_publishers(ros::NodeHandle& nh)
{
    rc_publisher = nh.advertise<lion_brain::rc>("/lion_brain/rc", 10);

    return true;
}
