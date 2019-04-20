
#include <lion_brain_node.h>

using lion::LionBrainNode;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lion_brain");

    ros::NodeHandle nh;              // public topics manager
    ros::NodeHandle nh_private("~"); // parameters manager

    LionBrainNode* lion_brain_node = new LionBrainNode(nh, nh_private);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    delete lion_brain_node;
    lion_brain_node = NULL;

    return 0;
}