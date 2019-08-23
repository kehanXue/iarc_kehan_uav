#include <ros/ros.h>
#include "controller/FlowController.h"
#include "interface/VoiceInterface.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "iarc_kehan_uav_node");
    ros::NodeHandle nh("~");

    std::cout << "\033[32m" << ros::this_node::getName() << " start!"
              << "\033[0m" << std::endl;


    vwpp::VoiceInterface::getInstance()->update();

    vwpp::FlowController flow_controller;
    vwpp::PX4Interface::getInstance()->switchOffboard();
    while (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
           != vwpp::VOICE_TAKEOFF)
    {
        vwpp::VoiceInterface::getInstance()->update();
    }
    vwpp::PX4Interface::getInstance()->unlockVehicle();

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        clock_t time_begin = clock();

        // TODO
        vwpp::DynamicRecfgInterface::getInstance()->update();
        vwpp::PX4Interface::getInstance()->update();
        vwpp::VoiceInterface::getInstance()->update();


        flow_controller.run();
        if (flow_controller.getFlowState() == vwpp::FlowState::FLOW_FINISH)
        {
            std::cout << "\033[32m" << "Mission complete!"
                      << "\033[0m" << std::endl;
            break;
        }

        loop_rate.sleep();
        ROS_ERROR("!!!!!!!!!!!!!!!fps time!!!!!!!!!!!!!!!!");
        ROS_ERROR("%lf\n\n", 1000.0 * (clock() - time_begin) / CLOCKS_PER_SEC);
    }

    return 0;
}
