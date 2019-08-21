//
// Created by kehan on 2019/8/21.
//

#include "LocalplannerInterface.h"


vwpp::LocalplannerInterface::LocalplannerInterface() :
        nh("~")
{
    // TODO
    goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/move_base/simple/goal", 1);
}


vwpp::LocalplannerInterface::LocalplannerInterface(const vwpp::LocalplannerInterface &)
{

}


vwpp::LocalplannerInterface &vwpp::LocalplannerInterface::operator=(const vwpp::LocalplannerInterface &)
{

}


vwpp::LocalplannerInterface::~LocalplannerInterface()
{
    delete instance;
}


vwpp::LocalplannerInterface* vwpp::LocalplannerInterface::instance = nullptr;

boost::mutex vwpp::LocalplannerInterface::mutex_instance;


vwpp::LocalplannerInterface* vwpp::LocalplannerInterface::getInstance()
{
    if (instance == nullptr)
    {
        boost::unique_lock<boost::mutex> uq_lock_instance(mutex_instance);
        if (instance == nullptr)
        {
            instance = new LocalplannerInterface();
        }
    }

    return instance;
}


int8_t vwpp::LocalplannerInterface::publishGoalPose(const geometry_msgs::PoseStamped &_goal_pose)
{
    goal_pose_pub.publish(_goal_pose);

    return 0;
}




