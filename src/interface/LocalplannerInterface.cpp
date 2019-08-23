//
// Created by kehan on 2019/8/21.
//

#include "LocalplannerInterface.h"
#include "DynamicRecfgInterface.h"


vwpp::LocalplannerInterface::LocalplannerInterface() :
        nh("~")
{
    // TODO
    goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/move_base_simple/goal", 1);
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


int8_t vwpp::LocalplannerInterface::cancelAction()
{
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.x = vwpp::PX4Interface::getInstance()->getCurX();
    target_pose.pose.position.y = vwpp::PX4Interface::getInstance()->getCurY();
    target_pose.pose.position.z = vwpp::PX4Interface::getInstance()->getCurZ();

    goal_pose_pub.publish(target_pose);

    return 0;
}


int8_t vwpp::LocalplannerInterface::publishGoalPoseSmooth(const geometry_msgs::PoseStamped &_goal_pose)
{

    geometry_msgs::PoseStamped cur_pose =
            PX4Interface::getInstance()->getCurPose();

    // The vector between goal pose and current pose
    tf::Vector3 vector_3_goal_cur = toTfVector3(subTwoPoint(_goal_pose.pose.position, cur_pose.pose.position));

    double_t length_threshold =
            vwpp::DynamicRecfgInterface::getInstance()->getGoalPositionSmoothLength();
    double_t smoothed_goal_cur_distance =
            vector_3_goal_cur.length() < length_threshold ? vector_3_goal_cur.length() : length_threshold;
    vector_3_goal_cur.normalize();
    vector_3_goal_cur *= smoothed_goal_cur_distance;

    geometry_msgs::PoseStamped smoothed_goal_pose;
    smoothed_goal_pose.pose.position.x = cur_pose.pose.position.x + vector_3_goal_cur.getX();
    smoothed_goal_pose.pose.position.y = cur_pose.pose.position.y + vector_3_goal_cur.getY();
    smoothed_goal_pose.pose.position.z = cur_pose.pose.position.z + vector_3_goal_cur.getZ();

    goal_pose_pub.publish(smoothed_goal_pose);

    return 0;
}

