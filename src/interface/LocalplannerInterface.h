//
// Created by kehan on 2019/8/21.
//

#ifndef IARC_KEHAN_UAV_LOCALPLANNERINTERFACE_H_
#define IARC_KEHAN_UAV_LOCALPLANNERINTERFACE_H_

#include <geometry_msgs/PoseStamped.h>
#include <boost/thread.hpp>
#include <ros/ros.h>

#include "interface/PX4Interface.h"
#include "utils/utils.h"

namespace vwpp
{
    class LocalplannerInterface
    {
    public:

        static LocalplannerInterface* getInstance();

        virtual ~LocalplannerInterface();

        int8_t publishGoalPose(const geometry_msgs::PoseStamped &_goal_pose);

        int8_t publishGoalPoseSmooth(const geometry_msgs::PoseStamped &_goal_pose);

        int8_t cancelAction();

    private:
        LocalplannerInterface();

        LocalplannerInterface(const LocalplannerInterface &);

        LocalplannerInterface &operator=(const LocalplannerInterface &);

        static LocalplannerInterface* instance;
        static boost::mutex mutex_instance;

        ros::NodeHandle nh;
        ros::Publisher goal_pose_pub;
    };
}


#endif //IARC_KEHAN_UAV_LOCALPLANNERINTERFACE_H_
