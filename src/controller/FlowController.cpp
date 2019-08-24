//
// Created by kehan on 2019/8/22.
//

#include <interface/VoiceInterface.h>
#include "FlowController.h"


vwpp::FlowController::FlowController() :
        cur_task_id(TAKEOFF),
        cur_flow_state(FLOW_START),
        flag_run_to_global_pose(false)
{
    global_goal_pose.pose.position.x = 20.;
    global_goal_pose.pose.position.y = 4.;
    global_goal_pose.pose.position.z = 3.3;
}


vwpp::FlowController::~FlowController()
= default;


int8_t vwpp::FlowController::run()
{
    if (vwpp::VoiceInterface::getInstance()->isVoiceCommandHasChanged())
    {
        flag_run_to_global_pose = false;
        if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
            == VOICE_FORWARD_0_5)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() + 0.5;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() + 0.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
            == VOICE_FORWARD_1)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() + 1.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() + 0.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_FORWARD_2)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() + 2.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() + 0.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
            == VOICE_FORWARD_3)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() + 3.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() + 0.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_FORWARD_5)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() + 5.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() + 0.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_BACKWARD_0_5)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() - 0.5;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() + 0.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_BACKWARD_1)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() - 1.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() + 0.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_BACKWARD_2)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() - 2.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() + 0.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_BACKWARD_3)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() - 3.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() + 0.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_BACKWARD_5)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() - 5.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() + 0.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_LEFT_0_5)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() + 0.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() + 0.5;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_LEFT_1)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() + 0.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() + 1.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_LEFT_2)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() + 0.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() + 2.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_LEFT_3)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() + 0.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() + 3.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_LEFT_5)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() + 0.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() + 5.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_RIGHT_0_5)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() + 0.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() - 0.5;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_RIGHT_1)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() + 0.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() - 1.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_RIGHT_2)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() + 0.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() - 2.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_RIGHT_3)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() + 0.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() - 3.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_RIGHT_5)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX() + 0.;
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY() - 5.;
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ();
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_UP_0_5)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX();
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY();
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ() + 0.5;
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_UP_1)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX();
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY();
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ() + 1.;
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_UP_2)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX();
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY();
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ() + 2.;
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_DOWN_0_5)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX();
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY();
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ() - 0.5;
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_DOWN_1)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX();
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY();
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ() - 1.;
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_DOWN_2)
        {
            geometry_msgs::PoseStamped target_pose;
            target_pose.pose.position.x = PX4Interface::getInstance()->getCurX();
            target_pose.pose.position.y = PX4Interface::getInstance()->getCurY();
            target_pose.pose.position.z = PX4Interface::getInstance()->getCurZ() - 2.;
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(target_pose);
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_STOP)
        {
            vwpp::LocalplannerInterface::getInstance()->cancelAction();
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_LANDING)
        {
            // this->cur_flow_state = FLOW_FINISH;
            return 0;
        }
        else if (vwpp::VoiceInterface::getInstance()->getCurVoiceCommand()
                 == VOICE_CHANGE)
        {
            flag_run_to_global_pose = true;
        }
    }
    else
    {
        // static int64_t hold_time = 0;
        // if (PX4Interface::getInstance()->getCurZ() > 3.0)
        // {
        //     hold_time++;
        // }
        if (flag_run_to_global_pose)  // && hold_time >= 200)
        {
            vwpp::LocalplannerInterface::getInstance()->publishGoalPose(global_goal_pose);
        }
    }

    this->cur_flow_state = FLOW_PROCESSING;
    return 0;
}


vwpp::FlowState vwpp::FlowController::getFlowState()
{
    return cur_flow_state;
}
