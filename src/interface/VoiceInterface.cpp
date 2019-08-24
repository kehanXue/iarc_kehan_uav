//
// Created by kehan on 2019/8/22.
//

#include "VoiceInterface.h"


vwpp::VoiceInterface::VoiceInterface() :
        nh("~"),
        cur_voice_command(VOICE_DEFAULT),
        last_voice_command(VOICE_DEFAULT),
        voiceCommandHasChanged(false)
{
    // TODO topic name
    voice_command_sub = nh.subscribe<std_msgs::Int8>
            ("/voice_command", 1, &VoiceInterface::voice_command_cb, this);
}


vwpp::VoiceInterface::VoiceInterface(const vwpp::VoiceInterface &)
{

}


vwpp::VoiceInterface &vwpp::VoiceInterface::operator=(const vwpp::VoiceInterface &)
{

}


vwpp::VoiceInterface::~VoiceInterface()
{
    delete instance;
}


vwpp::VoiceInterface* vwpp::VoiceInterface::instance = nullptr;

boost::mutex vwpp::VoiceInterface::mutex_instance;

vwpp::VoiceInterface* vwpp::VoiceInterface::getInstance()
{
    if (instance == nullptr)
    {
        boost::unique_lock<boost::mutex> uq_lock_instance(mutex_instance);
        if (instance == nullptr)
        {
            instance = new VoiceInterface();
        }
    }

    return instance;
}


int8_t vwpp::VoiceInterface::update()
{
    try
    {
        ros::spinOnce();
    }
    catch (ros::Exception &ex)
    {
        ROS_ERROR("VoiceInterface Update Error: %s", ex.what());
        return -1;
    }

    return 0;
}


void vwpp::VoiceInterface::voice_command_cb(const std_msgs::Int8::ConstPtr &msg)
{
    // TODO
    ROS_INFO("Voice data: %d", msg->data);
    if (msg->data != 0)
    {
        if (msg->data == 01)
        {
            ROS_ERROR("Voice takeoff");
            cur_voice_command = VOICE_TAKEOFF;
        }
        else if (msg->data == 11)
        {
            ROS_ERROR("Voice forward 0.5");
            cur_voice_command = VOICE_FORWARD_0_5;
        }
        else if (msg->data == 12)
        {
            ROS_ERROR("Voice forward 1");
            cur_voice_command = VOICE_FORWARD_1;
        }
        else if (msg->data == 13)
        {
            ROS_ERROR("Voice forward 2");
            cur_voice_command = VOICE_FORWARD_2;
        }
        else if (msg->data == 14)
        {
            ROS_ERROR("Voice forward 3");
            cur_voice_command = VOICE_FORWARD_3;
        }
        else if (msg->data == 15)
        {
            ROS_ERROR("Voice forward 5");
            cur_voice_command = VOICE_FORWARD_5;
        }
        else if (msg->data == 21)
        {
            ROS_ERROR("Voice backward 0.5");
            cur_voice_command = VOICE_BACKWARD_0_5;
        }
        else if (msg->data == 22)
        {
            ROS_ERROR("Voice backward 1");
            cur_voice_command = VOICE_BACKWARD_1;
        }
        else if (msg->data == 23)
        {
            ROS_ERROR("Voice backward 2");
            cur_voice_command = VOICE_BACKWARD_2;
        }
        else if (msg->data == 24)
        {
            ROS_ERROR("Voice backward 3");
            cur_voice_command = VOICE_BACKWARD_3;
        }
        else if (msg->data == 25)
        {
            ROS_ERROR("Voice backward 5");
            cur_voice_command = VOICE_BACKWARD_5;
        }
        else if (msg->data == 31)
        {
            ROS_ERROR("Voice left 0.5");
            cur_voice_command = VOICE_LEFT_0_5;
        }
        else if (msg->data == 32)
        {
            ROS_ERROR("Voice left 1");
            cur_voice_command = VOICE_LEFT_1;
        }
        else if (msg->data == 33)
        {
            ROS_ERROR("Voice left 2");
            cur_voice_command = VOICE_LEFT_2;
        }
        else if (msg->data == 34)
        {
            ROS_ERROR("Voice left 3");
            cur_voice_command = VOICE_LEFT_3;
        }
        else if (msg->data == 35)
        {
            ROS_ERROR("Voice left 5");
            cur_voice_command = VOICE_LEFT_5;
        }
        else if (msg->data == 41)
        {
            ROS_ERROR("Voice right 0.5");
            cur_voice_command = VOICE_RIGHT_0_5;
        }
        else if (msg->data == 42)
        {
            ROS_ERROR("Voice right 1");
            cur_voice_command = VOICE_RIGHT_1;
        }
        else if (msg->data == 43)
        {
            ROS_ERROR("Voice right 2");
            cur_voice_command = VOICE_RIGHT_2;
        }
        else if (msg->data == 44)
        {
            ROS_ERROR("Voice right 3");
            cur_voice_command = VOICE_RIGHT_3;
        }
        else if (msg->data == 45)
        {
            ROS_ERROR("Voice right 5");
            cur_voice_command = VOICE_RIGHT_5;
        }
        else if (msg->data == 51)
        {
            ROS_ERROR("Voice up 0.5");
            cur_voice_command = VOICE_UP_0_5;
        }
        else if (msg->data == 52)
        {
            ROS_ERROR("Voice up 1");
            cur_voice_command = VOICE_UP_1;
        }
        else if (msg->data == 53)
        {
            ROS_ERROR("Voice up 2");
            cur_voice_command = VOICE_UP_2;
        }
        else if (msg->data == 61)
        {
            ROS_ERROR("Voice down 0.5");
            cur_voice_command = VOICE_DOWN_0_5;
        }
        else if (msg->data == 62)
        {
            ROS_ERROR("Voice down 1");
            cur_voice_command = VOICE_DOWN_1;
        }
        else if (msg->data == 63)
        {
            ROS_ERROR("Voice down 1.5");
            cur_voice_command = VOICE_DOWN_1_5;
        }
        else if (msg->data == 64)
        {
            ROS_ERROR("Voice down 2");
            cur_voice_command = VOICE_DOWN_2;
        }
        else if (msg->data == 71)
        {
            ROS_ERROR("Voice stop");
            cur_voice_command = VOICE_STOP;
        }
        else if (msg->data == 81)
        {
            ROS_ERROR("Voice landing");
            cur_voice_command = VOICE_LANDING;
        }
        else if (msg->data == 91)
        {
            ROS_ERROR("Voice change");
            cur_voice_command = VOICE_CHANGE;
        }


        if (cur_voice_command != last_voice_command)
        {
            voiceCommandHasChanged = true;
            last_voice_command = cur_voice_command;
        }
        else
        {
            voiceCommandHasChanged = false;
        }
    }
}


vwpp::VoiceCommand vwpp::VoiceInterface::getCurVoiceCommand() const
{
    return cur_voice_command;
}


bool vwpp::VoiceInterface::isVoiceCommandHasChanged() const
{
    return voiceCommandHasChanged;
}
