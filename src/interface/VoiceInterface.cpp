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
        else if (msg->data == 04)
        {
            ROS_ERROR("Voice forward");
            cur_voice_command = VOICE_FORWARD;
        }
        else if (msg->data == 05)
        {
            ROS_ERROR("Voice backward");
            cur_voice_command = VOICE_BACKWARD;
        }
        else if (msg->data == 10)
        {
            ROS_ERROR("Voice cancel");
            cur_voice_command = VOICE_STOP;
        }
        else if (msg->data == 02)
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
