//
// Created by kehan on 2019/8/22.
//

#ifndef IARC_KEHAN_UAV_VOICEINTERFACE_H_
#define IARC_KEHAN_UAV_VOICEINTERFACE_H_


#include <ros/ros.h>
#include <stdint-gcc.h>
#include <std_msgs/Int8.h>
#include <boost/thread.hpp>

namespace vwpp
{

    enum VoiceCommand
    {
        VOICE_DEFAULT,
        VOICE_TAKEOFF,
        VOICE_FORWARD,
        VOICE_BACKWARD,
        VOICE_UP,
        VOICE_DOWN,
        VOICE_STOP,
        VOICE_LANDING,
        VOICE_CHANGE            // TODO
    };

    class VoiceInterface
    {
    public:
        static VoiceInterface* getInstance();

        virtual ~VoiceInterface();

        int8_t update();

        VoiceCommand getCurVoiceCommand() const;

        bool isVoiceCommandHasChanged() const;

    private:

        VoiceInterface();

        VoiceInterface(const VoiceInterface &);

        VoiceInterface &operator=(const VoiceInterface &);

        static VoiceInterface* instance;
        static boost::mutex mutex_instance;

        void voice_command_cb(const std_msgs::Int8::ConstPtr &msg);

        ros::NodeHandle nh;
        ros::Subscriber voice_command_sub;

        VoiceCommand cur_voice_command;
        VoiceCommand last_voice_command;

        bool voiceCommandHasChanged;
    };
}

#endif //IARC_KEHAN_UAV_VOICEINTERFACE_H_

