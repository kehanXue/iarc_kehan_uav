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
        VOICE_TAKEOFF,          // 01
        VOICE_FORWARD_0_5,      // 11
        VOICE_FORWARD_1,        // 12
        VOICE_FORWARD_2,        // 13
        VOICE_FORWARD_3,        // 14
        VOICE_FORWARD_5,        // 15
        VOICE_BACKWARD_0_5,     // 21
        VOICE_BACKWARD_1,       // 22
        VOICE_BACKWARD_2,       // 23
        VOICE_BACKWARD_3,       // 24
        VOICE_BACKWARD_5,       // 25
        VOICE_LEFT_0_5,         // 31
        VOICE_LEFT_1,           // 32
        VOICE_LEFT_2,           // 33
        VOICE_LEFT_3,           // 34
        VOICE_LEFT_5,           // 35
        VOICE_RIGHT_0_5,        // 41
        VOICE_RIGHT_1,          // 42
        VOICE_RIGHT_2,          // 43
        VOICE_RIGHT_3,          // 44
        VOICE_RIGHT_5,          // 45
        VOICE_UP_0_5,           // 51
        VOICE_UP_1,             // 52
        VOICE_UP_2,             // 53
        VOICE_DOWN_0_5,         // 61
        VOICE_DOWN_1,           // 62
        VOICE_DOWN_1_5,         // 63
        VOICE_DOWN_2,           // 64
        VOICE_STOP,             // 71
        VOICE_LANDING,          // 81
        VOICE_CHANGE            // 91
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

