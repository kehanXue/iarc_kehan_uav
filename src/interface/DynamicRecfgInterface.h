//
// Created by kehan on 19-7-25.
//

#ifndef IARC_KEHAN_UAV_DYNAMICRECFGINTERFACE_H_
#define IARC_KEHAN_UAV_DYNAMICRECFGINTERFACE_H_

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "iarc_kehan_uav/iarc_kehan_uav_dynamic_cfgConfig.h"


namespace vwpp
{
    class DynamicRecfgInterface
    {

    public:

        static DynamicRecfgInterface* getInstance();

        virtual ~DynamicRecfgInterface();

        int8_t update();


    private:

        DynamicRecfgInterface();

        DynamicRecfgInterface(const DynamicRecfgInterface &);

        DynamicRecfgInterface &operator=(const DynamicRecfgInterface &);

        static DynamicRecfgInterface* instance;
        static boost::mutex mutex_instance;

        void reconfig_cb(iarc_kehan_uav::iarc_kehan_uav_dynamic_cfgConfig &_config, uint32_t _level);

        ros::NodeHandle nh;
        dynamic_reconfigure::Server<iarc_kehan_uav::iarc_kehan_uav_dynamic_cfgConfig> dyconfig_server;
        dynamic_reconfigure::Server<iarc_kehan_uav::iarc_kehan_uav_dynamic_cfgConfig>::CallbackType dyconfig_cb_type;

        /* UAV move forward's velocity */
        double_t forward_vel{};

        /* Goal position smooth when use local_planner */
        double_t goal_position_smooth_length{};

        /* The altitude when UAV taking normal flight */
        double_t normal_flight_altitude{};

        /* The altitude when UAV landing */
        double_t landing_altitude{};

        /* Control x,y,z velocity to position use px4 data */
        double_t pid_p_v2p_x_kp{};
        double_t pid_p_v2p_x_ki{};
        double_t pid_p_v2p_x_kd{};
        bool pid_p_v2p_x_has_threshold{};
        double_t pid_p_v2p_x_threshold{};

        double_t pid_p_v2p_y_kp{};
        double_t pid_p_v2p_y_ki{};
        double_t pid_p_v2p_y_kd{};
        bool pid_p_v2p_y_has_threshold{};
        double_t pid_p_v2p_y_threshold{};

        double_t pid_p_v2p_z_kp{};
        double_t pid_p_v2p_z_ki{};
        double_t pid_p_v2p_z_kd{};
        bool pid_p_v2p_z_has_threshold{};
        double_t pid_p_v2p_z_threshold{};

        /* Control x,y,z velocity to position use vision data */
        double_t pid_v_v2p_x_kp{};
        double_t pid_v_v2p_x_ki{};
        double_t pid_v_v2p_x_kd{};
        bool pid_v_v2p_x_has_threshold{};
        double_t pid_v_v2p_x_threshold{};

        double_t pid_v_v2p_y_kp{};
        double_t pid_v_v2p_y_ki{};
        double_t pid_v_v2p_y_kd{};
        bool pid_v_v2p_y_has_threshold{};
        double_t pid_v_v2p_y_threshold{};

        double_t pid_v_v2p_z_kp{};
        double_t pid_v_v2p_z_ki{};
        double_t pid_v_v2p_z_kd{};
        bool pid_v_v2p_z_has_threshold{};
        double_t pid_v_v2p_z_threshold{};

        /* Control yaw velocity to position use px4 data */
        double_t pid_p_v2p_yaw_kp{};
        double_t pid_p_v2p_yaw_ki{};
        double_t pid_p_v2p_yaw_kd{};
        bool pid_p_v2p_yaw_has_threshold{};
        double_t pid_p_v2p_yaw_threshold{};

        /* Control yaw velocity to position use vision data */
        double_t pid_v_v2p_yaw_kp{};
        double_t pid_v_v2p_yaw_ki{};
        double_t pid_v_v2p_yaw_kd{};
        bool pid_v_v2p_yaw_has_threshold{};
        double_t pid_v_v2p_yaw_threshold{};

        /* PID controller while scaning tower. Depth to yaw rate */
        double_t pid_v_d2yr_yaw_rate_kp{};
        double_t pid_v_d2yr_yaw_rate_ki{};
        double_t pid_v_d2yr_yaw_rate_kd{};
        bool pid_v_d2yr_yaw_rate_has_threshold{};
        double_t pid_v_d2yr_yaw_rate_threshold{};

        /* Determine if the altitude has been reached during the flight */
        double_t altitude_tolerance{};

        /* Tolerance when control yaw to rotate. Unit deg*/
        double_t rotate_yaw_tolerance{};

        /* Go to point x, y tolerance */
        double_t goto_point_x_tolerance{};
        double_t goto_point_y_tolerance{};

        /* Tolerance when hovering on QR */
        double_t qr_offset_x_tolerance{};
        double_t qr_offset_y_tolerance{};

        /* When tf query is time out, wait tf_break_duration time */
        double_t tf_break_duration{};

        /* NAVIGATION Task per sustain time */
        int64_t navigation_per_sustain_time{};

        /* Judge whether achieve counter, Unit beat */
        int64_t judge_achieve_counter_threshold{};

        /* Open claw message send frequency */
        int64_t open_claw_msg_send_frequency{};

        /* Radius when cycle moving */
        double_t cycle_moving_radius{};

        /* Linear velocity when cycle moving */
        double_t cycle_moving_linear_vel{};


        /* The parent frame id when tf listener listening */
        std::string local_frame_id{};

        /* The child frame id when tf listener listening */
        std::string body_frame_id{};

    public:
        double_t getForwardVel() const;

        double_t getGoalPositionSmoothLength() const;

        double_t getNormalFlightAltitude() const;

        double_t getLandingAltitude() const;

        double_t getPidPV2PXKp() const;

        double_t getPidPV2PXKi() const;

        double_t getPidPV2PXKd() const;

        bool isPidPV2PXHasThreshold() const;

        double_t getPidPV2PXThreshold() const;

        double_t getPidPV2PYKp() const;

        double_t getPidPV2PYKi() const;

        double_t getPidPV2PYKd() const;

        bool isPidPV2PYHasThreshold() const;

        double_t getPidPV2PYThreshold() const;

        double_t getPidPV2PZKp() const;

        double_t getPidPV2PZKi() const;

        double_t getPidPV2PZKd() const;

        bool isPidPV2PZHasThreshold() const;

        double_t getPidPV2PZThreshold() const;

        double_t getPidVV2PXKp() const;

        double_t getPidVV2PXKi() const;

        double_t getPidVV2PXKd() const;

        bool isPidVV2PXHasThreshold() const;

        double_t getPidVV2PXThreshold() const;

        double_t getPidVV2PYKp() const;

        double_t getPidVV2PYKi() const;

        double_t getPidVV2PYKd() const;

        bool isPidVV2PYHasThreshold() const;

        double_t getPidVV2PYThreshold() const;

        double_t getPidVV2PZKp() const;

        double_t getPidVV2PZKi() const;

        double_t getPidVV2PZKd() const;

        bool isPidVV2PZHasThreshold() const;

        double_t getPidVV2PZThreshold() const;

        double_t getPidPV2PYawKp() const;

        double_t getPidPV2PYawKi() const;

        double_t getPidPV2PYawKd() const;

        bool isPidPV2PYawHasThreshold() const;

        double_t getPidPV2PYawThreshold() const;

        double_t getPidVV2PYawKp() const;

        double_t getPidVV2PYawKi() const;

        double_t getPidVV2PYawKd() const;

        bool isPidVV2PYawHasThreshold() const;

        double_t getPidVV2PYawThreshold() const;

        double_t getPidVD2YrYawRateKp() const;

        double_t getPidVD2YrYawRateKi() const;

        double_t getPidVD2YrYawRateKd() const;

        bool isPidVD2YrYawRateHasThreshold() const;

        double_t getPidVD2YrYawRateThreshold() const;

        double_t getAltitudeTolerance() const;

        double_t getRotateYawTolerance() const;

        double_t getGotoPointXTolerance() const;

        double_t getGotoPointYTolerance() const;

        double_t getQrOffsetXTolerance() const;

        double_t getQrOffsetYTolerance() const;

        double_t getTfBreakDuration() const;

        int64_t getNavigationPerSustainTime() const;

        int64_t getJudgeAchieveCounterThreshold() const;

        int64_t getOpenClawMsgSendFrequency() const;

        double_t getCycleMovingRadius() const;

        double_t getCycleMovingLinearVel() const;

        const std::string &getLocalFrameId() const;

        const std::string &getBodyFrameId() const;

    };


}



#endif //IARC_KEHAN_UAV_DYNAMICRECFGINTERFACE_H_
