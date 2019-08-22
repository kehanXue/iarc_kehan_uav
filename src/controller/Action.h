//
// Created by kehan on 19-7-27.
//


#ifndef IARC_KEHAN_UAV_ACTION_H_
#define IARC_KEHAN_UAV_ACTION_H_


#include <cmath>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include "interface/PX4Interface.h"
#include "interface/DynamicRecfgInterface.h"
#include "interface/LocalplannerInterface.h"
#include "controller/PIDController.h"
#include "utils/utils.h"

namespace vwpp
{

    enum ActionID
    {
        ADJUSTALTITUDE = 0,
        HOVERING,
        ROTATION,
        CYCLEMOVING,
        MOVETOHOLDYAW,
        MOVETO,
        PLANTO
    };

    struct DroneVelocity
    {
        double_t x;
        double_t y;
        double_t z;
        double_t yaw;
    };


    class ActionAdjustAltitude
    {
    public:

        ActionAdjustAltitude();

        virtual ~ActionAdjustAltitude();

        ActionID getActionId() const;

        int8_t calculateMotion(double_t _target_altitude);

        int8_t calculateMotion(double_t _target_altitude, double_t _cur_altitude);

        int8_t setAdjustAltitudeXYYaw(double_t _on_x, double_t _on_y, double_t _on_yaw);

    private:

        double_t on_p_x;
        double_t on_p_y;
        double_t on_p_yaw;

        ActionID action_id;

    };


    class ActionHovering
    {
    public:

        explicit ActionHovering(double_t _target_altitude);

        virtual ~ActionHovering();

        ActionID getActionId() const;

        int8_t calculateMotion(double_t _cur_v_x, double_t _cur_v_y);

        int8_t resetTargetYaw(double_t _target_yaw);

    private:

        double_t target_altitude;
        double_t target_yaw;

        tf::TransformListener odom_base_tf_listener;

        ActionID action_id;
    };

    class ActionRotating
    {
    public:

        explicit ActionRotating(double_t _target_altitude);

        virtual ~ActionRotating();

        ActionID getActionId() const;

        int8_t calculateMotion(double_t _target_yaw);

        int8_t calculateMotion(double_t _target_yaw, double_t _cur_yaw);

        int8_t resetRotatingOnXY(double_t _hover_x, double_t _hover_y);

        double_t getOnPX() const;

        double_t getOnPY() const;

        double_t getTargetAltitude() const;

    private:
        double_t target_altitude;

        double_t on_p_x;
        double_t on_p_y;

        ActionID action_id;
    };

    class ActionCycleMoving
    {
    public:

        ActionCycleMoving();

        virtual ~ActionCycleMoving();

        ActionID getActionId() const;

        int8_t
        calculateMotion(double_t _target_altitude, double_t _target_radius, double_t _truth_radius);

    private:

        ActionID action_id;

        tf::TransformListener odom_base_tf_listener;

    };


    class ActionMoveToHoldYaw
    {
    public:

        ActionMoveToHoldYaw();

        virtual ~ActionMoveToHoldYaw();

        ActionID getActionId() const;

        int8_t calculateMotion(geometry_msgs::Point _target_local_point);

        int8_t
        calculateMotion(TargetPosXYZYaw _target_local_target_by_vel);      // TODO another action class

        int8_t resetTargetYaw(double_t _new_target_yaw);

    private:

        ActionID action_id;
        double_t target_yaw;

    };

    class ActionMoveTo
    {
    public:

        ActionMoveTo();

        virtual ~ActionMoveTo();

        ActionID getActionId() const;

        static int8_t calculateMotion(const geometry_msgs::PoseStamped &_target_local_pose);

    private:

        ActionID action_id;
    };

    class ActionPlanTo
    {
    public:
        ActionPlanTo();

        virtual ~ActionPlanTo();

        ActionID getActionId() const;

        static int8_t calculateMotion(geometry_msgs::Point _target_point);

    private:

        ActionID action_id;
    };

}

#endif //IARC_KEHAN_UAV_ACTION_H_
