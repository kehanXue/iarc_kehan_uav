//
// Created by kehan on 19-7-27.
//

#include "Action.h"

using namespace vwpp;


ActionBase::ActionBase() :
        action_id(TRACKINGLINE)
{

}


ActionBase::~ActionBase()
= default;


ActionTrackingLine::ActionTrackingLine(double_t _target_altitude) :
        action_id(TRACKINGLINE),
        target_altitude(_target_altitude)
{

}


ActionTrackingLine::~ActionTrackingLine()
= default;


ActionID ActionTrackingLine::getActionID()
{
    return action_id;
}


TargetVelXYPosZYaw
ActionTrackingLine::calculateVelocity(double_t _cur_line_v_y, double_t _cur_v_yaw, double_t _forward_vel)
{
    // v means data from vision, p means data from px4.

    static vwpp::PIDController
            pid_controller_v_body_y(vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKp(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKi(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKd(),
                                    vwpp::DynamicRecfgInterface::getInstance()->isPidVV2PYHasThreshold(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYThreshold());

    pid_controller_v_body_y.setTarget(0.);
    pid_controller_v_body_y.update(_cur_line_v_y);

    ROS_ERROR("_cur_line_v_y: %lf", _cur_line_v_y);

    geometry_msgs::Vector3Stamped linear_body_vel{};
    linear_body_vel.header.stamp = ros::Time(0);
    linear_body_vel.header.frame_id = DynamicRecfgInterface::getInstance()->getBodyFrameId();
    ROS_WARN("forward velocity: %lf", _forward_vel);
    linear_body_vel.vector.x = _forward_vel;
    linear_body_vel.vector.y = pid_controller_v_body_y.output() *
                               judgeTrackingLineVVelYScaleSigmoidFunction(PX4Interface::getInstance()->getCurZ());
    linear_body_vel.vector.z = 0;

    geometry_msgs::Vector3Stamped linear_local_vel{};
    try
    {
        odom_base_tf_listener.transformVector(DynamicRecfgInterface::getInstance()->getLocalFrameId(),
                                              linear_body_vel, linear_local_vel);
    }
    catch (tf::TransformException &tf_ex)
    {
        ROS_ERROR("%s", tf_ex.what());
        ros::Duration(vwpp::DynamicRecfgInterface::getInstance()->getTfBreakDuration()).sleep();
    }

    TargetVelXYPosZYaw target_vel_xy_pos_z_yaw{};
    target_vel_xy_pos_z_yaw.vx = linear_local_vel.vector.x;
    target_vel_xy_pos_z_yaw.vy = linear_local_vel.vector.y;
    target_vel_xy_pos_z_yaw.pz = this->target_altitude;
    target_vel_xy_pos_z_yaw.yaw = (PX4Interface::getInstance()->getCurYaw() - _cur_v_yaw);

    return target_vel_xy_pos_z_yaw;
}


ActionHovering::ActionHovering(double_t _target_altitude) :
        action_id(HOVERING),
        target_altitude(_target_altitude)
{
    target_yaw = vwpp::PX4Interface::getInstance()->getCurYaw();
}


ActionHovering::~ActionHovering()
= default;


ActionID ActionHovering::getActionId() const
{
    return action_id;
}


TargetVelXYPosZYaw ActionHovering::calculateVelocity(double_t _cur_v_x, double_t _cur_v_y)
{

    static vwpp::PIDController
            pid_controller_v_body_x(vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PXKp(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PXKi(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PXKd(),
                                    vwpp::DynamicRecfgInterface::getInstance()->isPidVV2PXHasThreshold(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PXThreshold());

    static vwpp::PIDController
            pid_controller_v_body_y(vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKp(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKi(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYKd(),
                                    vwpp::DynamicRecfgInterface::getInstance()->isPidVV2PYHasThreshold(),
                                    vwpp::DynamicRecfgInterface::getInstance()->getPidVV2PYThreshold());
    // static vwpp::PIDController
    //         pid_controller_p_local_z(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKp(),
    //                                  vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKi(),
    //                                  vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKd(),
    //                                  vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PZHasThreshold(),
    //                                  vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZThreshold());
    //
    // static vwpp::PIDController
    //         pid_controller_p_local_yaw(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKp(),
    //                                    vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKi(),
    //                                    vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKd(),
    //                                    vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PYawHasThreshold(),
    //                                    vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawThreshold());


    pid_controller_v_body_x.setTarget(0.);
    pid_controller_v_body_y.setTarget(0.);
    // pid_controller_p_local_z.setTarget(this->target_altitude);
    // TODO
    // pid_controller_p_local_yaw.setTarget(this->target_yaw);


    pid_controller_v_body_x.update(_cur_v_x);
    pid_controller_v_body_y.update(_cur_v_y);
    // pid_controller_p_local_z.update(vwpp::PX4Interface::getInstance()->getCurZ());
    // pid_controller_p_local_yaw.update(
    //         convertCurYaw2FabsYawThetaBetweenPI(target_yaw, vwpp::PX4Interface::getInstance()->getCurZ()));

    geometry_msgs::Vector3Stamped linear_body_vel{};
    // linear_body_vel.header.stamp = ros::Time::now();
    linear_body_vel.header.stamp = ros::Time(0);
    linear_body_vel.header.frame_id = DynamicRecfgInterface::getInstance()->getBodyFrameId();
    linear_body_vel.vector.x = pid_controller_v_body_x.output();
    linear_body_vel.vector.y = pid_controller_v_body_y.output();
    linear_body_vel.vector.z = 0;

    geometry_msgs::Vector3Stamped linear_local_vel{};
    try
    {
        odom_base_tf_listener.transformVector(DynamicRecfgInterface::getInstance()->getLocalFrameId(),
                                              linear_body_vel, linear_local_vel);
    }
    catch (tf::TransformException &tf_ex)
    {
        ROS_ERROR("%s", tf_ex.what());
        ros::Duration(vwpp::DynamicRecfgInterface::getInstance()->getTfBreakDuration()).sleep();
    }

    TargetVelXYPosZYaw target_vel_xy_pos_z_yaw{};
    target_vel_xy_pos_z_yaw.vx = linear_local_vel.vector.x;
    target_vel_xy_pos_z_yaw.vy = linear_local_vel.vector.y;
    target_vel_xy_pos_z_yaw.pz = target_altitude;
    target_vel_xy_pos_z_yaw.yaw = target_yaw;

    return target_vel_xy_pos_z_yaw;

    // DroneVelocity drone_velocity{};
    // drone_velocity.x = linear_local_vel.vector.x;
    // drone_velocity.y = linear_local_vel.vector.y;
    // drone_velocity.z = linear_local_vel.vector.z;
    // drone_velocity.yaw = pid_controller_p_local_yaw.output();
    //
    //
    // return drone_velocity;
}


int8_t ActionHovering::resetTargetYaw(double_t _target_yaw)
{
    this->target_yaw = _target_yaw;
    return 0;
}


ActionRotating::ActionRotating(double_t _target_altitude) :
        action_id(ROTATION),
        target_altitude(_target_altitude)
{
    on_p_x = vwpp::PX4Interface::getInstance()->getCurX();
    on_p_y = vwpp::PX4Interface::getInstance()->getCurY();
}


ActionRotating::~ActionRotating()
= default;


ActionID ActionRotating::getActionId() const
{
    return action_id;
}


DroneVelocity ActionRotating::calculateVelocity(double_t _target_yaw, double_t _cur_yaw)
{

    static vwpp::PIDController
            pid_controller_p_local_x(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXKp(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXKi(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXKd(),
                                     vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PXHasThreshold(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXThreshold());

    static vwpp::PIDController
            pid_controller_p_local_y(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYKp(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYKi(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYKd(),
                                     vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PYHasThreshold(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYThreshold());
    static vwpp::PIDController
            pid_controller_p_local_z(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKp(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKi(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKd(),
                                     vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PZHasThreshold(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZThreshold());

    static vwpp::PIDController
            pid_controller_p_local_yaw(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKp(),
                                       vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKi(),
                                       vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKd(),
                                       vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PYawHasThreshold(),
                                       vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawThreshold());

    // TODO Local->Body?
    pid_controller_p_local_x.setTarget(this->on_p_x);
    pid_controller_p_local_y.setTarget(this->on_p_y);
    pid_controller_p_local_z.setTarget(this->target_altitude);
    ROS_ERROR("target altitude: %lf", this->target_altitude);
    pid_controller_p_local_yaw.setTarget(_target_yaw);

    pid_controller_p_local_x.update(vwpp::PX4Interface::getInstance()->getCurX());
    pid_controller_p_local_y.update(vwpp::PX4Interface::getInstance()->getCurY());
    pid_controller_p_local_z.update(vwpp::PX4Interface::getInstance()->getCurZ());
    pid_controller_p_local_yaw.update(convertCurYaw2FabsYawThetaBetweenPI(_target_yaw, _cur_yaw));

    DroneVelocity drone_velocity{};
    drone_velocity.x = pid_controller_p_local_x.output();
    drone_velocity.y = pid_controller_p_local_y.output();
    drone_velocity.z = pid_controller_p_local_z.output();
    drone_velocity.yaw = pid_controller_p_local_yaw.output();

    return drone_velocity;
}


TargetPosXYZYaw ActionRotating::calculateVelocity(double_t _target_yaw)
{
    TargetPosXYZYaw target_pose_xyz_yaw{};
    target_pose_xyz_yaw.px = on_p_x;
    target_pose_xyz_yaw.py = on_p_y;
    target_pose_xyz_yaw.pz = target_altitude;
    target_pose_xyz_yaw.yaw = _target_yaw;

    return target_pose_xyz_yaw;
}


int8_t ActionRotating::resetRotatingOnXY(double_t _hover_x, double_t _hover_y)
{
    on_p_x = _hover_x;
    on_p_y = _hover_y;
}


double_t ActionRotating::getOnPX() const
{
    return on_p_x;
}


double_t ActionRotating::getOnPY() const
{
    return on_p_y;
}


double_t ActionRotating::getTargetAltitude() const
{
    return target_altitude;
}


ActionAdjustAltitude::ActionAdjustAltitude() :
        action_id(ADJUSTALTITUDE)
{
    on_p_x = vwpp::PX4Interface::getInstance()->getCurX();
    on_p_y = vwpp::PX4Interface::getInstance()->getCurY();
    on_p_yaw = vwpp::PX4Interface::getInstance()->getCurYaw();
}


ActionAdjustAltitude::~ActionAdjustAltitude()
= default;


ActionID ActionAdjustAltitude::getActionId() const
{
    return action_id;
}


DroneVelocity ActionAdjustAltitude::calculateVelocity(double_t _target_altitude, double_t _cur_altitude)
{
    static vwpp::PIDController
            pid_controller_p_local_x(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXKp(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXKi(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXKd(),
                                     vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PXHasThreshold(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PXThreshold());

    static vwpp::PIDController
            pid_controller_p_local_y(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYKp(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYKi(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYKd(),
                                     vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PYHasThreshold(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYThreshold());

    static vwpp::PIDController
            pid_controller_p_local_z(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKp(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKi(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZKd(),
                                     vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PZHasThreshold(),
                                     vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PZThreshold());

    static vwpp::PIDController
            pid_controller_p_local_yaw(vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKp(),
                                       vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKi(),
                                       vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawKd(),
                                       vwpp::DynamicRecfgInterface::getInstance()->isPidPV2PYawHasThreshold(),
                                       vwpp::DynamicRecfgInterface::getInstance()->getPidPV2PYawThreshold());

    pid_controller_p_local_x.setTarget(this->on_p_x);
    pid_controller_p_local_y.setTarget(this->on_p_y);
    pid_controller_p_local_z.setTarget(_target_altitude);
    pid_controller_p_local_yaw.setTarget(this->on_p_yaw);

    pid_controller_p_local_x.update(vwpp::PX4Interface::getInstance()->getCurX());
    pid_controller_p_local_y.update(vwpp::PX4Interface::getInstance()->getCurY());
    pid_controller_p_local_z.update(_cur_altitude);
    pid_controller_p_local_yaw.update(
            convertCurYaw2FabsYawThetaBetweenPI(on_p_yaw, vwpp::PX4Interface::getInstance()->getCurYaw()));

    DroneVelocity drone_velocity{};
    drone_velocity.x = pid_controller_p_local_x.output();
    drone_velocity.y = pid_controller_p_local_y.output();
    drone_velocity.z = pid_controller_p_local_z.output();
    drone_velocity.yaw = pid_controller_p_local_yaw.output();

    return drone_velocity;

}


TargetPosXYZYaw ActionAdjustAltitude::calculateVelocity(double_t _target_altitude)
{
    TargetPosXYZYaw target_pos_xyz_yaw{};
    target_pos_xyz_yaw.px = on_p_x;
    target_pos_xyz_yaw.py = on_p_y;
    target_pos_xyz_yaw.pz = _target_altitude;
    target_pos_xyz_yaw.yaw = on_p_yaw;

    return target_pos_xyz_yaw;
}


int8_t ActionAdjustAltitude::setAdjustAltitudeXYYaw(double_t _on_x, double_t _on_y, double_t _on_yaw)
{
    on_p_x = _on_x;
    on_p_y = _on_y;
    on_p_yaw = _on_yaw;

    return 0;
}


ActionCycleMoving::ActionCycleMoving() :
        action_id(CYCLEMOVING)
{

}


ActionCycleMoving::~ActionCycleMoving()
= default;


ActionID ActionCycleMoving::getActionId() const
{
    return TRACKINGLINE;
}


TargetVelXYYawPosZ
ActionCycleMoving::calculateVelocity(double_t _target_altitude, double_t _target_radius, double_t _truth_radius)
{
    geometry_msgs::Vector3Stamped linear_body_vel{};
    linear_body_vel.header.stamp = ros::Time(0);
    linear_body_vel.header.frame_id = DynamicRecfgInterface::getInstance()->getBodyFrameId();
    linear_body_vel.vector.x = 0.;
    linear_body_vel.vector.y = DynamicRecfgInterface::getInstance()->getCycleMovingLinearVel();
    linear_body_vel.vector.z = 0.;

    geometry_msgs::Vector3Stamped linear_local_vel{};
    try
    {
        odom_base_tf_listener.transformVector(DynamicRecfgInterface::getInstance()->getLocalFrameId(),
                                              linear_body_vel, linear_local_vel);
    }
    catch (tf::TransformException &tf_ex)
    {
        ROS_ERROR("%s", tf_ex.what());
        ros::Duration(vwpp::DynamicRecfgInterface::getInstance()->getTfBreakDuration()).sleep();
    }


    static PIDController pid_controller_yaw_rate(DynamicRecfgInterface::getInstance()->getPidVD2YrYawRateKp(),
                                                 DynamicRecfgInterface::getInstance()->getPidVD2YrYawRateKi(),
                                                 DynamicRecfgInterface::getInstance()->getPidVD2YrYawRateKd(),
                                                 DynamicRecfgInterface::getInstance()->isPidVD2YrYawRateHasThreshold(),
                                                 DynamicRecfgInterface::getInstance()->getPidVD2YrYawRateThreshold());
    pid_controller_yaw_rate.setTarget(0.);
    double_t radius_offset = _target_radius - _truth_radius;
    pid_controller_yaw_rate.update(radius_offset);
    double_t yaw_rate_raw = -(linear_body_vel.vector.y / _target_radius);
    double_t yaw_rate = yaw_rate_raw - pid_controller_yaw_rate.output();

    ROS_ERROR("Output yaw rate: %lf", yaw_rate);

    TargetVelXYYawPosZ target_vel_xy_yaw_pos_z{};
    target_vel_xy_yaw_pos_z.vx = linear_local_vel.vector.x;
    target_vel_xy_yaw_pos_z.vy = linear_local_vel.vector.y;
    target_vel_xy_yaw_pos_z.pz = _target_altitude;
    target_vel_xy_yaw_pos_z.yaw_rate = yaw_rate;


    return target_vel_xy_yaw_pos_z;
}


ActionGoToLocalPositionHoldYaw::ActionGoToLocalPositionHoldYaw() :
        action_id(GOTOPOSITION)
{
    target_yaw = PX4Interface::getInstance()->getCurYaw();
}


ActionGoToLocalPositionHoldYaw::~ActionGoToLocalPositionHoldYaw()
= default;


ActionID ActionGoToLocalPositionHoldYaw::getActionId() const
{
    return action_id;
}


TargetPosXYZYaw
ActionGoToLocalPositionHoldYaw::calculateVelocity(geometry_msgs::Point _target_local_point)
{

    TargetPosXYZYaw target_pos_xyz_yaw{};
    target_pos_xyz_yaw.px = _target_local_point.x;
    target_pos_xyz_yaw.py = _target_local_point.y;
    target_pos_xyz_yaw.pz = _target_local_point.z;
    target_pos_xyz_yaw.yaw = target_yaw;

    return target_pos_xyz_yaw;
}


int8_t ActionGoToLocalPositionHoldYaw::resetTargetYaw(double_t _new_target_yaw)
{
    target_yaw = _new_target_yaw;
    return 0;
}


TargetVelXYPosZYaw ActionGoToLocalPositionHoldYaw::calculateVelocity(TargetPosXYZYaw _target_local_target_by_vel)
{
    static PIDController
            pid_controller_p_local_x(DynamicRecfgInterface::getInstance()->getPidPV2PXKp(),
                                     DynamicRecfgInterface::getInstance()->getPidPV2PXKi(),
                                     DynamicRecfgInterface::getInstance()->getPidPV2PXKd(),
                                     DynamicRecfgInterface::getInstance()->isPidPV2PXHasThreshold(),
                                     DynamicRecfgInterface::getInstance()->getPidPV2PXThreshold());
    static PIDController
            pid_controller_p_local_y(DynamicRecfgInterface::getInstance()->getPidPV2PYKp(),
                                     DynamicRecfgInterface::getInstance()->getPidPV2PYKi(),
                                     DynamicRecfgInterface::getInstance()->getPidPV2PYKd(),
                                     DynamicRecfgInterface::getInstance()->isPidPV2PYHasThreshold(),
                                     DynamicRecfgInterface::getInstance()->getPidPV2PYThreshold());

    pid_controller_p_local_x.setTarget(_target_local_target_by_vel.px);
    pid_controller_p_local_y.setTarget(_target_local_target_by_vel.py);

    pid_controller_p_local_x.update(PX4Interface::getInstance()->getCurX());
    pid_controller_p_local_y.update(PX4Interface::getInstance()->getCurY());

    TargetVelXYPosZYaw target_vel_xy_pos_z_yaw{};
    target_vel_xy_pos_z_yaw.vx = pid_controller_p_local_x.output();
    target_vel_xy_pos_z_yaw.vy = pid_controller_p_local_y.output();
    target_vel_xy_pos_z_yaw.pz = _target_local_target_by_vel.pz;
    target_vel_xy_pos_z_yaw.yaw = _target_local_target_by_vel.yaw;

    return target_vel_xy_pos_z_yaw;
}

