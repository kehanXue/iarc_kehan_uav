//
// Created by kehan on 19-7-23.
//

#ifndef IARC_KEHAN_UAV_UTILS_H_
#define IARC_KEHAN_UAV_UTILS_H_

#include <cmath>
#include <tf/transform_datatypes.h>

double_t convertCurYaw2FabsYawThetaBetweenPI(double_t _target_yaw, double_t _cur_yaw);

double_t convertYaw2BetweenFabsPI(double_t _yaw);

double_t judgeTrackingLineVVelYScaleSigmoidFunction(double_t _cur_altitude);

template <typename P1, typename P2>
P1 subTwoPoint(const P1& _p_1, const P2& _p_2)
{
    P1 ans_p;
    ans_p.x = _p_1.x - _p_2.x;
    ans_p.y = _p_1.y - _p_2.y;
    ans_p.z = _p_1.z - _p_2.z;

    return ans_p;
}

template <typename P>
tf::Vector3 toTfVector3(const P& point)
{
    return tf::Vector3(point.x, point.y, point.z);
}

#endif //IARC_KEHAN_UAV_UTILS_H_
