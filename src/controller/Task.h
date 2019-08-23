//
// Created by kehan on 2019/8/22.
//

#ifndef IARC_KEHAN_UAV_TASK_H_
#define IARC_KEHAN_UAV_TASK_H_


#include <cstdint>
#include "controller/Action.h"

namespace vwpp
{
    enum TaskState
    {
        TASK_START,
        TASK_PROCESSING,
        TASK_FINISH
    };

    enum TaskID
    {
        TAKEOFF = 0,
        FORWARDMOVETO = 1
    };

    class TaskForwardMoveTo
    {
    public:

        TaskForwardMoveTo();

        virtual ~TaskForwardMoveTo();

        int8_t run();
        int8_t resetStartXY(double_t _start_x, double_t _start_y);

    private:

        ActionMoveTo* p_action_move_to;

        TaskID task_id;
        TaskState task_state;

        double_t start_x;
        double_t start_y;
    };

    class TaskForwardPlanTo
    {
    public:
        TaskForwardPlanTo();
        virtual ~TaskForwardPlanTo();

        int8_t run();
        int8_t resetStartXY(double_t _start_x, double_t _start_y);

    private:

        ActionPlanTo* p_action_plan_to;

        TaskID task_id;
        TaskState task_state;

        double_t start_x;
        double_t start_y;
    };


}


#endif //IARC_KEHAN_UAV_TASK_H_
