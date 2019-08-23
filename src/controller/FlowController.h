//
// Created by kehan on 2019/8/22.
//

#ifndef IARC_KEHAN_UAV_FLOWCONTROLLER_H_
#define IARC_KEHAN_UAV_FLOWCONTROLLER_H_


#include <stdint-gcc.h>
#include "controller/Task.h"
#include "interface/VoiceInterface.h"

namespace vwpp
{
    enum FlowState
    {
        FLOW_START,
        FLOW_PROCESSING,
        FLOW_FINISH
    };

    class FlowController
    {
    public:

        FlowController();

        virtual ~FlowController();

        int8_t run();

        FlowState getFlowState();

    private:

        TaskID cur_task_id;

        FlowState cur_flow_state;

    };
}


#endif //IARC_KEHAN_UAV_FLOWCONTROLLER_H_
