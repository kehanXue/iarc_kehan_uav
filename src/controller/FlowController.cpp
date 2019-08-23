//
// Created by kehan on 2019/8/22.
//

#include <interface/VoiceInterface.h>
#include "FlowController.h"


vwpp::FlowController::FlowController() :
        cur_task_id(TAKEOFF),
        cur_flow_state(FLOW_START)
{

}


vwpp::FlowController::~FlowController()
= default;


int8_t vwpp::FlowController::run()
{
    if (vwpp::VoiceInterface::getInstance()->isVoiceCommandHasChanged())
    {

    }
    else
    {

    }

    return 0;
}


vwpp::FlowState vwpp::FlowController::getFlowState()
{
    return cur_flow_state;
}
