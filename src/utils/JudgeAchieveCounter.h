//
// Created by kehan on 19-7-27.
//

#ifndef IARC_KEHAN_UAV_JUDGEACHIEVECOUNTER_H_
#define IARC_KEHAN_UAV_JUDGEACHIEVECOUNTER_H_


#include <stdint-gcc.h>

namespace vwpp
{

    class JudgeAchieveCounter
    {
    public:

        explicit JudgeAchieveCounter(int64_t _target);

        virtual ~JudgeAchieveCounter();

        bool isAchieve();

    private:

        int64_t target;

        int64_t counter;

    };
}



#endif //IARC_KEHAN_UAV_JUDGEACHIEVECOUNTER_H_
