#pragma once

#include "Climber.h"
#include "Constants.h"

class testDiagonalExtendPitch
{
    public:
        testDiagonalExtendPitch(Climber& climber);
        void init();
        void periodic();
        void makePitchBad(bool badPitch);

        bool test();

    private:
        Climber& climber;
        bool badPitch = false;
        bool passedInitExtend = false;
        bool passedRetract = false;
        bool passedReextend = false;
};