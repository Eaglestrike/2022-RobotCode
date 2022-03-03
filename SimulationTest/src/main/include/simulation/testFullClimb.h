#include "Climber.h"
#include "Constants.h"

//note: only traverses 1 bar

class testFullClimb {
    public:
        testFullClimb();
        void Periodic();
        void Initialize();

    private:
        bool passedIdle = false;
        bool passedVerticalArmExtend = false;
        bool passedVerticalArmRetract = false;
        bool passedtestDiagonalArmExtend = false;
        bool passedDiagonalArmExtend = false;
        bool passedDiagonalArmRaise = false;
        bool passedDiagonalArmRetract = false;

        bool failureEncountered = false;

        void testIdle();
        void testVerticalArmExtend();
        void testVerticalArmRetract();
        void testTestDiagonalArmExtend();
        void testDiagonalArmExtend();
        void testDiagonalArmRaise();
        void testDiagonalArmRetract();

        Climber climber;

};