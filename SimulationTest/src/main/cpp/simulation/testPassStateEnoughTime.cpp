#include "testPassStateEnoughTime.h"
enum State{ //feel free to change the names if they suck
            IDLE,

           //only for climb to first bar
            VERTICAL_ARM_EXTEND,
            VERTICAL_ARM_RETRACT,

            TEST_DIAGONAL_ARM_EXTEND,

            //can be used for all subsequent bars
            DIAGONAL_ARM_EXTEND,
            DIAGONAL_ARM_RAISE, //hooks onto bar
            DIAGONAL_ARM_RETRACT //involves retracting & returning to vertical            
        };

testPassStateEnoughTime::testPassStateEnoughTime(Climber& climber) : climber{climber}
{
    init(); 
}

void testPassStateEnoughTime::init()
{ 

}

void testPassStateEnoughTime::periodic()
{ 

}

bool testPassStateEnoughTime::test() 
{ 

}

void testPassStateEnoughTime::setState(State newState) { 

}
State testPassStateEnoughTime::getState() 
{ 
    
}


