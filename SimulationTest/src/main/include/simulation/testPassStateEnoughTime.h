
#include "Climber.h"
#include "Constants.h"

class testPassStateEnoughTime 
{ 
    public:
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
        testPassStateEnoughTime(Climber& climber); 
        void init(); 
        void periodic(int time); 
        void test(State state); 
        void setState(State newState);
        State getState() {return state;}

    private: 
        Climber& climber;

        State prevState;
        State state;
}; 