#include "Climber.h"
#include "Constants.h"
 

class testClimbOneBar {
    public: 
        enum State { 
            IDLE,
            WAITING_FOR_EXTEND_BUTTON,
            EXTENDING,
            WAITING_FOR_RETRACT_BUTTON,
            RETRACTING
        };

        testClimbOneBar(Climber& c);
        void initTestClimbOneBar();
        void periodic();

        bool extending();
        bool retracting();

        void setState(State newState);
        State getState() {return state;}

    private:

        Climber& climber;

        State prevState;
        State state;

        int i = 0; //for command line animation

};