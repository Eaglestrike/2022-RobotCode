#include "Climber.h"
#include "Constants.h"
 

class testClimbOneBar {
    public: 
        enum State { 
            IDLE,
            WAITING_FOR_EXTEND_BUTTON,
            TESTING
        };

        testClimbOneBar(Climber& c);
        void initTestClimbOneBar();
        void periodic();

        bool test();

        void setState(State newState);
        State getState() {return state;}

    private:

        bool extending();
        bool retracting();

        Climber& climber;

        State prevState;
        State state;

        int passed;
        int passedExtending;
        int passedRetracting;

};