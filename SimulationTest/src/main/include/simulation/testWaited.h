#include "Climber.h"
#include "Constants.h"
#include "frc/Timer.h"
 

class testWaited {
    public: 
        enum State { 
            IDLE,
            WAITING_FOR_EXTEND_BUTTON,
            TESTING
        };

        testWaited(Climber& c);
        void initTestWaited();
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

        frc::Timer timer;
};