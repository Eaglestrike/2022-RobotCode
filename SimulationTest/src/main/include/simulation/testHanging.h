#include "Climber.h"
#include "Constants.h"


class testHanging{
    public:
        enum State{
            IDLE,
            WAITING_FOR_EXTEND_BUTTON,
            TESTING
        } ;

        testHanging(Climber & c);
        void initTestHanging();
        void periodic();
        void test();


        void setState(State newState);
        State getState() {return state;}

    private:

        bool waited();
        bool transition();
        bool otherTests();
        bool hooked();



        Climber& climber;

        double hookedCurrent;
        State state;
        State prevState;


        int passed;

};