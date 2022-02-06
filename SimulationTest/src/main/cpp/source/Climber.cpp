#include "Climber.h"

/*Logick
- first arm extend
- drive forward
- first arm retract (climb up)

- hooks flip up onto bar --> arms extend diagonally
- arms tilt up, hooks onto bar
- retract arms (climb onto second bar)

could repeat for a third bar? idk
*/


//Constructor
Climber::Climber(){
    // climbStage1.Set(true);
    // climbStage2.Set(true);
    
}


//Periodic Function
void
Climber::Periodic(double pitch){
    switch(state){
        case State::IDLE:
            state = Idle();
            break;
        case State::INITIALIZE:
            state = Initialize();
            break;
        case State::CLIMB:
            state = Climb();
            break;
        default:
            break;
    }
}


//Idle function
Climber::State
Climber::Idle(){
    // climbStage1.Set(false);
    // climbStage2.Set(true);
}


//Initialization function
Climber::State
Climber::Initialize(){
    // climbStage1.Set(false);
}


//Climb function
Climber::State
Climber::Climb(){

}


//set new state function
void
Climber::setState(State newState){
    state = newState;
}


//Calibration function for whatever
void
Climber::Calibrate(){
    
}
