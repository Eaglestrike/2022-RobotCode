#include "Climber.h"

//Constructor
Climber::Climber(){
    // climbStage1.Set(true);
    // climbStage2.Set(true);
    
}


//Periodic Function
void
Climber::Periodic(){
    switch(state){
        case State::IDLE:
            Idle();
            break;
        case State::INITIALIZE:
            Initialize();
            break;
        case State::CLIMB:
            break;
        default:
            break;
    }
}


//Idle function
void
Climber::Idle(){
    // climbStage1.Set(false);
    // climbStage2.Set(true);
}


//Initialization function
void
Climber::Initialize(){
    // climbStage1.Set(false);
}


//Climb function
void
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