#include "Climber.h"


//Constructor
Climber::Climber(){
    //both pneumatics retracted
    //motor wound, position zero
    //break on
    
}


//Periodic Function
void
Climber::Periodic(double pitch){
    switch(state){
        case State::IDLE:
            state = Idle();
            break;
        case State::VERTICAL_ARM_EXTEND: //needs human input to move on
            state = VerticalArmExtend();
            break;
         case State::VERTICAL_ARM_RETRACT:
            state = VerticalArmRetract();
            break;
         case State::DIAGONAL_ARM_EXTEND:
            state = DiagonalArmExtend();
            break;
         case State::DIAGONAL_ARM_RAISE:  //needs human input to move on
            state = DiagonalArmRaise();
            break;
         case State::DIAGONAL_ARM_RETRACT: 
            state = DiagonalArmRetract();
            break;
        default:
            break;
    }
}


Climber::State Climber::Idle(){
    //motors all the way wound, double pneumatic both retracted

    //if recieve correct button push && enough time --> return vertical arm extend
    //else --> return idle
}


Climber::State Climber::VerticalArmExtend(){
    //release brake
    //release motor 
    //if correct button push (indicated driven forward) && enough time --> return vertical arm retract
    //else --> return vertical arm extend
}

Climber::State Climber::VerticalArmRetract(){
    //retract motor
    //if motor is fully retracted && enough time && pitch is good --> return test diagonal arm extend
    //else --> return vertical arm retract
}

Climber::State Climber::TestDiagonalArmExtend() {
    //release motor a little bit
    //if hooked --> return diagonal arm extend
    //else --> return vertical arm retract
}


Climber::State Climber::DiagonalArmExtend(){
    //extend both pneumatics (wait before release motor!)
    //release motor
    //possibly wait some more
     //if pneumatics are extended && motor is released && navx is good && enough time --> return diagonal arm raise
    //else --> return diagonal arm extend
}

Climber::State Climber::DiagonalArmRaise(){
    //retract one of two pneumatics
     //if solenoid is done && recieved correct button push && enough time --> return diagonal arm retract
    //else --> return diagonal arm raise
}

Climber::State Climber::DiagonalArmRetract(){
    //retract motor
    //if motor is retracted enough && correct button && enough time --> return test diagonal arm extend
    //else return diagonal arm retract
}  

//add safety state that slowly lowers robot if engaged on non-static hooks


bool Climber::hooked() {
    //if more current --> hooked, true
    //not enough current --> not hooked, false
}



//set new state function
void
Climber::SetState(State newState){
    state = newState;
}


//Calibration function for whatever
void
Climber::Calibrate(){
    
}
