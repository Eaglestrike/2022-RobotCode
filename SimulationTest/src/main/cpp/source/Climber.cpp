#include "Climber.h"


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
        case State::FIRST_ARM_EXTEND:
            state = FirstArmExtend();
            break;
        case State::DRIVE_FORWARD:
            state = DriveForward();
            break;
         case State::FIRST_ARM_RETRACT:
            state = FirstArmRetract();
            break;
         case State::DIAGONAL_ARM_EXTEND:
            state = DiagonalArmExtend();
            break;
         case State::DIAGONAL_ARM_RAISE:
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
    //all pneumatics retracted
    //if recieve correct button push --> return first arm extend
    //else --> return idle
}


Climber::State Climber::FirstArmExtend(){
    //set correct solenoid to true
    //if solenoid is fully extended (how to determine?) --> return drive forward
    //else --> return first arm extend
}


Climber::State Climber::DriveForward(){
    //basically do nothing, wait for driver to position robot
    //if recieved confirmation from driver --> return first arm retract
    //else --> return drive forward
}


Climber::State Climber::FirstArmRetract(){
    //set correct solenoid to false
    //if solenoid is fully retracted && hooks have booked (how to determine) --> return diagonal arm extend
    //else --> return first arm retract
}


Climber::State Climber::DiagonalArmExtend(){
    //set correct double pneumatic to all the way extended
     //if solenoid is fully extended (how to determine?) && pitch is correct --> return diagonal arm raise
    //else --> return diagonal arm extend
}

Climber::State Climber::DiagonalArmRaise(){
    //set correct double pneumatic halfway extended
     //if solenoid is done getting to halfway position && hooks are hooked (how to determine?) --> return diagonal arm retract
    //else --> return diagonal arm raise
}

Climber::State Climber::DiagonalArmRetract(){
    //set double solenoid to not extended (vertical, initial position)
    //set other solenoid to retracted as well

    //either lock/brake pneumatics, or wait for them both to retract && hook and then call diagonal arm extend
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
