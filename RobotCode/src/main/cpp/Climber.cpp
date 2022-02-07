#include "Climber.h"


//Constructor
Climber::Climber(){
    //both pneumatics retracted
    //motor wound, position zero
    //break on

    climbFullExtend.Set(false);
    climbMedExtend.Set(false);

    gearboxSlave.Follow(gearboxMaster);
    gearboxMaster.SetSelectedSensorPosition(0);   

    brake.Set(true); 
}


//Periodic Function
void
Climber::Periodic(double time, bool passIdle, bool drivenForward, bool passDiagonalArmRaise, bool doSecondClimb){
    currTime = time;
    switch(state){
        case State::IDLE:
            state = Idle(passIdle);
            break;
        case State::VERTICAL_ARM_EXTEND: //needs human input to move on
            state = VerticalArmExtend(drivenForward);
            break;
         case State::VERTICAL_ARM_RETRACT:
            state = VerticalArmRetract();
            break;
         case State::DIAGONAL_ARM_EXTEND:
            state = DiagonalArmExtend();
            break;
         case State::DIAGONAL_ARM_RAISE:  //needs human input to move on
            state = DiagonalArmRaise(passDiagonalArmRaise);
            break;
         case State::DIAGONAL_ARM_RETRACT: 
            state = DiagonalArmRetract(doSecondClimb);
            break;
        default:
            break;
    }
}


Climber::State Climber::Idle(bool passIdle){
    //motors all the way wound, double pneumatic both retracted

    //if recieve correct button push && enough time --> return vertical arm extend
    //else --> return idle

    climbFullExtend.Set(false);
    climbMedExtend.Set(false);

    if (passIdle && currTime <= ClimbConstants::idleEnoughTime) return Climber::VERTICAL_ARM_EXTEND;
    else return Climber::IDLE;
}


Climber::State Climber::VerticalArmExtend(bool drivenForward){
    //release brake
    //release motor 
    //if correct button push (indicated driven forward) && enough time --> return vertical arm retract
    //else --> return vertical arm extend

    brake.Set(false);
    //haven't decided how to release motor yet...
    if (drivenForward && currTime <= ClimbConstants::verticalArmExtendEnoughTime) return Climber::VERTICAL_ARM_RETRACT;
    else return Climber::VERTICAL_ARM_EXTEND;
}

Climber::State Climber::VerticalArmRetract(){
    //retract motor
    //if motor is fully retracted && enough time && pitch is good --> return test diagonal arm extend
    //else --> return vertical arm retract

    //haven't decided how to retract motor yet
    
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

Climber::State Climber::DiagonalArmRaise(bool passDiagonalArmRaise){
    //retract one of two pneumatics
     //if solenoid is done && recieved correct button push && enough time --> return diagonal arm retract
    //else --> return diagonal arm raise
}

Climber::State Climber::DiagonalArmRetract(bool doSecondClimb){
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

bool Climber::waited(double time, double startTime) {
    return (startTime + time) >= currTime;
}

//Calibration function for whatever
void
Climber::Calibrate(){
    
}
