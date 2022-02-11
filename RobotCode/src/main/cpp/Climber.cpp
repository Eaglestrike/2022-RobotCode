#include "Climber.h"

//i took out all the pseudo code comments and put them here:
//https://docs.google.com/document/d/1I5caybg-bhfEYwxfIL1PCXAbqdznW7Qo1R-ZHObqCiY/edit?usp=sharing

//Constructor
Climber::Climber(){
    motorPIDController.SetTolerance(ClimbConstants::motorPoseTolerance, ClimbConstants::deltaMotorPoseTolerance);

    climbFullExtend.Set(false);
    climbMedExtend.Set(false);

    gearboxSlave.Follow(gearboxMaster);
    gearboxMaster.SetNeutralMode(NeutralMode::Brake);
    gearboxMaster.SetSelectedSensorPosition(0);   

    brake.Set(true); 
}


//Periodic Function

void
Climber::Periodic(double delta_pitch, double pitch, double time, bool passIdle, bool retryInitClimb, bool drivenForward, bool passDiagonalArmRaise, bool doSecondClimb){
    currTime = time;
    switch(state){
        case IDLE:
            SetState(Idle(passIdle)); 
            break;
        case VERTICAL_ARM_EXTEND: //needs human input to move on
            SetState(VerticalArmExtend(drivenForward));
            break;
         case VERTICAL_ARM_RETRACT:
            SetState(VerticalArmRetract(pitch, delta_pitch, retryInitClimb));
            break;
        case TEST_DIAGONAL_ARM_EXTEND:
            SetState(TestDiagonalArmExtend());
            break;
         case DIAGONAL_ARM_EXTEND:
            SetState(DiagonalArmExtend(pitch, delta_pitch));
            break;
         case DIAGONAL_ARM_RAISE:  //needs human input to move on
            SetState(DiagonalArmRaise(passDiagonalArmRaise));
            break;
         case DIAGONAL_ARM_RETRACT: 
            SetState(DiagonalArmRetract(doSecondClimb));
            break;
        default:
            break;
    }
}

//BE CAREFUL ABOUT CHANGING ORDER OF LINES!!! MAY AFFECT TIME SENSITIVE WAITS!!!


Climber::State Climber::Idle(bool passIdle){
    climbFullExtend.Set(false);
    climbMedExtend.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Brake);
    brake.Set(true);

    if (passIdle && currTime <= ClimbConstants::idleEnoughTime) return VERTICAL_ARM_EXTEND;
    else return IDLE;
}


Climber::State Climber::VerticalArmExtend(bool drivenForward){
    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);
    gearboxMaster.Set(ControlMode::PercentOutput, 
        std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
        ClimbConstants::motorExtendedPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));
    
    if (drivenForward && currTime <= ClimbConstants::verticalArmExtendEnoughTime && motorDone(ClimbConstants::motorExtendedPose)) return VERTICAL_ARM_RETRACT; 
    else return VERTICAL_ARM_EXTEND;
}

Climber::State Climber::VerticalArmRetract(double pitch, double delta_pitch, bool retry){
    if (retry) return VERTICAL_ARM_EXTEND;

    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);

    gearboxMaster.Set(ControlMode::PercentOutput, 
        std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
        ClimbConstants::motorRetractedPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));
    
    if (motorDone(ClimbConstants::motorRetractedPose) || currTime >= ClimbConstants::almostDoneTime) {
        gearboxMaster.SetNeutralMode(NeutralMode::Brake);
        brake.Set(true);
    }
    
    if (motorDone(ClimbConstants::motorRetractedPose) && currTime <= ClimbConstants::verticalArmRetractEnoughTime
        && pitchGood(pitch, delta_pitch)) return TEST_DIAGONAL_ARM_EXTEND;
    else return VERTICAL_ARM_RETRACT;

}

Climber::State Climber::TestDiagonalArmExtend() {
    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);

   gearboxMaster.Set(ControlMode::PercentOutput, 
        std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
        ClimbConstants::motorTestExtendPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));

    if (stateJustChanged()) waitStartTime = currTime;

    if (waited(ClimbConstants::timeToTestExtension, waitStartTime) && hooked()) { std::cout << "hooked\n"; return DIAGONAL_ARM_EXTEND; }
    else { std::cout << "too early or not hooked\n"; return VERTICAL_ARM_RETRACT; }
}


Climber::State Climber::DiagonalArmExtend(double pitch, double delta_pitch){
    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);

    if (stateJustChanged()) waitStartTime = currTime; //so only at start of state. i might try to implement a better way to determine this
    climbMedExtend.Set(true);
    climbFullExtend.Set(true);
    if (waited(ClimbConstants::diagonalArmExtendWaitTime, waitStartTime)) {
        gearboxMaster.Set(ControlMode::PercentOutput, 
            std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
            ClimbConstants::motorExtendedPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));
    }
    
    if (motorDone(ClimbConstants::motorExtendedPose) && pitchGood(pitch, delta_pitch) && currTime <= ClimbConstants::diagonalArmExtendEnoughTime) return DIAGONAL_ARM_RAISE;
    else return DIAGONAL_ARM_EXTEND;

}

Climber::State Climber::DiagonalArmRaise(bool passDiagonalArmRaise){
    //be in brake here?
    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);

    if (stateJustChanged()) waitStartTime = currTime;
    climbFullExtend.Set(false);
    if (waited(ClimbConstants::diagonalArmRaiseWaitTime, waitStartTime) && passDiagonalArmRaise && currTime <= ClimbConstants::diagonalArmRaiseEnoughTime) 
        return DIAGONAL_ARM_RETRACT; 
    else return DIAGONAL_ARM_RAISE;
}

Climber::State Climber::DiagonalArmRetract(bool doSecondClimb){
    gearboxMaster.Set(ControlMode::PercentOutput, 
        std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
        ClimbConstants::motorRetractedPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));
    if (stateJustChanged()) waitStartTime = currTime;
    if (waited(ClimbConstants::waitToRaiseVerticalTime, waitStartTime)) {
        climbFullExtend.Set(false);
        climbMedExtend.Set(false);
    }

    if (motorDone(ClimbConstants::motorRetractedPose) || currTime >= ClimbConstants::almostDoneTime) {
        gearboxMaster.SetNeutralMode(NeutralMode::Brake);
        brake.Set(true);
    }

    if (motorDone(ClimbConstants::motorRetractedPose) && doSecondClimb && currTime <= ClimbConstants::diagonalArmRetractEnoughTime) 
        return TEST_DIAGONAL_ARM_EXTEND;
    else return DIAGONAL_ARM_RETRACT;
}  

//add safety state that slowly lowers robot if engaged on non-static hooks?


bool Climber::hooked() {
    return (gearboxMaster.GetStatorCurrent() >= ClimbConstants::hookedCurrent);
}

//kinda redundant now that i figured out a better way to do this, may erase at some point
bool Climber::motorDone(double pose) {
    return (motorPIDController.AtSetpoint());

}

//may have to un-function this if we want tolerances to be different, but this helps for readability & re-usability if not
bool Climber::pitchGood(double pitch, double delta_pitch) {
    return (pitch < ClimbConstants::acceptablePitch && abs(delta_pitch) < ClimbConstants::deltaPitchTolerance);
}



//set new state function
void
Climber::SetState(State newState){
    prevState = state;
    state = newState;
}

bool Climber::waited(double time, double startTime) {
    return (startTime + time) >= currTime;
}

bool Climber::stateJustChanged() {
    return state != prevState;
}

//Calibration function for whatever
void
Climber::Calibrate(){
    
}
