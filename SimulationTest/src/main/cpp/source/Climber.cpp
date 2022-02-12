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
Climber::Periodic(double delta_pitch, double pitch, double time, bool passIdle, bool drivenForward, bool retryFirstClimb, bool passDiagonalArmRaise, bool doSecondClimb){
    currTime = time;
    switch(state){
        case IDLE: //needs human input to start climb
            SetState(Idle(passIdle)); 
            break;
        case VERTICAL_ARM_EXTEND: //needs human input to move to retracting
            SetState(VerticalArmExtend(drivenForward));
            break;
         case VERTICAL_ARM_RETRACT:
            SetState(VerticalArmRetract(pitch, delta_pitch, retryFirstClimb));
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
         case DIAGONAL_ARM_RETRACT: //needs human input to do second climb
            SetState(DiagonalArmRetract(doSecondClimb, pitch, delta_pitch));
            break;
        default:
            break;
    }
}

//states (https://docs.google.com/document/d/1I5caybg-bhfEYwxfIL1PCXAbqdznW7Qo1R-ZHObqCiY/edit?usp=sharing)

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

    if (stateJustChanged()) {
        waitStartTime = currTime;
        std::cout << "stateJustChanged, setting waitStartTime - Climber::TestDiagonalArmExtend" << "\n";
        std::cout << waitStartTime << "\n";
    }

    // std::cout << waited(ClimbConstants::timeToTestExtension, waitStartTime) << "\n"; // TESTING

    if (!waited(ClimbConstants::timeToTestExtension, waitStartTime)) {
        return TEST_DIAGONAL_ARM_EXTEND;
    }
    else if (hooked()) return DIAGONAL_ARM_EXTEND; 
    else return VERTICAL_ARM_RETRACT; 
}


Climber::State Climber::DiagonalArmExtend(double pitch, double delta_pitch){
    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);

    if (stateJustChanged()) {
        waitStartTime = currTime;
    } //so only at start of state. i might try to implement a better way to determine this
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
    //perhaps be in brake here?
    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);

    if (stateJustChanged()) waitStartTime = currTime;
    climbFullExtend.Set(false);
    if (waited(ClimbConstants::diagonalArmRaiseWaitTime, waitStartTime) && passDiagonalArmRaise && currTime <= ClimbConstants::diagonalArmRaiseEnoughTime) 
        return DIAGONAL_ARM_RETRACT; 
    else return DIAGONAL_ARM_RAISE;
}

Climber::State Climber::DiagonalArmRetract(bool doSecondClimb, double pitch, double delta_pitch){
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

    if (motorDone(ClimbConstants::motorRetractedPose) && pitchGood(pitch, delta_pitch) && doSecondClimb && currTime <= ClimbConstants::diagonalArmRetractEnoughTime) 
        return TEST_DIAGONAL_ARM_EXTEND;
    else return DIAGONAL_ARM_RETRACT;
}  


//have the static hooks engaged
bool Climber::hooked() {
    // testing!!! (Change back later);
    return true;
    // return (gearboxMaster.GetStatorCurrent() >= ClimbConstants::hookedCurrent);
}

//is the motor at set point (kinda redundant now that I know about AtSetpoint(), may erase later)
bool Climber::motorDone(double pose) {
    return (motorPIDController.AtSetpoint());

}

//has the climber swinging stabilized enough to continue
//may have to un-function this if we want tolerances to be different for different states, but this helps for readability & re-usability if not
bool Climber::pitchGood(double pitch, double delta_pitch) {
    return (pitch < ClimbConstants::acceptablePitch && abs(delta_pitch) < ClimbConstants::deltaPitchTolerance);
}



//set new state function, update prev state (this is important for waited)
void
Climber::SetState(State newState){
    prevState = state;
    state = newState;
}

//has the time passed
bool Climber::waited(double time, double startTime) {
    return (startTime + time) <= currTime;
}

//are we in a new state (this is important for waited)
bool Climber::stateJustChanged() {
    return state != prevState;
}

//Calibration function for whatever
void
Climber::Calibrate(){
    
}
