#include "Climber.h"

//i took out all the pseudo code comments and put them here:
//https://docs.google.com/document/d/1I5caybg-bhfEYwxfIL1PCXAbqdznW7Qo1R-ZHObqCiY/edit?usp=sharing

//Constructor
Climber::Climber(){
    motorPIDController.SetTolerance(ClimbConstants::motorPoseTolerance, ClimbConstants::deltaMotorPoseTolerance);
    //motorProfiledPID.SetTolerance(ClimbConstants::motorPoseTolerance, ClimbConstants::deltaMotorPoseTolerance);

    climbFullExtend.Set(true);
    climbMedExtend.Set(false);

    gearboxSlave.SetInverted(false);
    gearboxSlave.Follow(gearboxMaster);
    gearboxMaster.SetInverted(false);
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
            frc::SmartDashboard::PutString("Climber state", "IDLE");
            SetState(Idle(passIdle)); 
            break;
        case VERTICAL_ARM_EXTEND: //needs human input to move to retracting
            frc::SmartDashboard::PutString("Climber state", "VERTICAL_ARM_EXTEND");
            SetState(VerticalArmExtend(drivenForward));
            break;
         case VERTICAL_ARM_RETRACT:
            frc::SmartDashboard::PutString("Climber state", "VERTICAL_ARM_RETRACT");
            SetState(VerticalArmRetract(pitch, delta_pitch, retryFirstClimb));
            break;
        case TEST_DIAGONAL_ARM_EXTEND:
            frc::SmartDashboard::PutString("Climber state", "TEST_DIAGONAL_ARM_EXTEND");
            SetState(TestDiagonalArmExtend());
            break;
         case DIAGONAL_ARM_EXTEND:
            frc::SmartDashboard::PutString("Climber state", "DIAGONAL_ARM_EXTEND");
            SetState(DiagonalArmExtend(pitch, delta_pitch));
            break;
         case DIAGONAL_ARM_RAISE:  //needs human input to move on
            frc::SmartDashboard::PutString("Climber state", "DIAGONAL_ARM_RAISE");
            SetState(DiagonalArmRaise(passDiagonalArmRaise));
            break;
         case DIAGONAL_ARM_RETRACT: //needs human input to do second climb
            frc::SmartDashboard::PutString("Climber state", "DIAGONAL_ARM_RETRACT");
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

    //for debugging
    // frc::SmartDashboard::PutBoolean("Pass idle", passIdle);
    // frc::SmartDashboard::PutNumber("curr time", currTime);

    if (passIdle && currTime <= ClimbConstants::idleEnoughTime) { std::cout << "moving on to vertical arm extend\n"; return VERTICAL_ARM_EXTEND; }
    else return IDLE;
}


Climber::State Climber::VerticalArmExtend(bool drivenForward){
    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);
    gearboxMaster.Set(ControlMode::PercentOutput, 
        std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
        ClimbConstants::motorExtendedPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));

    //for debugging
    // frc::SmartDashboard::PutBoolean("Driven forward", drivenForward);
    // frc::SmartDashboard::PutNumber("Curr time", currTime);
    // frc::SmartDashboard::PutBoolean("motor done", motorDone(ClimbConstants::motorExtendedPose));

    if (drivenForward && currTime <= ClimbConstants::verticalArmExtendEnoughTime && motorDone(ClimbConstants::motorExtendedPose)) { 
        std::cout << "moving on to vertical arm retract\n"; return VERTICAL_ARM_RETRACT; }
    else return VERTICAL_ARM_EXTEND;
}

Climber::State Climber::VerticalArmRetract(double pitch, double delta_pitch, bool retry){
    if (retry) { std::cout << "retrying\n"; return VERTICAL_ARM_EXTEND; }

    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);

    gearboxMaster.Set(ControlMode::PercentOutput, 
        std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
        ClimbConstants::motorRetractedPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));
    
    if (/*motorDone(ClimbConstants::motorRetractedPose) ||*/ currTime >= ClimbConstants::almostDoneTime) {
        gearboxMaster.SetNeutralMode(NeutralMode::Brake);
        brake.Set(true);
    }

    //for debugging
    // frc::SmartDashboard::PutBoolean("motor done", motorDone(ClimbConstants::motorRetractedPose));
    // frc::SmartDashboard::PutBoolean("curr time", currTime);
    // frc::SmartDashboard::PutBoolean("pitch good", pitchGood(pitch, delta_pitch));
    
    if (motorDone(ClimbConstants::motorRetractedPose) && currTime <= ClimbConstants::verticalArmRetractEnoughTime
        && pitchGood(pitch, delta_pitch)) { std::cout << "moving on to test diagonal arm extend\n"; return TEST_DIAGONAL_ARM_EXTEND; }
    else return VERTICAL_ARM_RETRACT;

}

Climber::State Climber::TestDiagonalArmExtend() {
    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);

   gearboxMaster.Set(ControlMode::PercentOutput, 
        std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
        ClimbConstants::motorTestExtendPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));

    if (stateJustChanged()) waitStartTime = currTime;

    //for debugging
    // frc::SmartDashboard::PutBoolean("hooked", hooked());
    // frc::SmartDashboard::PutNumber("current", gearboxMaster.GetStatorCurrent());

    if (!waited(ClimbConstants::timeToTestExtension, waitStartTime)) {
        return TEST_DIAGONAL_ARM_EXTEND;
    }
    else if (hooked()) { std::cout << "moving on to diagonal arm extend\n"; return DIAGONAL_ARM_EXTEND; }
    else { std::cout << "returning to vertical arm retract\n"; return VERTICAL_ARM_RETRACT; }
}


Climber::State Climber::DiagonalArmExtend(double pitch, double delta_pitch){
    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);

    if (pitchVeryBad(pitch, delta_pitch)) pitchBad = true;
    if (pitchGood(pitch, delta_pitch)) pitchBad = false;

    //if pitch very bad, halt extension
    if (pitchBad) {
        std::cout << "pitch very bad\n";
        gearboxMaster.Set(ControlMode::PercentOutput, 0);
        gearboxMaster.SetNeutralMode(NeutralMode::Brake);
        waitStartTime = currTime; //so that we start over again when we try to re-extend solenoids
        return DIAGONAL_ARM_EXTEND;
    }

    if (stateJustChanged()) waitStartTime = currTime; //so only at start of state. i might try to implement a better way to determine this
    climbMedExtend.Set(false);
    climbFullExtend.Set(false);
    if (waited(ClimbConstants::diagonalArmExtendWaitTime, waitStartTime)) {
        std::cout << "extending motors in diagonal arm extend\n";
        gearboxMaster.Set(ControlMode::PercentOutput, 
            std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
            ClimbConstants::motorExtendedPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));
    }

    //for debugging
    // frc::SmartDashboard::PutBoolean("motor done", motorDone(ClimbConstants::motorExtendedPose));
    // frc::SmartDashboard::PutBoolean("pitch good", pitchGood(pitch, delta_pitch);
    // frc::SmartDashboard::PutBoolean("curr time", currTime);
    
    if (motorDone(ClimbConstants::motorExtendedPose) && pitchGood(pitch, delta_pitch) && currTime <= ClimbConstants::diagonalArmExtendEnoughTime) {
       std::cout << "moving on to diagonal arm raise\n"; return DIAGONAL_ARM_RAISE; }
    else return DIAGONAL_ARM_EXTEND;

}

Climber::State Climber::DiagonalArmRaise(bool passDiagonalArmRaise){
    //perhaps be in brake here?
    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);

    if (stateJustChanged()) waitStartTime = currTime;
    climbMedExtend.Set(false);
    climbFullExtend.Set(true);

    //for debugging
    // frc::SmartDashboard::PutBoolean("waited", waited(ClimbConstants::diagonalArmRaiseWaitTime, waitStartTime));
    // frc::SmartDashboard::PutBoolean("pass diagonal arm raise", passDiagonalArmRaise);
    // frc::SmartDashboard::PutNumber("curr time", currTime);

    if (waited(ClimbConstants::diagonalArmRaiseWaitTime, waitStartTime) && passDiagonalArmRaise && currTime <= ClimbConstants::diagonalArmRaiseEnoughTime) 
        { std::cout << "moving on to diagonal arm retract\n"; return DIAGONAL_ARM_RETRACT; }
    else return DIAGONAL_ARM_RAISE;
}

Climber::State Climber::DiagonalArmRetract(bool doSecondClimb, double pitch, double delta_pitch){
    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);

    gearboxMaster.Set(ControlMode::PercentOutput, 
        std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
        ClimbConstants::motorRetractedPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));
    if (stateJustChanged()) waitStartTime = currTime;
    if (waited(ClimbConstants::waitToRaiseVerticalTime, waitStartTime)) {
        climbFullExtend.Set(true);
        climbMedExtend.Set(true);
    }

    if (/*motorDone(ClimbConstants::motorRetractedPose) ||*/ currTime >= ClimbConstants::almostDoneTime) {
        gearboxMaster.SetNeutralMode(NeutralMode::Brake);
        brake.Set(true);
    }

    //for debugging
    // frc::SmartDashboard::PutBoolean("motor done", motorDone(ClimbConstants::motorRetractedPose));
    // frc::SmartDashboard::PutBoolean("pitch good", pitchGood(pitch, delta_pitch));
    // frc::SmartDashboard::PutBoolean("do second climb", doSecondClimb);
    // frc::SmartDashboard::PutNumber("curr time", currTime);


    if (motorDone(ClimbConstants::motorRetractedPose) && pitchGood(pitch, delta_pitch) && doSecondClimb && currTime <= ClimbConstants::diagonalArmRetractEnoughTime) 
        {std::cout << "moving on to test diagonal arm extend\n"; return TEST_DIAGONAL_ARM_EXTEND; }
    else return DIAGONAL_ARM_RETRACT;
}  


//have the static hooks engaged
bool Climber::hooked() {
    return (gearboxMaster.GetStatorCurrent() >= ClimbConstants::hookedCurrent);
}

//is the motor at set point (kinda redundant now that I know about AtSetpoint(), may erase later)
bool Climber::motorDone(double pose) {
    return (motorPIDController.AtSetpoint());

}

//has the climber swinging stabilized enough to continue
//may have to un-function this if we want tolerances to be different for different states, but this helps for readability & re-usability if not
bool Climber::pitchGood(double pitch, double delta_pitch) {
    return (abs(pitch) < ClimbConstants::acceptablePitch && abs(delta_pitch) < ClimbConstants::deltaPitchTolerance);
}

//this is if the pitch is so bad we are going to fall off as diagonal arm extends and we need to bring it back in
bool Climber::pitchVeryBad(double pitch, double delta_pitch) {
    return (abs(pitch) > ClimbConstants::veryBadPitch || abs(delta_pitch) > ClimbConstants::veryBadDeltaPitch);
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

units::length::meter_t Climber::meterPose() {
    //4096 ticks per rot. has to turn 10.39 rot befor actual spool turns once. spool circumference is 0.1 m
    //return units::meter_t{gearboxMaster.GetSelectedSensorPosition() / 4096 / 10.39 * 0.1};
    return units::meter_t{gearboxMaster.GetSelectedSensorPosition()};
}

//Calibration function for whatever
void
Climber::Calibrate(){
    
}
