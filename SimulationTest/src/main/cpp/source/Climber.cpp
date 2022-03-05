#include "Climber.h"

//i took out all the pseudo code comments and put them here:
//https://docs.google.com/document/d/1I5caybg-bhfEYwxfIL1PCXAbqdznW7Qo1R-ZHObqCiY/edit?usp=sharing

//Constructor
Climber::Climber(){
    motorPIDController.SetTolerance(ClimbConstants::motorPoseTolerance, 1000000000000000); 

    pneumaticsMed();

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
    if (stopped) {
        //std::cout << "stopped\n";
        Stop();
        return;
    }
    currTime = time;

    frc::SmartDashboard::PutBoolean("waiting", false);
    frc::SmartDashboard::PutBoolean("motor done", motorDone(ClimbConstants::motorExtendedPose));

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
            SetState(DiagonalArmExtend(pitch, delta_pitch, passDiagonalArmRaise));
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
    pnemuaticsDown();
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
    pneumaticsVert();
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);
    if (!motorDone(ClimbConstants::motorExtendedPose)) {
        if (TRYING_PERCENT_OUT) {
            gearboxMaster.Set(ControlMode::PercentOutput, -0.5);
        } else {
            gearboxMaster.Set(ControlMode::PercentOutput, 
            std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
            ClimbConstants::motorExtendedPose), -0.7, 0.7));
        }
        
    }
    if (motorDone(ClimbConstants::motorExtendedPose)) {
        gearboxMaster.Set(ControlMode::PercentOutput, 0);
        gearboxMaster.SetNeutralMode(NeutralMode::Brake);
    }


    //for debugging
    // frc::SmartDashboard::PutBoolean("Driven forward", drivenForward);
    // frc::SmartDashboard::PutNumber("Curr time", currTime);
    //frc::SmartDashboard::PutBoolean("motor done", motorDone(ClimbConstants::motorExtendedPose));

    if (drivenForward && currTime <= ClimbConstants::verticalArmExtendEnoughTime && motorDone(ClimbConstants::motorExtendedPose)) { 
        std::cout << "moving on to vertical arm retract\n"; return VERTICAL_ARM_RETRACT; }
    else return VERTICAL_ARM_EXTEND;
}

Climber::State Climber::VerticalArmRetract(double pitch, double delta_pitch, bool retry){
    if (retry) { std::cout << "retrying\n"; return VERTICAL_ARM_EXTEND; }

    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);
    gearboxMaster.Set(ControlMode::PercentOutput, motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition()));
    
    if (currTime >= ClimbConstants::almostDoneTime) {
        gearboxMaster.SetNeutralMode(NeutralMode::Brake);
        brake.Set(true);
    }

    //for debugging
    // frc::SmartDashboard::PutBoolean("motor done", motorDone(ClimbConstants::motorRetractedPose));
    // frc::SmartDashboard::PutBoolean("curr time", currTime);
    // frc::SmartDashboard::PutBoolean("pitch good", pitchGood(pitch, delta_pitch));
    
    if ((motorDone(ClimbConstants::motorRetractedPose) || gearboxMaster.GetStatorCurrent() > 200) && currTime <= ClimbConstants::verticalArmRetractEnoughTime
        && pitchGood(pitch, delta_pitch)) { std::cout << "moving on to test diagonal arm extend\n"; return TEST_DIAGONAL_ARM_EXTEND; }
    else return VERTICAL_ARM_RETRACT;

}

Climber::State Climber::TestDiagonalArmExtend() {
    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);

    if (stateJustChanged()) waitStartTime = currTime;

    gearboxMaster.Set(ControlMode::PercentOutput, -0.05);

    //for debugging
    frc::SmartDashboard::PutBoolean("hooked", hooked());
    //frc::SmartDashboard::PutNumber("current", gearboxMaster.GetStatorCurrent());
    frc::SmartDashboard::PutBoolean("waited", waited(ClimbConstants::timeToTestExtension, waitStartTime));

    if (!waited(ClimbConstants::timeToTestExtension, waitStartTime) || gearboxMaster.GetSelectedSensorPosition() > ClimbConstants::motorTestExtendPose) {
        return TEST_DIAGONAL_ARM_EXTEND;
    }
    else if (hooked()) { std::cout << "moving on to diagonal arm extend\n"; return DIAGONAL_ARM_EXTEND; }
    else { std::cout << "returning to vertical arm retract\n"; return VERTICAL_ARM_RETRACT; }
}


Climber::State Climber::DiagonalArmExtend(double pitch, double delta_pitch, bool passDiagonalArmRaise){
    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);

    if (pitchVeryBad(pitch, delta_pitch)) pitchBad = true;
    if (pitchGood(pitch, delta_pitch)) pitchBad = false;

    //if pitch very bad, halt extension
    if (pitchBad) {
        std::cout << "pitch very bad\n";
        stopped = true; //this way driver can re-start if for some reason climber doesn't
        waitStartTime = currTime; //so that we start over again when we try to re-extend solenoids
        return DIAGONAL_ARM_EXTEND;
    }

    if (stateJustChanged()) waitStartTime = currTime;

    if (!goingTraversal || (goingTraversal && motorDone(ClimbConstants::motorTestExtendPose))) {
        pneumaticsMed();
    }
    
    if (waited(ClimbConstants::diagonalArmExtendWaitTime, waitStartTime) && !motorDone(ClimbConstants::motorExtendedPose)) {
        if (TRYING_PERCENT_OUT) {
            gearboxMaster.Set(ControlMode::PercentOutput, -0.5);
        } else {
            gearboxMaster.Set(ControlMode::PercentOutput, 
            std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
            ClimbConstants::motorExtendedPose), -0.7, 0.7));
        }
        
    }
    if (motorDone(ClimbConstants::motorExtendedPose)) {
        gearboxMaster.Set(ControlMode::PercentOutput, 0);
        gearboxMaster.SetNeutralMode(NeutralMode::Brake);
    }


    //for debugging
    // frc::SmartDashboard::PutBoolean("motor done", motorDone(ClimbConstants::motorExtendedPose));
    // frc::SmartDashboard::PutBoolean("pitch good", pitchGood(pitch, delta_pitch);
    // frc::SmartDashboard::PutBoolean("curr time", currTime);
    
    if (motorDone(ClimbConstants::motorExtendedPose) && pitchGood(pitch, delta_pitch) && (!goingTraversal || passDiagonalArmRaise) && currTime <= ClimbConstants::diagonalArmExtendEnoughTime) {
        if (!goingTraversal) {std::cout << "moving on to diagonal arm raise\n"; return DIAGONAL_ARM_RAISE;}
        if (goingTraversal) {std::cout << "2nd climb, moving on to diagonal arm retract\n"; return DIAGONAL_ARM_RETRACT;}  
    } else return DIAGONAL_ARM_EXTEND;
    return IDLE;
}

Climber::State Climber::DiagonalArmRaise(bool passDiagonalArmRaise){
    //perhaps be in brake here?
    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);
  
    if (stateJustChanged()) waitStartTime = currTime;
    if (waited(ClimbConstants::diagonalArmRaiseWaitTime, waitStartTime)) {
        pneumaticsVert();
    }

    //for debugging
    // frc::SmartDashboard::PutBoolean("waited", waited(ClimbConstants::diagonalArmRaiseWaitTime, waitStartTime));
    // frc::SmartDashboard::PutBoolean("pass diagonal arm raise", passDiagonalArmRaise);
    // frc::SmartDashboard::PutNumber("curr time", currTime);

    if (waited(ClimbConstants::diagonalArmRaiseWaitTime, waitStartTime) && (passDiagonalArmRaise || goingTraversal) && currTime <= ClimbConstants::diagonalArmRaiseEnoughTime) 
        { if (!goingTraversal) {std::cout << "moving on to diagonal arm retract\n"; return DIAGONAL_ARM_RETRACT;}
        if (goingTraversal) {std::cout << "2nd climb, moving on to diagonal arm extend\n"; return DIAGONAL_ARM_EXTEND;}  
    } else return DIAGONAL_ARM_RAISE;
    return IDLE;
}

Climber::State Climber::DiagonalArmRetract(bool doSecondClimb, double pitch, double delta_pitch){
    brake.Set(false);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);

    if (stateJustChanged()) waitStartTime = currTime;

    if (!motorDone(ClimbConstants::motorRetractedPose) && (!goingTraversal || waited(1, waitStartTime))) {
        gearboxMaster.Set(ControlMode::PercentOutput, 
        std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
        ClimbConstants::motorRetractedPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));
    }
    if (motorDone(ClimbConstants::motorRetractedPose)) {
        gearboxMaster.Set(ControlMode::PercentOutput, 0);
        gearboxMaster.SetNeutralMode(NeutralMode::Brake);
    }

    if (!goingTraversal || motorDone(ClimbConstants::motorRetractedPose)) {
        pneumaticsVert();
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
        {std::cout << "moving on to test diagonal arm extend\n"; goingTraversal = true; return TEST_DIAGONAL_ARM_EXTEND; }
    else return DIAGONAL_ARM_RETRACT;
}  


//have the static hooks engaged
bool Climber::hooked() {
    return (gearboxMaster.GetStatorCurrent() <= ClimbConstants::hookedCurrent);
}

//is the motor at set point (kinda redundant now that I know about AtSetpoint(), may erase later)
//done this way b/c on first loop set point hasn't been set yet
bool Climber::motorDone(double pose) {
    return (motorPIDController.AtSetpoint() && abs(pose-gearboxMaster.GetSelectedSensorPosition()) <= ClimbConstants::motorPoseTolerance);

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
    frc::SmartDashboard::PutBoolean("waiting", true);
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

void Climber::pnemuaticsDown() {
    climbFullExtend.Set(true);
    climbMedExtend.Set(false);
}

void Climber::pneumaticsMed() {
    climbFullExtend.Set(false);
    climbMedExtend.Set(false);
}

void Climber::pneumaticsVert() {
    climbFullExtend.Set(false);
    climbMedExtend.Set(true);
}

//Calibration function for whatever
void
Climber::Calibrate(){
    
}
