#include "Climber.h"

//this lets us test states individually without having to go through the state machine

//this assumes climber periodic is being called in test periodic, AFTER climbPeriodic()
//the above AFTER is important! otherwise, the state won't be right!

//also call this if no other buttons are pressed
void Climber::InitializeTests() {
    state = IDLE;
    prevState = IDLE;
}

void Climber::Stop() {
    gearboxMaster.Set(ControlMode::PercentOutput, 0);
    gearboxMaster.SetNeutralMode(NeutralMode::Brake);
    // climbMedExtend.Set(false);
    // climbFullExtend.Set(false);
    //std::cout << "stop\n";
    //could turn on pneumatic brake?
}

void Climber::extendArmUntilStopped(bool inverted) {
    // gearboxSlave.SetNeutralMode(NeutralMode::Coast);
    // gearboxMaster.SetNeutralMode(NeutralMode::Coast);
    // if (inverted) gearboxMaster.Set(ControlMode::PercentOutput, -0.25);
    // else gearboxMaster.Set(ControlMode::PercentOutput, 0.25);
    if (gearboxMaster.GetSelectedSensorPosition() > ClimbConstants::motorExtendedPose) {
        if (inverted) gearboxMaster.Set(ControlMode::PercentOutput, -0.25);
        else gearboxMaster.Set(ControlMode::PercentOutput, 0.25);
    }
    else (gearboxMaster.SetNeutralMode(NeutralMode::Brake));
    std::cout << "motor positon: " << gearboxMaster.GetSelectedSensorPosition() << "\n";
    //std::cout << "current: " << gearboxMaster.GetStatorCurrent() << "\n";
    if (gearboxMaster.GetStatorCurrent() > 200) std::cout << "high current, probably wrong direction\n";
}

void Climber::retractArm() {
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);
    if (/*gearboxMaster.GetSelectedSensorPosition() < ClimbConstants::motorRetractedPose*/true) {
        gearboxMaster.Set(ControlMode::PercentOutput, 0.25);
    }
}

void Climber::setArmLowered() {
    climbMedExtend.Set(false);
    climbFullExtend.Set(false);
}

void Climber::setArmRaised() {
    climbMedExtend.Set(false);
    climbFullExtend.Set(true);
}

void Climber::setArmVertical() {
    climbMedExtend.Set(true);
    climbFullExtend.Set(true);
}





void Climber::testRaiseVerticalArm() {
   // SetState(VERTICAL_ARM_EXTEND);
   climbMedExtend.Set(true);
   climbFullExtend.Set(true);
   std::cout << "here\n";    
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);
    gearboxMaster.Set(ControlMode::PercentOutput, 
        std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
        ClimbConstants::motorExtendedPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));
    //gearboxMaster.Set(ControlMode::PercentOutput, motorProfiledPID.Calculate(136167_m));
    
}

void Climber::testRetractVerticalArm() {
  //  SetState(VERTICAL_ARM_RETRACT);
     
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);   
    gearboxMaster.Set(ControlMode::PercentOutput, 
        std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
        ClimbConstants::motorRetractedPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));
    
}

void Climber::testSeesIfHooked() {
    SetState(TEST_DIAGONAL_ARM_EXTEND);
}

void Climber::testDiagonalExtension() {
    //SetState(DIAGONAL_ARM_EXTEND);
    // climbMedExtend.Set(false);
    // climbFullExtend.Set(true);
    climbMedExtend.Set(true);
    climbFullExtend.Set(true);
    //won't do any wait for now, see how it goes
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);
    gearboxMaster.Set(ControlMode::PercentOutput, 
           std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
           ClimbConstants::motorExtendedPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));
}
        
void Climber::testDiagonalArmRaise() {
    //SetState(DIAGONAL_ARM_RAISE);
    climbMedExtend.Set(true);
    climbFullExtend.Set(true);
}
        
void Climber::testBarTraversalFromRaised() {
    //SetState(DIAGONAL_ARM_RETRACT);
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);
    gearboxMaster.Set(ControlMode::PercentOutput, 
        std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
        ClimbConstants::motorRetractedPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));
    //won't do this yet, may add wait function
    //climbFullExtend.Set(true);
    //climbMedExtend.Set(true);
}