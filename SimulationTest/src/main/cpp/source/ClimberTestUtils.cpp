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

}

void Climber::extendArmUntilStopped(bool inverted) {
    if (inverted) gearboxMaster.Set(ControlMode::PercentOutput, -0.25);
    else gearboxMaster.Set(ControlMode::PercentOutput, 0.25);
    std::cout << "motor positon: " << gearboxMaster.GetSelectedSensorPosition() << "\n";
    //std::cout << "current: " << gearboxMaster.GetStatorCurrent() << "\n";
    if (gearboxMaster.GetStatorCurrent() > 200) std::cout << "high current, probably wrong direction\n";
}

void Climber::retractArm() {
    while (gearboxMaster.GetSelectedSensorPosition() > ClimbConstants::motorRetractedPose) {
        gearboxMaster.Set(ControlMode::PercentOutput, -0.25);
    }
}

void Climber::setArmLowered() {
    climbMedExtend.Set(true);
    climbFullExtend.Set(true);
}

void Climber::setArmRaised() {
    climbMedExtend.Set(false);
    climbFullExtend.Set(true);
}

void Climber::setArmVertical() {
    climbMedExtend.Set(false);
    climbFullExtend.Set(false);
}





void Climber::testRaiseVerticalArm() {
   // SetState(VERTICAL_ARM_EXTEND);
     
    gearboxMaster.SetNeutralMode(NeutralMode::Coast);
    gearboxMaster.Set(ControlMode::PercentOutput, 
        std::clamp(motorPIDController.Calculate(gearboxMaster.GetSelectedSensorPosition(), 
        ClimbConstants::motorExtendedPose), -ClimbConstants::motorMaxOutput, ClimbConstants::motorMaxOutput));
    
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
    SetState(DIAGONAL_ARM_EXTEND);
}
        
void Climber::testDiagonalArmRaise() {
    SetState(DIAGONAL_ARM_RAISE);
}
        
void Climber::testBarTraversalFromRaised() {
    SetState(DIAGONAL_ARM_RETRACT);
}