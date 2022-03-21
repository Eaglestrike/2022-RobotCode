#include "Climber.h"

//Constructor
Climber::Climber(){
    // climbStage1.Set(true);
    // climbStage2.Set(true);
    gearboxMaster.SetSafetyEnabled(false);
    gearboxSlave.SetSafetyEnabled(false);

    gearboxMaster.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 100);
    gearboxSlave.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 100);

    gearboxMaster.SetNeutralMode(NeutralMode::Brake);
    gearboxSlave.SetNeutralMode(NeutralMode::Brake);

    climbStage2.SetPulseDuration(pulse);
    climbStage1.SetPulseDuration(pulse);

    //diskBrake.Set(false);
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
    //One will have to be true
    climbStage1.Set(false);
    climbStage2.Set(true);
    gearboxMaster.SetSelectedSensorPosition(0);
    gearboxSlave.SetSelectedSensorPosition(0);
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


void
Climber::armExtension(double input){
    gearboxMaster.Set(ControlMode::PercentOutput, 0.5* input);
    gearboxSlave.Set(ControlMode::PercentOutput, 0.5*input);
    frc::SmartDashboard::PutNumber("extension", gearboxMaster.GetSelectedSensorPosition());
}


void
Climber::ExtendfirstStage(){
    //climbStage1.Set(true);
    climbStage1.Toggle();
}


void
Climber::ExtendsecondStage(){
    climbStage2.Toggle();
}


void
Climber::RetractfirstStage(){
    climbStage1.Set(false);
}


void
Climber::RetractsecondStage(){
    climbStage2.Set(false);
}

void
Climber::disableBrake(){
    diskBrake.Set(true);
}

void
Climber::whenDisabled(){
    climbStage1.Set(true);
    climbStage2.Set(false);
}