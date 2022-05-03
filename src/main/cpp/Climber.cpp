/* History:
  January 2022, Andrew Kim, Created.
  
  Purpose: 
  Climber File

  For the clmber subsystem
  Manual climb for moving the double pnuematic and gearbox motors
*/


#include "Climber.h"

//Constructor
//set motor and pneumatic configurations
Climber::Climber(){
    // climbStage1.Set(true);
    // climbStage2.Set(true);
    gearboxMaster.SetSafetyEnabled(false); 
    gearboxSlave.SetSafetyEnabled(false);

    //don't send over unimportant information very often 
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
//sets pneumatics to starting robot position, resets motor positions
void
Climber::Initialize(){
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


// Turn motors with direction & percent output proportional to joystick input
// moves arms up and down
void
Climber::armExtension(double input){
    gearboxMaster.Set(ControlMode::PercentOutput, 0.5*input);
    gearboxSlave.Set(ControlMode::PercentOutput, 0.5*input);
    // frc::SmartDashboard::PutNumber("extension", gearboxMaster.GetSelectedSensorPosition());
}


// Toggle first stage pneumatic
void
Climber::ExtendfirstStage(){
    //climbStage1.Set(true);
    climbStage1.Toggle();
}


// Toggle second stage pneumatic
void
Climber::ExtendsecondStage(){
    climbStage2.Toggle();
}


//unused
void
Climber::RetractfirstStage(){
    climbStage1.Set(false);
}

//unused
void
Climber::RetractsecondStage(){
    climbStage2.Set(false);
}

//unused
void
Climber::disableBrake(){
    diskBrake.Set(true);
}

//unused
void
Climber::whenDisabled(){
    climbStage1.Set(true);
    climbStage2.Set(false);
}