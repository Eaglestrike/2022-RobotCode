#include "Intake.h"

//Constructor
Intake::Intake(){
    intakeMotor.SetNeutralMode(NeutralMode::Coast);
    intakeMotor.SetSafetyEnabled(false);

    colorMatcher.AddColorMatch(red); //red
    colorMatcher.AddColorMatch(blue); //blue
}


//Periodic Function to execute intake states
void
Intake::Periodic(){
    switch(state){
        case State::IDLE:
            Retract();
            break;
        case State::RUN:
            Run();
            break;
        case State::UNJAM:
            Unjam();
            break;
        case State::CLIMB:
            Retract();
            break;
        default:
            break;
    }
}


//Deploy the Intake
void
Intake::Deploy(){
    //pneumatics.Set(true);
}



//Run the Intake
void
Intake::Run(){
    double confidence = 0.0; //it complains if I don't do this
    if ((GeneralConstants::matchColor == "BLUE" && colorMatcher.MatchClosestColor(colorSensor.GetColor(), confidence) == red)
        || (GeneralConstants::matchColor == "RED" && colorMatcher.MatchClosestColor(colorSensor.GetColor(), confidence) == blue)) {
            intakeMotor.Set(ControlMode::PercentOutput, 0.45); //or could just stop?
        }
    intakeMotor.Set(ControlMode::PercentOutput, -0.45);
}


//Retract the Intake
void
Intake::Retract(){
    //might not retract the intake
    //pneumatics.Set(false);
    intakeMotor.Set(ControlMode::PercentOutput, 0.0);
}


//Unjam the Intake
//positive output is reverse
void
Intake::Unjam(){
    intakeMotor.Set(ControlMode::PercentOutput, 0.25);
}


//Set the state of the intake
void
Intake::setState(State newstate){
    state = newstate;
}


void
Intake::calibrate(){

}