/* History:
  January 2022, Andrew Kim, Created.
  
  Purpose: 
  Intake File

  For the Intake subsystem
  Enable pneumatics and run intake motor
*/


#include "Intake.h"

//Constructor
Intake::Intake(){
    m_intakeMotor.SetNeutralMode(NeutralMode::Coast);
    m_intakeMotor.SetSafetyEnabled(false);
    m_intakeMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 20);
    pneumatics.SetPulseDuration(duration);
    pneumatics.Set(false);
}


//Periodic Function to execute intake states
void
Intake::Periodic(){
    switch(m_state){
        case State::IDLE:
            Stop();
            break;
        case State::RUN:
            Run();
            break;
        case State::INTAKE:
            Run();
            Deploy();
            break;
        // case State::TOGGLE:
        //     toggle();
        //     break;
        case State::ZERO:
            Deploy();
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


//Deploy the Intake (extend it)
void
Intake::Deploy(){
    pneumatics.Set(true);
}


//Run the Intake
void
Intake::Run(){
    m_intakeMotor.Set(ControlMode::PercentOutput, 0.39);
}


//Stop the Intake
void 
Intake::Stop(){
    m_intakeMotor.Set(ControlMode::PercentOutput, 0.0);
}

//Retract the Intake
void
Intake::Retract(){
    pneumatics.Set(false);
}


//Unjam the Intake (run it in reverse)
void
Intake::Unjam(){
    m_intakeMotor.Set(ControlMode::PercentOutput, -0.39);
}


//Set the state of the intake
void
Intake::setState(State newstate){
    m_state = newstate;
}

//toggle intake extend/retract
void
Intake::toggle(){
    pneumatics.Toggle();
}
