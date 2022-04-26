/* History:
  January 2022, Andrew Kim, Created.
  
  Purpose: 
  For the hardware between the intake and shooter

  *** THIS IS A DEAD CLASS !!! ***
  Hardware is incorporated within the Intake
*/


#include <Channel.h>
#include <iostream>

//Constructor
Channel::Channel(){
    m_channel.SetNeutralMode(NeutralMode::Brake);
    m_channel.SetSafetyEnabled(false);
    m_channel.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 100, 100);
}


//Periodic Function
void
Channel::Periodic(){
    switch(m_state){
        case State::IDLE:
            Stop();
            break;
        case State::RUN:
            Run();
            break;
        case State::Badidea:
            Reverse();
            break;
        default:
            break;
    }
}


//Run function
void
Channel::Run(){
    m_channel.Set(ControlMode::PercentOutput, 0.40);
}


//Stop all channel movement
void
Channel::Stop(){
    m_channel.Set(ControlMode::PercentOutput, 0.0);
}


//set State of the Channel
void
Channel::setState(State newState){
    m_state = newState;
}


void
Channel::DisableMotor(){
    m_channel.Disable();
}


void
Channel::Reverse(){
    m_channel.Set(ControlMode::PercentOutput, -0.4);
}