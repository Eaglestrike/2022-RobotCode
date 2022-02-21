#include <Channel.h>
#include <iostream>

//Constructor is empty for now
Channel::Channel(){
    m_channel.SetNeutralMode(NeutralMode::Brake);
    colorMatcher.AddColorMatch(red); //red
    colorMatcher.AddColorMatch(blue); //blue
}


//Periodic Function
void
Channel::Periodic(){
    switch(state){
        case State::IDLE:
            Stop();
            break;
        case State::RUN:
            Run();
            break;
        default:
            break;
    }
}


//Run function
void
Channel::Run(){
    m_channel.Set(ControlMode::PercentOutput, 0.35);
}


//Stop all channel movement
void
Channel::Stop(){
    m_channel.Set(ControlMode::PercentOutput, 0.0);
}


//set State of the Channel
void
Channel::setState(State newState){
    state = newState;
}