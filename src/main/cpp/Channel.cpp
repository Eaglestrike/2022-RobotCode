#include <Channel.h>

//Constructor is empty for now
Channel::Channel(){}


//Periodic Function
void
Channel::Periodic(){
    switch(state){
        case State::IDLE:
            break;
        case State::RUN:
            break;
        default:
            break;
    }
}


//Run function
void
Channel::Run(){
    m_channel.Set(ControlMode::PercentOutput, 0.25);
    if(!photogate1.Get()){
        Stop();
    }
}


//Stop all channel movement
void
Channel::Stop(){
    m_channel.Set(ControlMode::PercentOutput, 0);
}


//set State of the Channel
void
Channel::setState(State newState){
    state = newState;
}