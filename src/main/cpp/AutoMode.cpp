#include <AutoMode.h>

AutoMode::AutoMode(){
    index = 0;
   
}


void
AutoMode::ResetAuto(){
    index = 0;
}


void
AutoMode::Periodic(double time){
    frc::SmartDashboard::PutNumber("index", index);
    if(index == times.size() -1 || time <= times[index]){
        return;
    } else {
        index ++;
    }
}


AutoMode::State 
AutoMode::getState(){
    return actions[index]; 
}

int
AutoMode::getWaypointIndex(){
    return waypointIndex[index];
}