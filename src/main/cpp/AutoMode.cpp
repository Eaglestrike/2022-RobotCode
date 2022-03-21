#include <AutoMode.h>

AutoMode::AutoMode(){
    index = 0;
}


void
AutoMode::ResetAuto(){
    index = 0;
}

void
AutoMode::SetMode(int autoMode){
    if(autoMode == 1){
        actions = actions1;
        times = times1;
        waypointIndex = waypointIndex1;
    }
    if(autoMode == 2){
        actions = actions2;
        times = times2;
        waypointIndex = waypointIndex2;
    }
    if(autoMode == 3){
        actions = actions3;
        times = times3;
        waypointIndex = waypointIndex3;
    }
    if(autoMode == 5){
        actions = actions5;
        times = times5;
        waypointIndex = waypointIndex5;
    }
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