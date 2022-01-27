#include "Limelight.h"
#include <math.h>

Limelight::Limelight() {
    network_table = nt::NetworkTableInstance::GetDefault().GetTable(table_name);
    network_table->PutNumber("pipeline", PIPELINE);
}

double Limelight::CalculateDistance() {
    return (GOAL_HEIGHT - CAMERA_HEIGHT) / tan(CAMERA_PITCH+GetXOff());
}

double Limelight::GetXOff() {
    return network_table->GetNumber("tx", 10000.0);
}

double Limelight::GetYOff() {
    return network_table->GetNumber("ty", 10000.0);
}

bool Limelight::SeesTarget(){
    double tv = network_table->GetNumber("tv", 0.0);
    if(tv == 0.0){
        //The target is not visible
        return false;
    }
    return true;
}

void Limelight::SetLEDMode(std::string mode) {
    if (mode == "OFF") network_table->PutNumber("ledMode", 1);
    if (mode == "ON") network_table->PutNumber("ledMode", 3);
}