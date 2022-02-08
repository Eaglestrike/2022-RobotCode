#include "Limelight.h"


Limelight::Limelight(){
    network_table = nt::NetworkTableInstance::GetDefault().GetTable(table_name);
    network_table->PutNumber("pipeline", PIPELINE);
}


//Return the distance
double
Limelight::calculateDistance(){
    return (GeneralConstants::goalHeight - GeneralConstants::cameraHeight)
        / tan(GeneralConstants::cameraPitch + getYOff());
}


//Return the X offset
double
Limelight::getXOff(){
    return network_table->GetNumber("tx", 10000.0);
}


//Return the y offset
double
Limelight::getYOff(){
    return network_table->GetNumber("ty", 10000.0);
}


//Check if the limeight sees the target
bool
Limelight::targetAquired(){
    double tv = network_table->GetBoolean("tv", false);
    return tv;
}


//Set the LED mode
void
Limelight::setLEDMode(std::string mode){
    if(mode == "OFF"){
        network_table->PutNumber("ledMode", 1);
    }
    if(mode == "BLINK"){
        network_table->PutNumber("ledMode", 2);
    }
    if(mode == "ON"){
        network_table->PutNumber("ledMode", 3);
    }
}