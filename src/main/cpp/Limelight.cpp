/* History:
  January 2022, Andrew Kim, Created.
  
  Purpose: 
  For the limelight

  Can get the x, y offsetts.
  Calculate distance has been fixed and units are in meters
*/


#include "Limelight.h"


Limelight::Limelight(){
    network_table = nt::NetworkTableInstance::GetDefault().GetTable(table_name);
    network_table->PutNumber("pipeline", PIPELINE);
}


// Return the distance. 
// This will have to change if the limelight location is changed
double
Limelight::calculateDistance(){
    // return (GeneralConstants::goalHeight - GeneralConstants::cameraHeight)
    //     / tan(GeneralConstants::cameraPitch + getYOff()*3.14159/180.0);
    return 2.1 / tan((40+getYOff())*3.1415/180);
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
    double tv = network_table->GetNumber("tv", 0.0);
    if(tv == 0.0){
        return false;
    } else {
        return true;
    }
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