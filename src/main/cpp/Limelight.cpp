//actually I made the limelight class
#include "Limelight.h"


Limelight::Limelight(){
    network_table = nt::NetworkTableInstance::GetDefault().GetTable(table_name);
    network_table->PutNumber("pipeline", PIPELINE);
}

//moved distance calculator to shooter calc

//Return the X offset to the target
double
Limelight::getXOff(){
    return network_table->GetNumber("tx", 10000.0);
}


//Return the y offset to the target
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

void Limelight::adjustAngles(double& ax, double& ay)
{
    double flippedY = (90 - ay) * M_PI / 180;
    double flippedX = (90 - ax) * M_PI / 180;

    //Spherical to cartesian
    double x = sin(flippedY) * cos(flippedX);
    double y = sin(flippedY) * sin(flippedX);
    double z = cos(flippedY);
    
    //TODO: confirm?
    double rotAng = CAM_ANGLE * M_PI / 180;

    //rotate
    double rotatedY = y * cos(rotAng) - z * sin(rotAng);
    double rotatedZ = y * sin(rotAng) + z * cos(rotAng);

    //get new angles
    double horizontal = sqrt(x * x + rotatedY * rotatedY);
    double nay, nax;
    if(rotatedZ != 0 || horizontal != 0) {
        nay = atan2(rotatedZ, horizontal) * 180 / M_PI;
    }
    else nay = 0;

    if(rotatedY != 0 || x != 0) {
        nax = (atan2(x, rotatedY) * 180 / M_PI);
    }
    else nax = 0;

    ax = nax;
    ay = nay;
}

double Limelight::getAdjustedX()
{
    double x = getXOff();
    double y = getYOff();
    adjustAngles(x, y);

    frc::SmartDashboard::PutNumber("LAX", x);
    frc::SmartDashboard::PutNumber("LAY", y);

    return x;
}

//coordinates: gonna assume angle is zero when robot facing directly away
frc::Pose2d Limelight::getPose(double navx, double turretAngle) {
    double distance = getDist();// + 0.6096; //todo: add back in later
    double robotGoalAngle_ = -(turretAngle + getAdjustedX()); 
    double angleToGoal = navx + robotGoalAngle_;
    double x = -distance * cos(angleToGoal * M_PI / 180);
    double y = -distance * sin(angleToGoal * M_PI / 180);

   // frc::SmartDashboard::PutNumber("distance", distance);
    // frc::SmartDashboard::PutNumber("robotGoalAngle", robotGoalAngle_);
    // frc::SmartDashboard::PutNumber("angleToGoal", angleToGoal);
    // frc::SmartDashboard::PutNumber("Pose x", x);
    // frc::SmartDashboard::PutNumber("Pose y", y);

    return frc::Pose2d{units::meter_t{x}, units::meter_t{y}, frc::Rotation2d{units::degree_t{navx}}};
}

double Limelight::getDist() {
    double x = getXOff();
    double y = getYOff();
    adjustAngles(x, y);
    return (HUB_HEIGHT - CAM_HEIGHT) / tan(y * M_PI / 180);
}



//Set the LED mode
void
Limelight::setLEDMode(std::string mode){
    if(mode == "OFF"){
        network_table->PutNumber("ledMode", 1.0);
    }
    if(mode == "BLINK"){
        network_table->PutNumber("ledMode", 2.0);
    }
    if(mode == "ON"){
        network_table->PutNumber("ledMode", 3.0);
    }
}