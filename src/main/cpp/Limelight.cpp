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

// get corners
std::vector<double> Limelight::getCorners() {
    std::vector<double> corners = network_table->GetEntry("tcornxy").GetDoubleArray(std::vector<double>());

    return corners;
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
    double distance = getDist() + 0.686;
    double robotGoalAngle_ = -(turretAngle + getAdjustedX()); 
    double angleToGoal = navx + robotGoalAngle_;
    double y = -distance * cos(angleToGoal * M_PI / 180);
    double x = distance * sin(angleToGoal * M_PI / 180);

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

// Pixels to Angles
// returns the angle from the camera to the pixel
std::pair<double, double> pixelsToAngle(double px, double py) {
    // From here: https://docs.limelightvision.io/en/latest/theory.html#from-pixels-to-angles
    const double H_FOV = 54;
    const double V_FOV = 41;
    const double IMG_WIDTH = 320;
    const double IMG_HEIGHT = 240;

    // normalized x and y
    double nx = (1/(IMG_WIDTH/2)) * (px - 159.5);
    double ny = (1/(IMG_HEIGHT/2)) * (119.5 - py);

    // view plane width/height
    double vpw = 2.0 * tan(H_FOV/2);
    double vph = 2.0 * tan(V_FOV/2);

    // view plane coordinates
    double x = vpw/2 * nx;
    double y = vph/2 * ny;

    // calc angles
    double ax = atan2(1, x); 
    double ay = atan2(1, y);

    std::pair ans(ax, ay);
    return ans;
}

// angles to actual x, y, z of point
std::tuple<double, double, double> angleToCoords(double ax, double ay, double targetHeight) {
    // From: https://www.chiefdelphi.com/t/calculating-distance-to-vision-target/387183/6
    // and https://www.chiefdelphi.com/t/what-does-limelight-skew-actually-measure/381167/7 

    // ax and ay are the angles from the camera to the point 
    // make sure x rotation is around y-axis
    // make sure y rotation is around x-axis
    double x = tan(ax);
    double y = tan(ay);
    double z = 1;

    double length = sqrt(x*x + y*y + z*z);

    // get normalized x, y, and z values
    x = x/length;
    y = y/length;
    z = z/length;

    // apply transformations to 3d vector (compensate for pitch) -> rotate down by camera pitch
    // multiply [x, y, z] vector by rotation matrix around x-axis
    double theta = -GeneralConstants::cameraPitch * M_PI / 180; // for testing
    x = x*1 + y*0 + z*0; // technically not necessary, but just for understandability
    y = x*0 + y*cos(theta) + z*(-sin(theta));
    z = x*0 + y*sin(theta) + z*cos(theta);

    // denormalize coordinates via known height
    double scale = (targetHeight - GeneralConstants::cameraHeight) / y;    

    x *= scale;
    y *= scale;
    z *= scale;

    x += GeneralConstants::cameraHeight;
    y += GeneralConstants::cameraHeight;
    z += GeneralConstants::cameraHeight;
    
    return std::tuple(x, y, z);
}