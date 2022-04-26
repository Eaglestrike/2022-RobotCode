/* History:
  January 2022, Andrew Kim, Created.
  
  Purpose: 
  Odometry File

  This is forward kinematics for swerve drive
  Gets the wheel drive values to calculate distance
*/


#include <Odometry.h>

//Constructor
Odometry::Odometry(){}


//Updates the Odometry Robot location
void
Odometry::updateOdometry(double BR_A, double BR_V,
            double BL_A, double BL_V,
            double FR_A, double FR_V,
            double FL_A, double FL_V,
            double theta){

    double B_FL = sin(FL_A * PI/180) * FL_V;
    double A_RL = sin(BL_A * PI/180) * BL_V;
    double A_RR = sin(BR_A * PI/180) * BR_V;
    double D_FL = cos(FL_A * PI/180) * FL_V;
    double C_FR = cos(FR_A * PI/180) * FR_V;
    double B_FR = sin(FR_A * PI/180) * FR_V;
    double D_RL = cos(BL_A * PI/180) * BL_V;
    double C_RR = cos(BR_A * PI/180) * BR_V;

    double A = (A_RR + A_RL) / 2;
    double B = (B_FL + B_FR) / 2;
    double C = (C_FR + C_RR) / 2;
    double D = (D_FL + D_RL) / 2;

    double ROT1 = (B-A)/m_L;
    double ROT2 = (C-D)/m_W;
    double ROT = (ROT1 + ROT2) / 2;

    m_STR = (A + B) / 2;
    m_FWD = -(C + D) / 2;

    m_FWD_new = m_FWD * cos(theta * PI/180) + m_STR * sin(theta * PI/180);
    m_STR_new = m_STR * cos(theta * PI/180) - m_FWD * sin(theta * PI/180);
    
    // double STR, FWD, STR1, STR2, FWD1, FWD2;
    // STR1 = ROT * (m_L / 2.0) + A;
	// STR2 = -ROT * (m_L / 2.0) + B;
	// FWD1 = ROT * (m_W / 2.0) + C;
	// FWD2 = -ROT * (m_W / 2.0) + D;

	// STR = (STR1 + STR2) / 2.0;
	// FWD = (FWD1 + FWD2) / 2.0;
    //From Velocity to Position
    m_prevtimeStep = m_timeStep;
    m_timeStep += timeInterval;
    m_x = m_x + m_STR_new * (m_timeStep - m_prevtimeStep);
    m_y = m_y + m_FWD_new * (m_timeStep - m_prevtimeStep);
}


//Return X position
double
Odometry::getX(){
    return m_x;
}


//Return Y position
double
Odometry::getY(){
    return m_y;
}


double
Odometry::getYSpeed(){
    return m_FWD_new;
}


double
Odometry::getXSpeed(){
    return m_STR_new;
}


//Reset Odometry
void
Odometry::Reset(){
    m_x = 0;
    m_y = 0;
    m_timeStep = 0;
}