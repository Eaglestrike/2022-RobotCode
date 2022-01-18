#include <Odometry.h>

//Constructor
Odometry::Odometry(){}


//BUG FIX
//Updates the Odometry Robot location
//https://www.first1684.com/uploads/2/0/1/6/20161347/chimiswerve_whitepaper__2_.pdf
//https://www.chiefdelphi.com/t/calculating-odometry-of-a-swerve-drive/160043/6
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
    //Potential Typo
    double C_RR = cos(BR_A * PI/180) * BR_V;

    double A = (A_RR + A_RL) / 2;
    double B = (B_FL + B_FR) / 2;
    double C = (C_FR + C_RR) / 2;
    double D = (D_FL + D_RL) / 2;

    double ROT1 = (B-A)/m_L;
    double ROT2 = (C-D)/m_W;
    double ROT = (ROT1 + ROT2) / 2;

    m_FWD = ((ROT * (m_L/2) + A) + (-ROT * (m_L/2) + B)) / 2;
    m_STR = ((ROT * (m_W/2) + C) + (-ROT * (m_W/2) + D)) / 2;

    m_FWD_new = m_FWD * cos(theta * PI/180) + m_STR * sin(theta * PI/180);
    m_STR_new = m_STR * cos(theta * PI/180) - m_FWD * sin(theta * PI/180);
    frc::SmartDashboard::PutNumber("m_FWD", m_FWD_new);
    frc::SmartDashboard::PutNumber("m_STR", m_STR_new);
    
    //From Velocity to Position
    m_prevtimeStep = m_timeStep;
    m_timeStep += timeInterval;
    //frc::SmartDashboard::PutNumber("Time", m_timeStep);
    m_x = m_x + m_FWD_new * (m_timeStep - m_prevtimeStep) * 58.04;
    m_y = m_y + m_STR_new * (m_timeStep - m_prevtimeStep) * 58.04;
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


//Reset Odometry
void
Odometry::Reset(){
    m_x = 0;
    m_y = 0;
    m_timeStep = 0;
}