#include <Odometry.h>


Odometry::Odometry(){}


void
Odometry::updateOdometry(double BR_A, double BR_V,
            double BL_A, double BL_V,
            double FR_A, double FR_V,
            double FL_A, double FL_V,
            double ROT, double theta){

    double B_FL = sin(FL_A)  * PI/180 * FL_V;
    double B_FR = sin(FR_A) * PI/180 * FR_V;
    double A_RL = sin(BL_A) * PI/180 * BL_V;
    double A_RR = sin(BR_A) * PI/180 * BR_V;
    double D_FL = cos(FL_A) * PI/180 * FL_V;
    double C_FR = cos(FR_A) * PI/180 * FR_V;
    double D_RL = cos(BL_A) * PI/180 * BL_V;
    //This one might be wrong
    double C_RR = cos(FR_A) * PI/180 * FR_V;

    double A = (A_RR + A_RL) / 2;
    double B = (B_FL + B_FR) / 2;
    double C = (C_FR + C_RR) / 2;
    double D = (D_FL + D_RL) / 2;

    m_FWD = ((ROT * (m_L/2) + A) + (-ROT * (m_L/2) + B)) / 2;
    m_STR = ((ROT * (m_W/2) + C) + (-ROT * (m_W/2) + D)) / 2;

    m_FWD_new = m_FWD * cos(theta) * PI/180 + m_STR * sin(theta) * PI/180;
    m_STR_new = m_STR * cos(theta) * PI/180 - m_FWD * sin(theta) * PI/180;
    frc::SmartDashboard::PutNumber("m_FWD", m_FWD_new);
    frc::SmartDashboard::PutNumber("m_STR", m_STR_new);
    //From Velocity to Position
    m_timeStep += timeInterval;
    frc::SmartDashboard::PutNumber("Time", m_timeStep);
    m_x = m_x + m_FWD_new * m_timeStep;
    m_y = m_y + m_STR_new * m_timeStep;
}


double
Odometry::getX(){
    return m_x;
}


double
Odometry::getY(){
    return m_y;
}


void
Odometry::Reset(){
    m_x = 0;
    m_y = 0;
}