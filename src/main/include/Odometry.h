#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>

#define PI 3.14159265

class Odometry{

    public:
        Odometry();
        void updateOdometry(double BR_A, double BR_V,
            double BL_A, double BL_V,
            double FR_A, double FR_V,
            double FL_A, double FL_V,
            double theta);
        double getX();
        double getY();
        double getXSpeed();
        double getYSpeed();
        void Reset();


    private:
        const double timeInterval = 0.02;
        double m_timeStep = 0;
        double m_prevtimeStep = 0;

        double m_x = 0;
        double m_y = 0;
        double m_L = 0.394;
        double m_W = 0.394;

        double m_FWD;
        double m_STR;
        double m_FWD_new;
        double m_STR_new;
};