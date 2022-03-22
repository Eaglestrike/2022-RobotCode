#pragma once

#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>

class Odometry {
   public:
    Odometry();
    void updateOdometry(double BR_A, double BR_V, double BL_A, double BL_V,
                        double FR_A, double FR_V, double FL_A, double FL_V,
                        double theta, double dt);
    double getX();
    double getY();
    double getXSpeed();
    double getYSpeed();
    void Reset();

   private:
    const double timeInterval = GeneralConstants::timeStep;
    double m_timeStep = 0;
    double m_prevtimeStep = 0;

    double m_x = 0;
    double m_y = 0;

    double m_xdot = 0;
    double m_ydot = 0;

    double init = false;
    double prev_bl{0.0}, prev_br{0.0}, prev_fl{0.0}, prev_fr{0.0};
};
