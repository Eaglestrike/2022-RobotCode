#pragma once

#include <frc/controller/PIDController.h>
#include <vector>

class SwerveController{

    public:
        SwerveController();
        double calculateForward(double goal);
        double calculateStrafe(double goal);
        double calculateRotation(double goal);
        void setForwardPID(double Kp, double Ki, double Kd);
        void setStrafePID(double Kp, double Ki, double Kd);
        void setRotationPID(double Kp, double Ki, double Kd);
        void updatePosition(double forward, double strafe, double rotation);
        

    private:
        double m_forward = 0;
        double m_strafe = 0;
        double m_rotation = 0;

        frc2::PIDController m_forwardcontroller{0, 0, 0};
        frc2::PIDController m_strafecontroller{0, 0, 0};
        frc2::PIDController m_rotationcontroller{0, 0, 0};

};