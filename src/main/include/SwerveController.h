#pragma once

#include <frc/controller/PIDController.h>
#include <vector>
#include <frc/smartdashboard/SmartDashboard.h>

//IN PROGRESS
class SwerveController{

    public:
        SwerveController();
        double calculateForward(double goal);
        double calculateStrafe(double goal);
        double calculateRotation(double goal);
        void setPID();
        void setRotPID();
        void updatePosition(double forward, double strafe, double rotation);
        

    private:
        double m_forward = 0;
        double m_strafe = 0;
        double m_rotation = 0;

        frc2::PIDController m_forwardcontroller{0.37, 0, 0};
        frc2::PIDController m_strafecontroller{0.37, 0, 0};
        frc2::PIDController m_rotationcontroller{0.0001, 0, 0};

};