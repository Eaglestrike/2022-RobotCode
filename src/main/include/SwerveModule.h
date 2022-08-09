#include <ctre/Phoenix.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <Constants.h>
#include <frc/MathUtil.h>

class SwerveModule {

    public:
        SwerveModule(int angMotorPort, int speedMotorPort, int cancoderPort, bool inverted, double offset);

        double getVelocity();
        double getYaw();
        frc::SwerveModuleState getState();
        frc::SwerveModuleState getOptState(frc::SwerveModuleState state);

        void setAngMotorVoltage(double voltage);
        void setSpeedMotor(double power);

    private:
        double offset_;

        int angMotorPort_;
        int speedMotorPort_;
        int canCoderPort_;

        WPI_TalonFX angleMotor_;
        WPI_TalonFX speedMotor_;
        WPI_CANCoder canCoder_;

        //converts raw talon velocity to meters per second
        //raw velocity units are ticks per 100ms
        units::meters_per_second_t talonVelToMps(double vel); 
};