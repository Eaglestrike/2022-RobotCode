#include "SwerveModule.h"

SwerveModule::SwerveModule(int angMotorPort, int speedMotorPort, int canCoderPort, bool inverted, double offset) : angMotorPort_{angMotorPort}, speedMotorPort_{speedMotorPort},
canCoderPort_{canCoderPort}, angleMotor_{angMotorPort, "Drivebase"}, speedMotor_{speedMotorPort, "Drivebase"}, canCoder_{canCoderPort, "Drivebase"}, offset_{offset} {
    
    speedMotor_.SetInverted(inverted);

    angleMotor_.SetSelectedSensorPosition(0);
    speedMotor_.SetSelectedSensorPosition(0);

    angleMotor_.SetNeutralMode(NeutralMode::Brake);
    speedMotor_.SetNeutralMode(NeutralMode::Brake);   
}

double SwerveModule::getVelocity() {
    return speedMotor_.GetSelectedSensorVelocity();
}

double SwerveModule::getYaw() {
    return canCoder_.GetAbsolutePosition();
}

units::meters_per_second_t SwerveModule::talonVelToMps(double vel) {
    double wheel_radius = 0.05; //in meters
    double meters_per_rev = wheel_radius*2*M_PI; //wheel circumberence
    double ticks_per_rev = 12650;
    return units::meters_per_second_t{vel / 0.1 * (meters_per_rev / ticks_per_rev)};
}


//TODO: check input modulus of Rotation2d
frc::SwerveModuleState SwerveModule::getState() {
    frc::SwerveModuleState state; //TODO: can this be made inline?
    state.speed = talonVelToMps(speedMotor_.GetSelectedSensorVelocity());
    state.angle = frc::Rotation2d{units::angle::degree_t{angleMotor_.GetSelectedSensorVelocity() + DriveConstants::FLOFF}};
    return state;
}

frc::SwerveModuleState SwerveModule::getOptState(frc::SwerveModuleState state) {
    double yaw = frc::InputModulus(canCoder_.GetAbsolutePosition() + offset_, -180.0, 180.0);
    frc::SwerveModuleState opt_state = frc::SwerveModuleState::Optimize(state, units::degree_t(yaw)); 
    return opt_state;
}

void SwerveModule::setAngMotorVoltage(double voltage) {
    angleMotor_.SetVoltage(units::volt_t{voltage});
}

void SwerveModule::setSpeedMotor(double power) {
    speedMotor_.Set(ControlMode::PercentOutput, power);
}