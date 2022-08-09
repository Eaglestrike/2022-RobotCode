#include "Swerve.h"
#include <iostream>
#include <iostream>
#include <fstream>
#include <sstream>

//wrapper class for wpilib swerve

Swerve::Swerve(AHRS * nx, DataLogger * logger) : m_navx{nx}, m_logger{logger} {

    angPID_.EnableContinuousInput(-180, 180);
    angPID_.SetIntegratorRange(-0.5, 0.5);

}

//make new odometry object specifying robot's initial angle and position on the field
void Swerve::initializeOdometry(frc::Rotation2d gyroAngle, frc::Pose2d initPose) {
  delete odometry_;
  odometry_ = new frc::SwerveDriveOdometry<4>(m_kinematics, gyroAngle, initPose);
}

//get the robot's speed (as a 2d vector)
frc::ChassisSpeeds Swerve::getSpeeds() {
  frc::ChassisSpeeds speeds = m_kinematics.ToChassisSpeeds(getRealModuleStates());
  frc::Translation2d t{units::meter_t{speeds.vx.value()}, units::meter_t{speeds.vy.value()}}; //sus units... if we hate it i can write this myself
  t.RotateBy(frc::Rotation2d{units::degree_t{m_navx->GetYaw()}});
  speeds.vx = units::meters_per_second_t{t.X().value()};
  speeds.vy = units::meters_per_second_t{t.Y().value()};
  return speeds;
}

//returns an object that contains the speed and angle of each swerve module
wpi::array<frc::SwerveModuleState, 4> Swerve::getRealModuleStates() {
  wpi::array<frc::SwerveModuleState, 4> moduleStates = {
    flModule_.getState(), frModule_.getState(), blModule_.getState(), brModule_.getState() };
  return moduleStates;
}

//called every 20ms, updates odometry and wheel speeds/angles based on joystick input
/**
 * @param dx the x joystick input, specifies speed in the x direction
 * @param dy the y joystick input, specifies speed in the y direction
 * @param dtheta the right joystick left/right input, specifies angular speed
 * @param navx_yaw the gyro reading of the robot's position relative to starting
 * TODO: adjust initial navx_yaw depending on robot init angle so it'll be field oriented during comp
**/
void Swerve::Periodic(units::meters_per_second_t dx, units::meters_per_second_t dy, units::radians_per_second_t dtheta, 
units::degree_t navx_yaw) {

  //in mps, not joystck units
  if (abs(dy.value()) < 0.5) dy = units::meters_per_second_t{0};
  if (abs(dx.value()) < 0.5) dx = units::meters_per_second_t{0};

  frc::SmartDashboard::PutNumber("Dx", dx.value());
  frc::SmartDashboard::PutNumber("Dy", dy.value());
  frc::SmartDashboard::PutNumber("Dtheta", dtheta.value());
  frc::SmartDashboard::PutNumber("navx yaw", navx_yaw.value());

  //converts field-relative joystick input to robot-relative speeds
  speeds_ = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    dx, dy, dtheta, frc::Rotation2d(navx_yaw));

  //convert robot speeds into swerve module states (speed & angle of each module)
  auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds_);
  
  //update odometry
  odometry_->Update(navx_yaw, getRealModuleStates());

  //optimize module states so they can have most direct path to goal
  auto fl_opt = flModule_.getOptState(fl);
  auto fr_opt = frModule_.getOptState(fr);
  auto bl_opt = blModule_.getOptState(bl);
  auto br_opt = brModule_.getOptState(br);


 //command each swerve motor
 //TODO: fix weird thing with speed if drivers complain or we have time
  flModule_.setAngMotorVoltage( std::clamp(
      angPID_.Calculate(flModule_.getYaw(), fl_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );

  flModule_.setSpeedMotor( 0.2*std::clamp(fl_opt.speed.value(), -1.0, 1.0) );

  frModule_.setAngMotorVoltage( std::clamp(
      angPID_.Calculate(frModule_.getYaw(), fr_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );

  frModule_.setSpeedMotor( 0.2*std::clamp(fr_opt.speed.value(), -1.0, 1.0) );

  blModule_.setAngMotorVoltage( std::clamp(
      angPID_.Calculate(blModule_.getYaw(), bl_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );

  blModule_.setSpeedMotor( 0.2*std::clamp(bl_opt.speed.value(), -1.0, 1.0) );

  brModule_.setAngMotorVoltage( std::clamp(
      angPID_.Calculate(brModule_.getYaw(), br_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );

  brModule_.setSpeedMotor( 0.2*std::clamp(br_opt.speed.value(), -1.0, 1.0) );

}

//For tuning the PID of the motors that rotate the swerve modules
void Swerve::SetPID() {

  double P = frc::SmartDashboard::GetNumber("Swerve P", P);
  double I = frc::SmartDashboard::GetNumber("Swerve I", I);
  double D = frc::SmartDashboard::GetNumber("Swerve D", D);

  frc::SmartDashboard::PutNumber("Swerve P", P);
  frc::SmartDashboard::PutNumber("Swerve I", I);
  frc::SmartDashboard::PutNumber("Swerve D", D);


  speedPID_.SetP(P);
  speedPID_.SetI(I);
  speedPID_.SetD(D);
}
