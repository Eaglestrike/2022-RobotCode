#include "Swerve.h"
#include <iostream>
#include <iostream>
#include <fstream>
#include <sstream>

//wrapper class for wpilib swerve

Swerve::Swerve(AHRS * nx, DataLogger * logger) : m_navx{nx}, m_logger{logger} {

    m_fl_pid.EnableContinuousInput(-180, 180);
    m_fr_pid.EnableContinuousInput(-180, 180);
    m_rl_pid.EnableContinuousInput(-180, 180);
    m_rr_pid.EnableContinuousInput(-180, 180);

    m_fl_pid.SetIntegratorRange(-0.5, 0.5);
    m_fr_pid.SetIntegratorRange(-0.5, 0.5);
    m_rl_pid.SetIntegratorRange(-0.5, 0.5);
    m_rr_pid.SetIntegratorRange(-0.5, 0.5);

    m_fr_speedMotor.SetInverted(true);
    m_rl_speedMotor.SetInverted(true);

    m_fl_angleMotor.SetSelectedSensorPosition(0);
    m_fr_angleMotor.SetSelectedSensorPosition(0);
    m_rl_angleMotor.SetSelectedSensorPosition(0);
    m_rr_angleMotor.SetSelectedSensorPosition(0);

    m_fl_angleMotor.SetNeutralMode(NeutralMode::Brake);
    m_fr_angleMotor.SetNeutralMode(NeutralMode::Brake);
    m_rl_angleMotor.SetNeutralMode(NeutralMode::Brake);
    m_rr_angleMotor.SetNeutralMode(NeutralMode::Brake);

    m_fl_speedMotor.SetNeutralMode(NeutralMode::Brake);
    m_fr_speedMotor.SetNeutralMode(NeutralMode::Brake);
    m_rl_speedMotor.SetNeutralMode(NeutralMode::Brake);
    m_rr_speedMotor.SetNeutralMode(NeutralMode::Brake);


    m_fl_speedMotor.SetSelectedSensorPosition(0);
    m_fr_speedMotor.SetSelectedSensorPosition(0);
    m_rl_speedMotor.SetSelectedSensorPosition(0);
    m_rr_speedMotor.SetSelectedSensorPosition(0);

}

//make new odometry object specifying robot's initial angle and position on the field
void Swerve::initializeOdometry(frc::Rotation2d gyroAngle, frc::Pose2d initPose) {
  delete odometry;
  odometry = new frc::SwerveDriveOdometry<4>(m_kinematics, gyroAngle, initPose);
}

//get the robot's speed (as a 2d vector)
frc::ChassisSpeeds Swerve::getSpeeds() {
  return m_kinematics.ToChassisSpeeds(GetRealModuleStates());
}

//returns an object that contains the speed and angle of each swerve module
//TODO: should navx angle be added for odometry (i don't think so)?? and is it correct for getting speed either?
wpi::array<frc::SwerveModuleState, 4> Swerve::GetRealModuleStates() {
  wpi::array<frc::SwerveModuleState, 4> moduleStates(wpi::empty_array);

  moduleStates[0] = frc::SwerveModuleState{};
  moduleStates[0].speed = talonVelToMps(m_fl_speedMotor.GetSelectedSensorVelocity());
  moduleStates[0].angle = frc::Rotation2d{units::angle::degree_t{m_fl_canCoder.GetAbsolutePosition() + m_navx->GetYaw()}};

  moduleStates[1] = frc::SwerveModuleState{};
  moduleStates[1].speed = talonVelToMps(m_fr_speedMotor.GetSelectedSensorVelocity());
  moduleStates[1].angle = frc::Rotation2d{units::angle::degree_t{m_fr_canCoder.GetAbsolutePosition() + m_navx->GetYaw()}};
    
  moduleStates[2] = frc::SwerveModuleState{};
  moduleStates[2].speed = talonVelToMps(m_rl_speedMotor.GetSelectedSensorVelocity());
  moduleStates[2].angle = frc::Rotation2d{units::angle::degree_t{m_rl_canCoder.GetAbsolutePosition() + m_navx->GetYaw()}};

  moduleStates[3] = frc::SwerveModuleState{};
  moduleStates[3].speed = talonVelToMps(m_rr_speedMotor.GetSelectedSensorVelocity());
  moduleStates[3].angle = frc::Rotation2d{units::angle::degree_t{m_rr_canCoder.GetAbsolutePosition() + m_navx->GetYaw()}};

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

  // frc::SmartDashboard::PutNumber("fl pos: ", m_fl_speedMotor.GetSelectedSensorPosition());
  // frc::SmartDashboard::PutNumber("fr pos: ", m_fr_speedMotor.GetSelectedSensorPosition());
  // frc::SmartDashboard::PutNumber("rl pos: ", m_rl_speedMotor.GetSelectedSensorPosition());
  // frc::SmartDashboard::PutNumber("rr pos: ", m_rr_speedMotor.GetSelectedSensorPosition());

  // frc::SmartDashboard::PutNumber("Dx 1", dx.value());
  // frc::SmartDashboard::PutNumber("Dy 1", dy.value());


  //in mps, not joystck units
  if (abs(dy.value()) < 0.5) dy = units::meters_per_second_t{0};
  if (abs(dx.value()) < 0.5) dx = units::meters_per_second_t{0};

  frc::SmartDashboard::PutNumber("Dx", dx.value());
  frc::SmartDashboard::PutNumber("Dy", dy.value());
  frc::SmartDashboard::PutNumber("Dtheta", dtheta.value());
  frc::SmartDashboard::PutNumber("navx yaw", navx_yaw.value());


  // The desired field relative speed here is dx meters per second
  // toward the opponent's alliance station wall, and dy meters per
  // second toward the left field boundary. The desired rotation
  // is dtheta counterclockwise. The current robot angle is navx->GetYaw() degrees.
  speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    dx, dy, dtheta, frc::Rotation2d(navx_yaw));

  // Convert to module states. Here, we can use C++17's structured
  // bindings feature to automatically split up the array into its
  // individual SwerveModuleState components.
  auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);
  
  frc::SmartDashboard::PutNumber("odometry x", odometry->GetPose().X().value());
  frc::SmartDashboard::PutNumber("odometry y", odometry->GetPose().Y().value());
  std::cout << odometry->GetPose().X().value() << "\t" << odometry->GetPose().Y().value() << "\n";

  odometry->Update(navx_yaw, GetRealModuleStates());


  double fl_raw_yaw = m_fl_canCoder.GetAbsolutePosition();
  double fr_raw_yaw = m_fr_canCoder.GetAbsolutePosition();
  double bl_raw_yaw = m_rl_canCoder.GetAbsolutePosition();
  double br_raw_yaw = m_rr_canCoder.GetAbsolutePosition();

  //  // Optimize the wheel module yaw
  double fl_yaw = frc::InputModulus(fl_raw_yaw + DriveConstants::FLOFF, -180.0, 180.0);
  double fr_yaw = frc::InputModulus(fr_raw_yaw + DriveConstants::FROFF, -180.0, 180.0);
  double rl_yaw = frc::InputModulus(bl_raw_yaw + DriveConstants::BLOFF, -180.0, 180.0);
  double rr_yaw = frc::InputModulus(br_raw_yaw + DriveConstants::BROFF, -180.0, 180.0);

  
  auto fl_opt = frc::SwerveModuleState::Optimize(fl, units::degree_t(fl_yaw));
  auto fr_opt = frc::SwerveModuleState::Optimize(fr, units::degree_t(fr_yaw));
  auto rl_opt=  frc::SwerveModuleState::Optimize(bl, units::degree_t(rl_yaw));
  auto rr_opt = frc::SwerveModuleState::Optimize(br, units::degree_t(rr_yaw));


 //Finally command each swerve motor
 //TODO: fix weird thing with speed if drivers complain or we have time
  m_fl_angleMotor.SetVoltage(
    units::volt_t{std::clamp(
      m_rl_pid.Calculate(fl_yaw, fl_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE)}
  );
  m_fl_speedMotor.Set(
    0.2*std::clamp(fl_opt.speed.value(), -1.0, 1.0)
  );
  m_fr_angleMotor.SetVoltage(
    units::volt_t{std::clamp(
      m_rl_pid.Calculate(fr_yaw, fr_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE)}
  );
  m_fr_speedMotor.Set(
    0.2*std::clamp(fr_opt.speed.value(), -1.0, 1.0)
  );
  m_rl_angleMotor.SetVoltage(
    units::volt_t{std::clamp(
      m_rl_pid.Calculate(rl_yaw, rl_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE)}
  );
  m_rl_speedMotor.Set(
    0.2*std::clamp(rl_opt.speed.value(), -1.0, 1.0)
  );
  m_rr_angleMotor.SetVoltage(
    units::volt_t{std::clamp(
      m_rl_pid.Calculate(rr_yaw, rr_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE)}
  );

  m_rr_speedMotor.Set(
    0.2*std::clamp(rr_opt.speed.value(), -1.0, 1.0)
 );

}
//rest of this is for test/debug

//For tuning the PID of the motors that rotate the swerve modules
void Swerve::SetPID() {

  double P = frc::SmartDashboard::GetNumber("Swerve P", P);
  double I = frc::SmartDashboard::GetNumber("Swerve I", I);
  double D = frc::SmartDashboard::GetNumber("Swerve D", D);

  frc::SmartDashboard::PutNumber("Swerve P", P);
  frc::SmartDashboard::PutNumber("Swerve I", I);
  frc::SmartDashboard::PutNumber("Swerve D", D);


  m_fl_pid.SetP(P);
  m_fl_pid.SetI(I);
  m_fl_pid.SetD(D);

  m_fr_pid.SetP(P);
  m_fr_pid.SetI(I);
  m_fr_pid.SetD(D);

  m_rl_pid.SetP(P);
  m_rl_pid.SetI(I);
  m_rl_pid.SetD(D);
  
  m_rr_pid.SetP(P);
  m_rr_pid.SetI(I);
  m_rr_pid.SetD(D);
}

//gets all of the swerve motor objects
std::vector<TalonFX *> Swerve::getTalons() {
  return {&m_fl_angleMotor, &m_fl_speedMotor, &m_fr_angleMotor, &m_fr_speedMotor, &m_rl_angleMotor, &m_rl_speedMotor, &m_rr_angleMotor, &m_rr_speedMotor};
}

//for speed testing
//--------------TODO: Re-write--------------------
void Swerve::test1ms() {
  m_fl_speedMotor.SetVoltage(units::volt_t{1.0});
  m_fr_speedMotor.SetVoltage(units::volt_t{-1.0});
  m_rl_speedMotor.SetVoltage(units::volt_t{1.0});
  m_rr_speedMotor.SetVoltage(units::volt_t{-1.0});

}

void Swerve::test2_5ms() {
 m_fl_speedMotor.SetVoltage(units::volt_t{4.0});
  m_fr_speedMotor.SetVoltage(units::volt_t{-4.0});
  m_rl_speedMotor.SetVoltage(units::volt_t{4.0});
  m_rr_speedMotor.SetVoltage(units::volt_t{4.0});
}

void Swerve::test4ms() {
  m_fl_speedMotor.SetVoltage(units::volt_t{8.0});
  m_fr_speedMotor.SetVoltage(units::volt_t{8.0});
  m_rl_speedMotor.SetVoltage(units::volt_t{8.0});
  m_rr_speedMotor.SetVoltage(units::volt_t{8.0});
}

void Swerve::SetModulesStraight() {

  speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, frc::Rotation2d(units::degree_t{0}));


  auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);


  double fl_raw_yaw = m_fl_canCoder.GetAbsolutePosition();
  double fr_raw_yaw = m_fr_canCoder.GetAbsolutePosition();
  double bl_raw_yaw = m_rl_canCoder.GetAbsolutePosition();
  double br_raw_yaw = m_rr_canCoder.GetAbsolutePosition();

  //  // Optimize the wheel module yaw
  double fl_yaw = frc::InputModulus(fl_raw_yaw + DriveConstants::FLOFF, -180.0, 180.0);
  double fr_yaw = frc::InputModulus(fr_raw_yaw + DriveConstants::FROFF, -180.0, 180.0);
  double rl_yaw = frc::InputModulus(bl_raw_yaw + DriveConstants::BLOFF, -180.0, 180.0);
  double rr_yaw = frc::InputModulus(br_raw_yaw + DriveConstants::BROFF, -180.0, 180.0);

  auto fl_opt = frc::SwerveModuleState::Optimize(fl, units::degree_t(fl_yaw));
  auto fr_opt = frc::SwerveModuleState::Optimize(fr, units::degree_t(fr_yaw));
  auto rl_opt=  frc::SwerveModuleState::Optimize(bl, units::degree_t(rl_yaw));
  auto rr_opt = frc::SwerveModuleState::Optimize(br, units::degree_t(rr_yaw));
  

 //Finally command each swerve motor
  m_fl_angleMotor.SetVoltage(
    units::volt_t{std::clamp(
      m_rl_pid.Calculate(fl_yaw, fl_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE)}
  );
 
  m_fr_angleMotor.SetVoltage(
    units::volt_t{std::clamp(
      m_rl_pid.Calculate(fr_yaw, fr_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE)}
  );

  m_rl_angleMotor.SetVoltage(
    units::volt_t{std::clamp(
      m_rl_pid.Calculate(rl_yaw, rl_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE)}
  );
 
  m_rr_angleMotor.SetVoltage(
    units::volt_t{std::clamp(
      m_rl_pid.Calculate(rr_yaw, rr_opt.angle.Degrees().value()),
      -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE)}
  );

}

//--------------TODO: Re-write--------------------



  //saved this block for sim testing purposes
  // std::cout << "=========================================\n";
  // std::cout << "fl angle: " << fl_opt.angle.Degrees().value() << "\n";
  // std::cout << "fr angle: " << fr_opt.angle.Degrees().value() << "\n";
  // std::cout << "bl angle: " << rl_opt.angle.Degrees().value() << "\n";
  // std::cout << "br angle: " << rr_opt.angle.Degrees().value() << "\n";
  // std::cout << "\n";
  // std::cout << "fl speed: " << fl_opt.speed.value() << "\n";
  // std::cout << "fr speed: " << fr_opt.speed.value() << "\n";
  // std::cout << "bl speed: " << rl_opt.speed.value() << "\n";
  // std::cout << "br speed: " << rr_opt.speed.value() << "\n";
  // std::cout << "=========================================\n";