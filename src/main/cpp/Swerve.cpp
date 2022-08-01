#include "Swerve.h"
#include <iostream>
#include <iostream>
#include <fstream>
#include <sstream>

//wrapper class for wpilib swerve
//currently a mess cause I was trying to debug it

Swerve::Swerve(AHRS * nx, DataLogger * logger) : m_navx{nx}, m_logger{logger} {

   // outstream.open("/home/lvuser/odometry_out.txt", std::ofstream::out); //TEMPORARY - REMOVE
   //freopen("/home/lvuser/odometry_out.txt", "w", stdout);

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

void Swerve::initializeOdometry(frc::Rotation2d gyroAngle, frc::Pose2d initPose) {
  delete odometry; //attempting garbage collection
  odometry = new frc::SwerveDriveOdometry<4>(m_kinematics, gyroAngle, initPose);
}

//I'll have to put more thought into the navx angle offset, but I'll just add it to the module angle for now
//i think the ticks to degrees thing is still valid here?? TODO: come back to this when testing odometry
frc::ChassisSpeeds Swerve::getSpeeds() {
  wpi::array<frc::SwerveModuleState, 4> moduleStates(wpi::empty_array);

  // std::cout << "talon vel to mps 0: " << talonVelToMps(m_fl_speedMotor.GetSelectedSensorVelocity()).value() << "\n";
  // std::cout << "talon vel to mps 1: " << talonVelToMps(m_fr_speedMotor.GetSelectedSensorVelocity()).value() << "\n";
  // std::cout << "talon vel to mps 2: " << talonVelToMps(m_rl_speedMotor.GetSelectedSensorVelocity()).value() << "\n";
  // std::cout << "talon vel to mps 3: " << talonVelToMps(m_rr_speedMotor.GetSelectedSensorVelocity()).value() << "\n";


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
  
  return m_kinematics.ToChassisSpeeds(moduleStates);
}


//4 m/s is 2500
//2.5 m/s is 1600
//1 m/s is 700

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

  // frc::SmartDashboard::PutNumber("fl speed:", m_fl_speedMotor.GetSelectedSensorVelocity());
  // frc::SmartDashboard::PutNumber("fr speed:", m_fr_speedMotor.GetSelectedSensorVelocity());
  // frc::SmartDashboard::PutNumber("rl speed:", m_rl_speedMotor.GetSelectedSensorVelocity());
  // frc::SmartDashboard::PutNumber("rr speed:", m_rr_speedMotor.GetSelectedSensorVelocity());

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


  std::cout << odometry->GetPose().X().value() << "\t" << odometry->GetPose().Y().value() << "\n";
 // outstream << odometry->GetPose().X().value() << "\t" << odometry->GetPose().Y().value() << "\n";

  odometry->Update(navx_yaw, moduleStates);

  frc::SmartDashboard::PutNumber("odometry x", odometry->GetPose().X().value());
  frc::SmartDashboard::PutNumber("odometry y", odometry->GetPose().Y().value());



  // //raw reported encoder reading, convrted to degrees
  // auto& fl_raw_yaw = m_logger->get_float64("swerve.fl.raw_yaw");
  // auto& fr_raw_yaw = m_logger->get_float64("swerve.fr.raw_yaw");
  // auto& bl_raw_yaw = m_logger->get_float64("swerve.bl.raw_yaw");
  // auto& br_raw_yaw = m_logger->get_float64("swerve.br.raw_yaw");


  double fl_raw_yaw = m_fl_canCoder.GetAbsolutePosition();
  double fr_raw_yaw = m_fr_canCoder.GetAbsolutePosition();
  double bl_raw_yaw = m_rl_canCoder.GetAbsolutePosition();
  double br_raw_yaw = m_rr_canCoder.GetAbsolutePosition();

  // frc::SmartDashboard::PutNumber("fl raw yaw", fl_raw_yaw);
  // frc::SmartDashboard::PutNumber("fr raw yaw", fr_raw_yaw);
  // frc::SmartDashboard::PutNumber("bl raw yaw", bl_raw_yaw);
  // frc::SmartDashboard::PutNumber("br raw yaw", br_raw_yaw);

  //angle + offset, converted to degrees & modulated
  // auto& fl_yaw = m_logger->get_float64("swerve.fl.calib_yaw");
  // auto& fr_yaw = m_logger->get_float64("swerve.fr.calib_yaw");
  // auto& rl_yaw = m_logger->get_float64("swerve.bl.calib_yaw");
  // auto& rr_yaw = m_logger->get_float64("swerve.br.calib_yaw");

  //  // Optimize the wheel module yaw
  double fl_yaw = frc::InputModulus(fl_raw_yaw + DriveConstants::FLOFF, -180.0, 180.0);
  double fr_yaw = frc::InputModulus(fr_raw_yaw + DriveConstants::FROFF, -180.0, 180.0);
  double rl_yaw = frc::InputModulus(bl_raw_yaw + DriveConstants::BLOFF, -180.0, 180.0);
  double rr_yaw = frc::InputModulus(br_raw_yaw + DriveConstants::BROFF, -180.0, 180.0);

  // frc::SmartDashboard::PutNumber("fl yaw reading", fl_yaw);
  // frc::SmartDashboard::PutNumber("fr yaw reading", fr_yaw);
  // frc::SmartDashboard::PutNumber("bl yaw reading", rl_yaw);
  // frc::SmartDashboard::PutNumber("br yaw reading", rr_yaw);
  
  auto fl_opt = frc::SwerveModuleState::Optimize(fl, units::degree_t(fl_yaw));
  auto fr_opt = frc::SwerveModuleState::Optimize(fr, units::degree_t(fr_yaw));
  auto rl_opt=  frc::SwerveModuleState::Optimize(bl, units::degree_t(rl_yaw));
  auto rr_opt = frc::SwerveModuleState::Optimize(br, units::degree_t(rr_yaw));

  //target angle & speed for swerve modules
  // m_logger->get_float64("swerve.fl.target_yaw") = fl_opt.angle.Degrees().value();
  // m_logger->get_float64("swerve.fr.target_yaw") = fr_opt.angle.Degrees().value();
  // m_logger->get_float64("swerve.bl.target_yaw") = rl_opt.angle.Degrees().value();
  // m_logger->get_float64("swerve.br.target_yaw") = rr_opt.angle.Degrees().value();

  // m_logger->get_float64("swerve.fl.target_speed") = fl_opt.speed.value();
  // m_logger->get_float64("swerve.fr.target_speed") = fr_opt.speed.value();
  // m_logger->get_float64("swerve.bl.target_speed") = rl_opt.speed.value();
  // m_logger->get_float64("swerve.br.target_speed") = rr_opt.speed.value();

  // frc::SmartDashboard::PutNumber("fl opt angle", fl_opt.angle.Degrees().value());
  // frc::SmartDashboard::PutNumber("fl opt speed", fl_opt.speed.value());
  // frc::SmartDashboard::PutNumber("fr opt angle", fr_opt.angle.Degrees().value());
  // frc::SmartDashboard::PutNumber("fr opt speed", fr_opt.speed.value());
  // frc::SmartDashboard::PutNumber("bl opt angle", rl_opt.angle.Degrees().value());
  // frc::SmartDashboard::PutNumber("bl opt speed", rl_opt.speed.value());
  // frc::SmartDashboard::PutNumber("br opt angle", rr_opt.angle.Degrees().value());
  // frc::SmartDashboard::PutNumber("br opt speed", rr_opt.speed.value());

  //SetPID();
  

 //Finally command each swerve motor
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

//for debugging. Prints states of swerve modules without running motors. Takes in optional swerve module states
void Swerve::DisabledPeriodic(wpi::array<frc::SwerveModuleState, 4> * moduleStates) {


  //raw encoder reading
  //todo: make sure that for each motor, goes from 180 to -180 and loops once per rotation
  double fl_raw_yaw = m_fl_canCoder.GetAbsolutePosition();
  double fr_raw_yaw = m_fr_canCoder.GetAbsolutePosition();
  double bl_raw_yaw = m_rl_canCoder.GetAbsolutePosition();
  double br_raw_yaw = m_rr_canCoder.GetAbsolutePosition();

  frc::SmartDashboard::PutNumber("fl raw yaw", fl_raw_yaw);
  frc::SmartDashboard::PutNumber("fr raw yaw", fr_raw_yaw);
  frc::SmartDashboard::PutNumber("bl raw yaw", bl_raw_yaw);
  frc::SmartDashboard::PutNumber("br raw yaw", br_raw_yaw);

  //std::cout << "here\n";

  //apply offset (THIS SHOULD MAKE ALL MODULES REPORT SAME ANGLE)
  //todo: make sure that they all read the same at the same position
  double fl_yaw = frc::InputModulus(fl_raw_yaw + DriveConstants::FLOFF, -180.0, 180.0);
  double fr_yaw = frc::InputModulus(fr_raw_yaw + DriveConstants::FROFF, -180.0, 180.0);
  double rl_yaw = frc::InputModulus(bl_raw_yaw + DriveConstants::BLOFF, -180.0, 180.0);
  double rr_yaw = frc::InputModulus(br_raw_yaw + DriveConstants::BROFF, -180.0, 180.0);

  frc::SmartDashboard::PutNumber("fl yaw reading", fl_yaw);
  frc::SmartDashboard::PutNumber("fr yaw reading", fr_yaw);
  frc::SmartDashboard::PutNumber("bl yaw reading", rl_yaw);
  frc::SmartDashboard::PutNumber("br yaw reading", rr_yaw);

  // check if empty array
  if (moduleStates == nullptr) {
    return;
  }
  auto [fl, fr, bl, br] = *moduleStates;

  auto fl_opt = frc::SwerveModuleState::Optimize(fl, units::degree_t(fl_yaw));
  auto fr_opt = frc::SwerveModuleState::Optimize(fr, units::degree_t(fr_yaw));
  auto rl_opt= frc::SwerveModuleState::Optimize(bl, units::degree_t(rl_yaw));
  auto rr_opt = frc::SwerveModuleState::Optimize(br, units::degree_t(rr_yaw));

  //check that these are right (should be equivilant to module states)
  frc::SmartDashboard::PutNumber("fl opt angle", fl_opt.angle.Degrees().value());
  frc::SmartDashboard::PutNumber("fr opt angle", fr_opt.angle.Degrees().value());
  frc::SmartDashboard::PutNumber("bl opt angle", rl_opt.angle.Degrees().value());
  frc::SmartDashboard::PutNumber("br opt angle", rr_opt.angle.Degrees().value());

}


std::vector<TalonFX *> Swerve::getTalons() {
  return {&m_fl_angleMotor, &m_fl_speedMotor, &m_fr_angleMotor, &m_fr_speedMotor, &m_rl_angleMotor, &m_rl_speedMotor, &m_rr_angleMotor, &m_rr_speedMotor};
}


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