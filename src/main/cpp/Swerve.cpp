#include "Swerve.h"
#include <iostream>

Swerve::Swerve() {
    m_fl_pid.EnableContinuousInput(-180, 180);
    m_fr_pid.EnableContinuousInput(-180, 180);
    m_rl_pid.EnableContinuousInput(-180, 180);
    m_rr_pid.EnableContinuousInput(-180, 180);

    m_fl_pid.SetIntegratorRange(-0.5, 0.5);
    m_fr_pid.SetIntegratorRange(-0.5, 0.5);
    m_rl_pid.SetIntegratorRange(-0.5, 0.5);
    m_rr_pid.SetIntegratorRange(-0.5, 0.5);
}

void Swerve::Init() {
  m_fl_angle_encoder.Reset();
  m_fr_angle_encoder.Reset();
  m_rl_angle_encoder.Reset();
  m_rr_angle_encoder.Reset();
}

void Swerve::Periodic(units::meters_per_second_t dx, units::meters_per_second_t dy, units::radians_per_second_t dtheta, 
units::degree_t navx_yaw) {

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

  auto& fl_raw_yaw = logger->get_float64("swerve.fl.raw_yaw");
  fl_raw_yaw = ((m_fl_angle_encoder.Get().value()));//+1.0 + DriveConstants::FLOFF)/4.96)*360.0 - 180.0;
 
  auto& fr_raw_yaw = logger->get_float64("swerve.fr.raw_yaw");
  fr_raw_yaw =  ((m_fr_angle_encoder.Get().value()));//+0.3 + DriveConstants::FROFF)/3.44)*360.0 - 180.0;
  
  auto& bl_raw_yaw = logger->get_float64("swerve.bl.raw_yaw");
  bl_raw_yaw = ((m_rl_angle_encoder.Get().value()));//+1.0 + DriveConstants::BLOFF)/4.96)*360.0 - 180.0;

  auto& br_raw_yaw = logger->get_float64("swerve.br.raw_yaw");
  br_raw_yaw = ((m_rr_angle_encoder.Get().value()));//+1.0 + DriveConstants::BROFF)/4.96)*360.0 - 180.0;
  
  frc::SmartDashboard::PutNumber("Fl angle encoder reading", m_fl_angle_encoder.Get().value());
  frc::SmartDashboard::PutNumber("Fr angle encoder reading", m_fr_angle_encoder.Get().value());
  frc::SmartDashboard::PutNumber("bl angle encoder reading", m_rl_angle_encoder.Get().value());
  frc::SmartDashboard::PutNumber("br angle encoder reading", m_rr_angle_encoder.Get().value());


  auto& fl_yaw = logger->get_float64("swerve.fl.calib_yaw");
  auto& fr_yaw = logger->get_float64("swerve.fr.calib_yaw");
  auto& rl_yaw = logger->get_float64("swerve.bl.calib_yaw");
  auto& rr_yaw = logger->get_float64("swerve.br.calib_yaw");

  // Optimize the wheel module yaw
  fl_yaw = frc::InputModulus(fl_raw_yaw + DriveConstants::FLOFF, -180.0, 180.0);
  fr_yaw = frc::InputModulus(fr_raw_yaw + DriveConstants::FROFF, -180.0, 180.0);
  rl_yaw = frc::InputModulus(bl_raw_yaw + DriveConstants::BLOFF, -180.0, 180.0);
  rr_yaw = frc::InputModulus(br_raw_yaw + DriveConstants::BROFF, -180.0, 180.0);
  auto fl_opt = frc::SwerveModuleState::Optimize(fl, units::degree_t(fl_yaw));
  auto fr_opt = frc::SwerveModuleState::Optimize(fr, units::degree_t(fr_yaw));
  auto rl_opt= frc::SwerveModuleState::Optimize(bl, units::degree_t(rl_yaw));
  auto rr_opt = frc::SwerveModuleState::Optimize(br, units::degree_t(rr_yaw));

  logger->get_float64("swerve.fl.target_yaw") = fl_opt.angle.Degrees().value();
  logger->get_float64("swerve.fr.target_yaw") = fr_opt.angle.Degrees().value();
  logger->get_float64("swerve.bl.target_yaw") = rl_opt.angle.Degrees().value();
  logger->get_float64("swerve.br.target_yaw") = rr_opt.angle.Degrees().value();

  logger->get_float64("swerve.fl.target_speed") = fl_opt.speed.value();
  logger->get_float64("swerve.fr.target_speed") = fr_opt.speed.value();
  logger->get_float64("swerve.bl.target_speed") = rl_opt.speed.value();
  logger->get_float64("swerve.br.target_speed") = rr_opt.speed.value();

  frc::SmartDashboard::PutNumber("fl opt angle", fl_opt.angle.Degrees().value());
  frc::SmartDashboard::PutNumber("fl opt speed", fl_opt.speed.value());
  frc::SmartDashboard::PutNumber("fr opt angle", fr_opt.angle.Degrees().value());
  frc::SmartDashboard::PutNumber("fr opt speed", fr_opt.speed.value());
  frc::SmartDashboard::PutNumber("bl opt angle", rl_opt.angle.Degrees().value());
  frc::SmartDashboard::PutNumber("bl opt speed", rl_opt.speed.value());
  frc::SmartDashboard::PutNumber("br opt angle", rr_opt.angle.Degrees().value());
  frc::SmartDashboard::PutNumber("br opt speed", rr_opt.speed.value());

  frc::SmartDashboard::PutNumber("fl yaw reading", fl_yaw);
  frc::SmartDashboard::PutNumber("fr yaw reading", fr_yaw);
  frc::SmartDashboard::PutNumber("bl yaw reading", rl_yaw);
  frc::SmartDashboard::PutNumber("br yaw reading", rr_yaw);

  m_fl_angleMotor.Set(ControlMode::PercentOutput, 0.1);
  std::cout << "here\n";

//  Finally command each swerve motor
  // m_fl_angleMotor.Set( ControlMode::PercentOutput,
  //   std::clamp(
  //     m_fl_pid.Calculate(fl_yaw, fl_opt.angle.Degrees().value()),
  //     -1.0, 1.0)
  // );

  // m_fl_speedMotor.Set( ControlMode::PercentOutput,
  //   std::clamp(
  //     m_fl_speed_pid.Calculate(m_fl_speed_encoder.GetRate(), fl_opt.speed.value()),
  //     -1.0, 1.0)
  // );

  // m_fr_angleMotor.Set( ControlMode::PercentOutput,
  //   std::clamp(
  //     m_fr_pid.Calculate(fr_yaw, fr_opt.angle.Degrees().value()),
  //     -1.0, 1.0)
  // );

  // m_fr_speedMotor.Set( ControlMode::PercentOutput,
  //   std::clamp(
  //     m_fr_speed_pid.Calculate(m_fr_speed_encoder.GetRate(), fr_opt.speed.value()),
  //     -1.0, 1.0)
  // );

  // m_rl_angleMotor.Set( ControlMode::PercentOutput,
  //   std::clamp(
  //     m_rl_pid.Calculate(rl_yaw, rl_opt.angle.Degrees().value()),
  //     -1.0, 1.0)
  // );

  // m_rl_speedMotor.Set( ControlMode::PercentOutput,
  //   std::clamp(
  //     m_rl_speed_pid.Calculate(m_rl_speed_encoder.GetRate(), rl_opt.speed.value()),
  //     -1.0, 1.0)
  // );

  // m_rr_angleMotor.Set( ControlMode::PercentOutput,
  //   std::clamp(
  //     m_rr_pid.Calculate(rr_yaw, rr_opt.angle.Degrees().value()),
  //     -1.0, 1.0)
  // );

  // m_rr_speedMotor.Set( ControlMode::PercentOutput,
  //   std::clamp(
  //     m_rr_speed_pid.Calculate(m_rr_speed_encoder.GetRate(), rr_opt.speed.value()),
  //     -1.0, 1.0)
  // );
  
  
}