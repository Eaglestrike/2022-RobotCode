#include "SwerveModule.h"

SwerveModule::SwerveModule(int mn, int turnMotID, int driveMotID, double zeroOff, bool invT, bool invTh) : 
moduleNum(mn), mTurningMotor(turnMotID), mDrivingMotor(driveMotID), mZeroOffset(zeroOff) {
    
    mTurningMotor.ConfigFactoryDefault();
    mTurningMotor.ConfigOpenloopRamp(0.1);
    mTurningMotor.ConfigClosedloopRamp(0.1);

    //we are using DutyCycleEncoders. idk how to specitfy what the port is supposed to be?
    mTurningMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::PulseWidthEncodedPosition);
    mDrivingMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::PulseWidthEncodedPosition);
    
    mTurningMotor.SetInverted(invT);
    mDrivingMotor.SetInverted(invTh);
    mTurningMotor.SetSelectedSensorPosition(0);
    mDrivingMotor.SetSelectedSensorPosition(0);

    mTurningMotor.Config_kF(0, kF);
    mTurningMotor.Config_kP(0, kP);
    mTurningMotor.Config_kI(0, kI);
    mTurningMotor.Config_IntegralZone(0, kI_Zone);
    mTurningMotor.ConfigMaxIntegralAccumulator(0, kMaxIAccum);
    mTurningMotor.Config_kD(0, kD);
    mTurningMotor.ConfigMotionCruiseVelocity(kCruiseVelocity);
    mTurningMotor.ConfigMotionAcceleration(kMotionAccel);
    mTurningMotor.ConfigAllowableClosedloopError(0, kErrorBand);

    mTurningMotor.SetNeutralMode(NeutralMode::Brake);
    mDrivingMotor.SetNeutralMode(NeutralMode::Brake);

    //mTurningPID.EnableContinuousInput(-PI, PI);

    //can set up other simulation stuff
}

void SwerveModule::resetEncoders() {
    //will this reset the abs encoder? should that even get reset?
    mTurningMotor.SetSelectedSensorPosition(0);
    mDrivingMotor.SetSelectedSensorPosition(0);
    //can reset simulation encoders. also could use pheonix sim stuff?
}

double SwerveModule::getTurningRadians() {
    return mTurningMotor.GetSelectedSensorPosition() * ModuleConstants::kTurningEncoderDistPerPulse; //is this affected by gear ratio?
}

double SwerveModule::getVelocity() {
    return mDrivingMotor.GetSelectedSensorVelocity() * ModuleConstants::kDriveEncoderDistPerPulse*10; //why the *10?
}
   
void SwerveModule::setDesiredState(frc::SwerveModuleState state) {
    frc::SwerveModuleState outputState = frc::SwerveModuleState::Optimize(state, frc::Rotation2d{units::radian_t{getTurningRadians()}});

    //calculate drive output from drive PID
    mDriveOutput = mDrivePID.Calculate(getVelocity(), outputState.speed.value());
    double driveFeedforward; //use trapezoid thing to calculate
    mTurnOutput; //use trapexoid thing to calculate

    mDrivingMotor.Set(ControlMode::PercentOutput, mDriveOutput+driveFeedforward);
    mTurningMotor.Set(ControlMode::PercentOutput, mTurnOutput);

}

void SwerveModule::setPercentOutput(double output) {
    mDrivingMotor.Set(ControlMode::PercentOutput, output);
}

void SwerveModule::setBrakeMode(bool on) {
    mDrivingMotor.Set(on ? NeutralMode::Brake : NeutralMode::Coast);
    mTurningMotor.SetNeutralMode(NeutralMode::Brake);
}