#include <WheelDrive.h>


WheelDrive::WheelDrive(int angleMotorPort, int speedMotorPort,
    int encoder)
    : angleMotor(angleMotorPort, "Drivebase"), speedMotor(speedMotorPort, "Drivebase"),
    m_canCoder(encoder, "Drivebase"){
    pidController.EnableContinuousInput(-180, 180);
    initializeController.EnableContinuousInput(-180, 180);

    angleMotor.SetNeutralMode(NeutralMode::Brake);
    speedMotor.SetNeutralMode(NeutralMode::Brake);

    angleMotor.SetSafetyEnabled(false);
    speedMotor.SetSafetyEnabled(false);

    // m_canCoder.SetStatusFramePeriod(CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20, 100);
    m_canCoder.ClearStickyFaults();
}


//Drive function, set each module's speed and angle
void 
WheelDrive::drive(double speed, double angle, double offSet){    
    double value = getAngle(offSet);
    if(abs(value - angle) > 90){
        angle = (angle > 0 )? angle - 180: angle + 180;
        m_reverse = true;
    }
    else {
        m_reverse = false;
    }
    
    double turnOutput = std::clamp(pidController.Calculate(value, angle),-0.5, 0.5);
    
    m_speedOut = (m_reverse)? -1*0.87*speed: 0.87*speed;
    
    speedMotor.Set(m_speedOut);
    angleMotor.Set(turnOutput);
}


//Converts Encoder Value to -180, +180
double 
WheelDrive::normalizeEncoderValue(){
    m_currEncoderValue = m_canCoder.GetPosition();
    int encoderDelta = -1*(m_currEncoderValue - m_prevEncoderValue);
    m_angle += (double) encoderDelta;
    m_angle = (m_angle > 360) ? m_angle-360: m_angle;
    m_angle = (m_angle < 0) ? m_angle+360 : m_angle;
    m_prevEncoderValue = m_currEncoderValue;
    m_angle = (m_angle > 180)? m_angle - 360: m_angle;
    return m_angle;
}


//Function to set the PID for Swerve Modules
void 
WheelDrive::setPID(){
    double pGain_a = frc::SmartDashboard::GetNumber("P angle", 0.0);
    frc::SmartDashboard::PutNumber("P angle", pGain_a);
    double iGain_a = frc::SmartDashboard::GetNumber("I angle", 0.0);
    frc::SmartDashboard::PutNumber("I angle", iGain_a);
    double dGain_a = frc::SmartDashboard::GetNumber("D angle", 0.0);
    frc::SmartDashboard::PutNumber("D angle", dGain_a);
    //initializeController.SetPID(pGain_a, iGain_a, dGain_a);
    pidController.SetPID(pGain_a, iGain_a, dGain_a);
}


//Reset encoder curruntly not used
void 
WheelDrive::resetEncoder(){
    m_reverse = false;
}


//Returns the velocity of Swerve Module in ft per 0.02s
double
WheelDrive::getVelocity(){
    return speedMotor.GetSelectedSensorVelocity()/2048 * (1/6.12) * 4.375 * PI / 5/ 12;
}


//Returns the angle of Swerve Module
double 
WheelDrive::getAngle(double offset){
    return m_canCoder.GetAbsolutePosition() - offset;
}


//Initializes the zero position of Swerve Module
void 
WheelDrive::initialization(double Offset){
    m_reverse = false;
    //double turnOutput = std::clamp(initializeController.Calculate(getAngle(Offset), Offset), -0.5, 0.5);
    //angleMotor.Set(turnOutput); 
}


//Helper function
void
WheelDrive::Debug(){
    //frc::SmartDashboard::PutNumber("relative", normalizeEncoderValue());
    //frc::SmartDashboard::PutNumber("absolute", GetabsAngle());
}


void
WheelDrive::Stop(){
    angleMotor.Set(ControlMode::PercentOutput, 0.0);
    speedMotor.Set(ControlMode::PercentOutput, 0.0);
}