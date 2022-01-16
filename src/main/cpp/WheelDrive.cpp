#include <WheelDrive.h>

// WheelDrive::WheelDrive(int angleMotorPort, int speedMotorPort,
//     int encoderPortA, int encoderPortB)
//     : angleMotor(angleMotorPort), speedMotor(speedMotorPort),
//     encoder(encoderPortA, encoderPortB) {

//     pidController.EnableContinuousInput(-180, 180);
// }

WheelDrive::WheelDrive(int angleMotorPort, int speedMotorPort,
    int encoderPortA, int encoderPortB, int pwmPort)
    : angleMotor(angleMotorPort), speedMotor(speedMotorPort),
    encoder(encoderPortA, encoderPortB), absEncoder(pwmPort){
    pidController.EnableContinuousInput(-180, 180);
    initializeController.EnableContinuousInput(0,1);
}


void 
WheelDrive::drive(double speed, double angle){
    double value = normalizeEncoderValue();
    if(abs(value - angle) > 90){
        angle = (angle > 0 )? angle - 180: angle + 180;
        m_reverse = true;
    }
    else {
        m_reverse = false;
    }
    
    double turnOutput = std::clamp(pidController.Calculate(value, angle), -0.5, 0.5);
    
    m_speedOut = (m_reverse)? -1*0.7*speed: 0.7*speed;
    
    speedMotor.Set(m_speedOut);
    angleMotor.Set(turnOutput);
}


//Converts Encoder Value to -180 - +180
double 
WheelDrive::normalizeEncoderValue(){
    m_currEncoderValue = encoder.Get();
    int encoderDelta = -1*(m_currEncoderValue - m_prevEncoderValue);
    m_angle += (double) encoderDelta / 1024 * 360;
    m_angle = (m_angle > 360) ? m_angle-360: m_angle;
    m_angle = (m_angle < 0) ? m_angle+360 : m_angle;
    m_prevEncoderValue = m_currEncoderValue;
    m_angle = (m_angle > 180)? m_angle - 360: m_angle;
    return m_angle;
}


void 
WheelDrive::setPID(){
    double pGain_a = frc::SmartDashboard::GetNumber("P angle", 0.0);
    frc::SmartDashboard::PutNumber("P angle", pGain_a);
    double iGain_a = frc::SmartDashboard::GetNumber("I angle", 0.0);
    frc::SmartDashboard::PutNumber("I angle", iGain_a);
    double dGain_a = frc::SmartDashboard::GetNumber("D angle", 0.0);
    frc::SmartDashboard::PutNumber("D angle", dGain_a);
    initializeController.SetPID(pGain_a, iGain_a, dGain_a);
}


void 
WheelDrive::resetEncoder(){
    encoder.Reset();
}


double
WheelDrive::getVelocity(){
    return speedMotor.GetSelectedSensorVelocity() * 1/6.12;
}


double 
WheelDrive::getAngle(){
    return normalizeEncoderValue();
}


//Front Left = 0.43
//Front Right = 0.695 
//Back Left = 0.78
//Back Right = 0.19 
//Input is from 0-1
double
WheelDrive::GetabsAngle(){
    double angle;
    angle = absEncoder.Get().value();
    //frc::SmartDashboard::PutNumber("Absolute encoder", angle);
    return angle;
}

//Testing ?????? 
void 
WheelDrive::initialization(double Offset){
    m_reverse = false;
    double turnOutput = std::clamp(initializeController.Calculate(GetabsAngle(), Offset), -0.5, 0.5);
    angleMotor.Set(turnOutput);
}


void
WheelDrive::Debug(){
    frc::SmartDashboard::PutBoolean("Reverse", m_reverse);
}