#pragma once


namespace
GeneralConstants{
    const double timeStep = 0.02;
}


namespace
OIConstants{
    //Joystick & xbox input ports
    const int l_joy_port = 0;
    const int r_joy_port = 1;
    const int O_joy_port = 2;
}


namespace
DriveConstants{
    //Dimensions of the Robot - SwerveDrive.h
    const double Width = 0.394;
    const double Length = 0.394;

    //Absolute Encoder Offset Values - SwerveDrive.h
    const double FLOFF = 0.43;
    const double FROFF = 0.695;
    const double BLOFF = 0.78;
    const double BROFF = 0.19;

    //Ports for Back Right Swerve Module - SwerveDrive.h
    const int BRanglePort = 18;
    const int BRspeedPort = 17;
    const int BRencoderPortA = 6;
    const int BRencoderPortB = 7;
    const int BRencoderPWM = 13;

    //Ports for Back Left Swerve Module - SwerveDrive.h
    const int BLanglePort = 16;
    const int BLspeedPort = 15;
    const int BLencoderPortA = 4;
    const int BLencoderPortB = 5;
    const int BLencoderPWM = 12;

    //Ports for Front Right Swerve Module - SwerveDrive.h
    const int FRanglePort = 14;
    const int FRspeedPort = 13;
    const int FRencoderPortA = 2;
    const int FRencoderPortB = 3;
    const int FRencoderPWM = 11;

    //Ports for Front Left Swerve Module - SwerveDrive.h
    const int FLanglePort = 12;
    const int FLspeedPort = 11;
    const int FLencoderPortA = 0;
    const int FLencoderPortB = 1;
    const int FLencoderPWM = 10;

    //Wheeldrive 
    constexpr double MAX_VOLTS = 1.0;
    const double speedGearRatio = 1/6.12;
    const double angleGearRatio = 1/12.8;
    const double P = 0.012;
    const double I = 0.008;
    const double D = 0.0001;
    const double Pinit = 4.8;
    const double Iinit = 1.2;
    const double Dinit = 0.02;

    //Swerve Controller
    const double fwdstrP = 0.45;
    const double fwdstrI = 0.0;
    const double fwdstrD = 0.0;
    const double rotP = 0.0001;
    const double rotI = 0.00004;
    const double rotD = 0.0;
}

//TODO: determine constants
namespace
ShooterConstants{
    //turret pid
    const double Tp = 0.0;
    const double Ti = 0.0;
    const double Td = 0.0;

    //flywheel pid
    const double FWp = 0.0;
    const double FWi = 0.0;
    const double FWd = 0.0;

    //hood pid
    const double Hp = 0.0;
    const double Hi = 0.0;
    const double Hd = 0.0;

    //turret rotation limts
    const double UpperRotLimit = 0.0;
    const double LowerRotLimit = 0.0;

    //error tolerances (kicker is counting as part of channel)
    const double TurretTolerance = 0.0;
    const double FlywheelTolerance = 0.0;
    const double HoodTolerance = 0.0;
}

namespace
IntakeConstants{}

namespace
ClimbConstants{}
