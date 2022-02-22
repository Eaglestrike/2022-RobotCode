#include "units/units.h"
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

const int PI = 3.14159265359;


namespace OIConstants{
    //Joystick & xbox input ports
    const int l_joy_port = 0;
    const int r_joy_port = 1;
    const int O_joy_port = 2;
}

namespace DriveConstants{
     //Dimensions of the Robot - SwerveDrive.h (meters)
    const double Width = 29 * 0.0254;
    const double Length = 29 * 0.0254;

    frc::Translation2d ModulePositions[4] = {
        frc::Translation2d{units::meter_t{Length/2}, units::meter_t{Width/2}},
        frc::Translation2d{units::meter_t{Length/2}, units::meter_t{-Width/2}},
        frc::Translation2d{units::meter_t{-Length/2}, units::meter_t{Width/2}},
        frc::Translation2d{units::meter_t{-Length/2}, units::meter_t{-Width/2}},
    };

    frc::SwerveDriveKinematics<4> kDriveKinematics{
        ModulePositions[0],
        ModulePositions[1],
        ModulePositions[2],
        ModulePositions[3]
    };

    //we're supposed to figure these out for our robot, these are what the sample had
    const double ksVolts = 0.587;
    const double kvVoltSecsPerMeter = 2.3;
    const double kaVoltSecsSquaredPerMeter = 0.0917;
    const double kMaxSpeedMetersPerSec = 4.2672;
    const double kvVoltSecsPerRadian = 3.41;
    const double kaVoltSecsSquaredPerRadian = 0.111;
    const double kMaxChassisRotationSpeed = 10*PI;


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
    const double rotP = 0.0006;
    const double rotI = 0.00004;
    const double rotD = 0.0;
}

namespace ModuleConstants {
    const double drivingGearRatio = 1/6.12;
    const double angleGearRatio = 1/12.8;
    const double kEncoderCPR = 4096;
    const double kWheelDiameterMeters = 0;
   
    //idk if these next ones are correct, or even really what they are
    const double kMaxModuleAngularSpeedRadsPerSec = 10*PI;
    const double kMaxModuleAngularAccelRadsPerSecSquared = 10*PI;
    const double kDriveEncoderDistPerPulse = (kWheelDiameterMeters * PI) / ((double) kEncoderCPR * drivingGearRatio);
    const double kTurningEncoderDistPerPulse = (2.0 * PI) / (kEncoderCPR* angleGearRatio); //assumes encoders 1:1 reduction w/module shaft

    const double kpTurningController = 2.4;
    const double kPDrivingController = 2.3;

    //again, no idea if these are correct...
    const double kvVoltSecsPerMeter = 1.47;
    const double kaVoltSecsSquaredPerMeter = 0.0348;
    const double kvVoltSecsPerRad = 1.47;
    const double kaVoltSecsSquaredPerRad = 0.0348;

}

//this will be more for conceptual testing so i don't really care if these are right
namespace AutoConstants {
    const double kMaxSpeedMetersPerSec = DriveConstants::kMaxSpeedMetersPerSec;
    const double kMaxAccelMetersPerSecSquared = 3;
    const double kMaxAngularSpeedRadPerSec = PI;
    const double kMaxAngualrSpeedRadPerSecSquared = Pi;

    const double kpXController = 1;
    const double kpYController = 1;
    const double kpθController = 3.5; //yes I am milking the greek characters for all they are worth

    //not sure how to tranlate cause c++ requires a distance template and idk what that's supposed to be
    // public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
    //             new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond * 10,
    //                     kMaxAngularSpeedRadiansPerSecondSquared * 10);

}