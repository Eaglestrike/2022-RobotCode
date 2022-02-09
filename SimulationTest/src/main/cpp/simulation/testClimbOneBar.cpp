#include "testClimbOneBar.h"

testClimbOneBar::testClimbOneBar(Climber& c) : climber(c) {
    initTestClimbOneBar();
}

void testClimbOneBar::initTestClimbOneBar() {
    climber.getFullExtend().Set(false);
    climber.getMedExtend().Set(false);  
    climber.getMotor().SetNeutralMode(NeutralMode::Brake);
    climber.getMotor().SetSelectedSensorPosition(0);
    climber.SetState(Climber::State::IDLE);
}

void testClimbOneBar::periodic() {
    switch(state) {
        case WAITING_FOR_EXTEND_BUTTON:
            std::cout << "\n";
            std::cout << "Waiting for raw button 1...\r";
            break;
        case EXTENDING:
            extending();
            break;
        case WAITING_FOR_RETRACT_BUTTON:
            std::cout << "\n";
            std::cout << "Waiting for raw button 2...\r";
            break;
        case RETRACTING:
            break;
        case IDLE:
            initTestClimbOneBar();
            break;

    }
}

bool testClimbOneBar::extending() {
    climber.SetState(Climber::State::VERTICAL_ARM_EXTEND);
    climber.Periodic(0, 0, 0, false, false, false, false);

    //evaluate if failed, good, or done
    //failes if: anything in brake, motors going wrong way, motors going too fast
    std::cout << "testing...\r";
    if (climber.getBrake().Get() || 
    climber.getMotor().GetMotorOutputPercent() <= 0 || climber.getMotor().GetMotorOutputPercent() > 0.5) {
        std::cout << "\n";
        if (climber.getMotor().GetMotorOutputPercent() == 0) std::cout << "test failed: motor not moving\r";
        if (climber.getMotor().GetMotorOutputPercent() < 0) std::cout << "test failed: motor going backwards\r";
        if (climber.getMotor().GetMotorOutputPercent() > 0.5) std::cout << "test failed: motor power > 0.5\r";
        return false;
    }

    if (abs(ClimbConstants::motorExtendedPose - climber.getMotor().GetSelectedSensorPosition()) <= ClimbConstants::motorPoseTolerance
        && climber.getMotor().GetMotorOutputPercent() <= ClimbConstants::motorStoppedOutput) {
        std::cout << "\n";
        std::cout << "Testing state transition...\r";
        climber.SetState(Climber::State::VERTICAL_ARM_EXTEND);
        climber.Periodic(0, 0, 0, false, true, false, false);
        if (climber.GetState() == Climber::State::VERTICAL_ARM_RETRACT) {
            std::cout << "\n";
            std::cout << "test passed :)\r";
        }
        else {
            std::cout << "\n";
            std::cout << "test failed: state isn't correct \r";
        }
    }

    return true;
}

void testClimbOneBar::setState(State newState){
    prevState = state;
    state = newState;
}

