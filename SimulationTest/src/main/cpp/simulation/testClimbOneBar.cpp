#include "testClimbOneBar.h"
#include <fstream>

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
   // std::cout << "passed: " << passed << "\n";
    if (passed == 2) {
        std::cout << "test passed\n";
        return;
    }
    if (passed == 1) {
        std::cout << "test failed\n";
        return;
    }
    switch(state) {
        case WAITING_FOR_EXTEND_BUTTON:
            std::cout << "Waiting for raw button 1...\n";
            break;
        case TESTING:
            test();
            break;
        default:
            break;

    }
}

bool testClimbOneBar::test() {
    if (passedExtending == 0) extending();
    else if (passedExtending == 2 && passedRetracting == 0) retracting();
    return passedExtending == 2 && passedRetracting == 2;
}

bool testClimbOneBar::extending() {

    climber.SetState(Climber::State::VERTICAL_ARM_EXTEND);
    climber.Periodic(0, 0, 0, false, false, false, false);

    //evaluate if failed, good, or done
    //failes if: anything in brake, motors going wrong way, motors going too fast
   std::cout << "testing extending...\n";
    if (climber.getBrake().Get() || 
    climber.getMotor().GetSelectedSensorPosition() < 0 || climber.getMotor().GetMotorOutputPercent() > 0.5) {
        if (climber.getMotor().GetSelectedSensorPosition() < 0) std::cout << "test failed: motor position < 0\n";
        if (climber.getMotor().GetMotorOutputPercent() > 0.5) std::cout << "test failed: motor power > 0.5\n";
        passedExtending = 1;
        passed = 1;
        return false;
    }

    if (abs(ClimbConstants::motorExtendedPose - climber.getMotor().GetSelectedSensorPosition()) <= ClimbConstants::motorPoseTolerance
        && climber.getPIDCntrl().AtSetpoint()) {
        std::cout << "Testing state transition...\n";
        climber.SetState(Climber::State::VERTICAL_ARM_EXTEND);
        climber.Periodic(0, 0, 0, false, true, false, false);
        if (climber.GetState() == Climber::State::VERTICAL_ARM_RETRACT) {
            std::cout << "test passed :)\n";
            passedExtending = 2;
        }
        else {
            std::cout << "test failed: state isn't correct \n";
            passed = 1;
        }
    }

    return true;
}

bool testClimbOneBar::retracting() {
    std::cout << "testing retracting...\n";

    climber.SetState(Climber::State::VERTICAL_ARM_EXTEND);
    climber.Periodic(0, 0, 0, false, false, false, false); //can implement swing simulator later, but should be about no swing in this case anyway

    //in real life the motor would have to work so hard to compress below 0 that it won't, it will oscilate in simulation
    if (climber.getBrake().Get() || 
    (climber.getMotor().GetSelectedSensorPosition() > ClimbConstants::motorExtendedPose && climber.getMotor().GetMotorOutputPercent() > 0)
     || climber.getMotor().GetMotorOutputPercent() > 0.5) {
        if (climber.getMotor().GetSelectedSensorPosition() > ClimbConstants::motorExtendedPose ) std::cout << "test failed: motor position > extended\n";
        if (climber.getMotor().GetMotorOutputPercent() > 0.5) std::cout << "test failed: motor power > 0.5\n";
        passedRetracting = 1;
        passed = 1;
        return false;
    }

     if (abs(ClimbConstants::motorRetractedPose - climber.getMotor().GetSelectedSensorPosition()) <= ClimbConstants::motorPoseTolerance
     && climber.getPIDCntrl().AtSetpoint()) {
            if (!climber.getBrake().Get()) {
                std::cout << "test failed: brake didn't engage\n";
                passedRetracting = 1;
                passed = 1;
                return false;
            }
            climber.SetState(Climber::State::VERTICAL_ARM_EXTEND);
            climber.Periodic(0, 0, 0, false, false, false, false); //test all good
            if (climber.GetState() != Climber::State::TEST_DIAGONAL_ARM_EXTEND) {
                std::cout << "test failed: didn't enter test diagonal arm extend when all good\n";
                passedRetracting = 1;
                passed = 1;
                return false;
            }

            climber.SetState(Climber::State::VERTICAL_ARM_EXTEND);
            climber.Periodic(100, 100, 0, false, false, false, false); //test bad pitch
            if (climber.GetState() != Climber::State::VERTICAL_ARM_EXTEND) {
                std::cout << "test failed: didn't stay in vertical arm extend with bad pitch\n";
                passedRetracting = 1;
                passed = 1;
                return false;
            }

            climber.SetState(Climber::State::VERTICAL_ARM_EXTEND);
            climber.Periodic(0, 0, 2, false, false, false, false); //test no time
            if (climber.GetState() != Climber::State::VERTICAL_ARM_EXTEND) {
                std::cout << "test failed: didn't remain with no time\n";
                passedRetracting = 1;
                passed = 1;
                return false;
            }

        }
    
    std::cout << "test passed :)\n";
    passed = 2;
    passedRetracting = 2;
    return true;
}

void testClimbOneBar::setState(State newState){
    prevState = state;
    state = newState;
}

