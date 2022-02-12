#include "testWaited.h"
#include <fstream>

testWaited::testWaited(Climber& c) : climber(c) {
    initTestWaited();
}

void testWaited::initTestWaited() {
    climber.getFullExtend().Set(false);
    climber.getMedExtend().Set(false);  
    climber.getMotor().SetNeutralMode(NeutralMode::Brake);
    climber.getMotor().SetSelectedSensorPosition(0);
    climber.SetState(Climber::State::IDLE);
}

void testWaited::periodic() {
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
            std::cout << "Wait testing...\n";
            test();
            break;
        default:
            break;

    }
}

bool testWaited::test() {
    climber.SetState(Climber::State::TEST_DIAGONAL_ARM_EXTEND);
    climber.Periodic(0, 0, timer.GetMatchTime().value(), false, false, false, false, false);

    // std::cout << timer.GetMatchTime().value() << "\n";

    if (climber.GetState() == Climber::State::TEST_DIAGONAL_ARM_EXTEND) {
        std::cout << "Not switched yet\n";
    } 
    else if (climber.GetState() == Climber::State::DIAGONAL_ARM_EXTEND) {
        std::cout << "Switched to Diagonal Arm Extend\n";
        passed = 1; // TESTING
    }
    else {
        std::cout << "huh- switched to a different state, TEST FAILED\n";
        passed = 1; // 
    }

    return true; // TESTING
}

void testWaited::setState(State newState){
    prevState = state;
    state = newState;
}