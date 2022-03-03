#include "testFullClimb.h"

testFullClimb::testFullClimb() {
    Initialize();
}

void testFullClimb::Initialize() {
    passedIdle = false;
    passedVerticalArmExtend = false;
    passedVerticalArmRetract = false;
    passedtestDiagonalArmExtend = false;
    passedDiagonalArmExtend = false;
    passedDiagonalArmRaise = false;
    passedDiagonalArmRetract = false;
    
    failureEncountered = false;

    climber.SetState(Climber::State::IDLE);
}

void testFullClimb::Periodic() {
    if (failureEncountered) return;
    if (!passedIdle) {
        testIdle();
        return;
    }
    if (!passedVerticalArmExtend) {
        testVerticalArmExtend();
        return;
    }
    if (!passedVerticalArmRetract) {
        testVerticalArmRetract();
        return;
    }
    if (!passedtestDiagonalArmExtend) {
        testTestDiagonalArmExtend();
        return;
    }
    if (!passedDiagonalArmExtend) {
        testDiagonalArmExtend();
        return;
    }
    if (!passedDiagonalArmRaise) {
        testDiagonalArmRaise();
        return;
    }
    if (!passedDiagonalArmRetract) {
        testDiagonalArmRetract();
        return;
    }
}

void testFullClimb::testIdle() {
    //
    //to start: motor position should be 0, pneumatics should be half extended, brakes should be on
    //without pass idle, should not exit idle
    //idle should exit when thing is true
}

void testFullClimb::testVerticalArmExtend() {
    //brakes should be off
    //using pid, motor extends
    //once motor gets there and driven forward is true, go to vertical arm retract
}

void testFullClimb::testVerticalArmRetract() {
    //do pid back to thing
    //goes to test hooked once motor is there
}

void testFullClimb::testTestDiagonalArmExtend() {
    //set the current to be super freaking high at the beginning, then set it to be lower after a second to test wait
    //see if it moves on to diagonal arm extend
}

void testFullClimb::testDiagonalArmExtend() {
    //set pneumatics to all the way lowered
    //extend motor pid thing again
}

void testFullClimb::testDiagonalArmRaise() {
    //may arm retract on its own?
    //set pneumatics to have raised
    //wait for go-ahead
}

void testFullClimb::testDiagonalArmRetract() {
    //retract dose arms
}