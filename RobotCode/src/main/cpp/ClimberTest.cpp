#include "Climber.h"

//this lets us test states individually without having to go through the state machine

//this assumes climber periodic is being called in test periodic, AFTER climbPeriodic()
//the above AFTER is important! otherwise, the state won't be right!

//also call this if no other buttons are pressed
void Climber::InitializeTests() {
    state = IDLE;
    prevState = IDLE;
}

void Climber::testRaiseVerticalArm() {
    SetState(VERTICAL_ARM_EXTEND);
}

void Climber::testRetractVerticalArm() {
    SetState(VERTICAL_ARM_RETRACT);
}

void Climber::testSeesIfHooked() {
    SetState(TEST_DIAGONAL_ARM_EXTEND);
}

void Climber::testDiagonalExtension() {
    SetState(DIAGONAL_ARM_EXTEND);
}
        
void Climber::testDiagonalArmRaise() {
    SetState(DIAGONAL_ARM_RAISE);
}
        
void Climber::testBarTraversalFromRaised() {
    SetState(DIAGONAL_ARM_RETRACT);
}