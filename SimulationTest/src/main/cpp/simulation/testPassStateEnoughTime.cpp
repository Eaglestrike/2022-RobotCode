// #include "testPassStateEnoughTime.h"
// #include "Constants.h"
// #include "Climber.h"
// using namespace std; 
// enum State{ //feel free to change the names if they suck
//             IDLE,

//            //only for climb to first bar
//             VERTICAL_ARM_EXTEND,
//             VERTICAL_ARM_RETRACT,

//             TEST_DIAGONAL_ARM_EXTEND,
//             //can be used for all subsequent bars
//             DIAGONAL_ARM_EXTEND,
//             DIAGONAL_ARM_RAISE, //hooks onto bar
//             DIAGONAL_ARM_RETRACT //involves retracting & returning to vertical            
//         };

// testPassStateEnoughTime::testPassStateEnoughTime(Climber& c) : climber(c)
// {
//     init(); 
// }

// void testPassStateEnoughTime::init()
// { 
//     test(VERTICAL_ARM_EXTEND); 
//     test(VERTICAL_ARM_RETRACT);
//     test(DIAGONAL_ARM_EXTEND); 
//     test(DIAGONAL_ARM_RAISE);
//     test(DIAGONAL_ARM_RETRACT); 
// }

// void testPassStateEnoughTime::periodic(int time)
// { 
//     climber.Periodic(0,0,time, true,true,true,true, true); 
// }

// void testPassStateEnoughTime::test(State state) 
// { 
//     switch(state){ 
//         case VERTICAL_ARM_EXTEND: 
//             periodic(2);
//             if (climber.stateJustChanged()) { 
//                 cout << "Test failed: State did change" << " State: VERTICAL_ARM_EXTEND " << endl; 
//                 return; 
//             } 
//             else { 
//                 cout << "Test Passes: State did not change" << " State: VERTICAL_ARM_EXTEND" << endl; 
//             }
//             periodic(0); 
//             if (climber.stateJustChanged()) { 
//                 cout << "Test Passed: State did change" << " State: VERTICAL_ARM_EXTEND" << state << endl; 
//                 return; 
//             } 
//             else { 
//                 cout << "Test failed: State did not change" << " State: VERTICAL_ARM_EXTEND" << state << endl; 
//                 return;
//             }
//         case VERTICAL_ARM_RETRACT: 
//             periodic(2);
//             if (climber.stateJustChanged()) { 
//                 cout << "Test failed: State did change"  << " State: VERTICAL_ARM_RETRACT" << endl; 
//                 return; 
//             } 
//             else { 
//                 cout << "Test Passes: State did not change" << " State: VERTICAL_ARM_RETRACT" <<  endl; 
//             }
//             periodic(0); 
//             if (climber.stateJustChanged()) { 
//                 cout << "Test Passed: State did change" << " State: VERTICAL_ARM_RETRACT" << endl; 
//                 return; 
//             } 
//             else { 
//                 cout << "Test failed: State did not change" << " State: VERTICAL_ARM_RETRACT" << endl; 
//                 return;
//             }
//         case DIAGONAL_ARM_EXTEND: 
//             periodic(2);
//             if (climber.stateJustChanged()) { 
//                 cout << "Test failed: State did change" << " State: DIAGONAL_ARM_EXTEND" << endl; 
//                 return; 
//             } 
//             else { 
//                 cout << "Test Passes: State did not change" << " State: DIAGONAL_ARM_EXTEND" << endl; 
//             }
//             periodic(0); 
//             if (climber.stateJustChanged()) { 
//                 cout << "Test Passed: State did change" << " State: DIAGONAL_ARM_EXTEND"<< endl; 
//                 return; 
//             } 
//             else { 
//                 cout << "Test failed: State did not change" << " State: DIAGONAL_ARM_EXTEND" << endl; 
//                 return;
//             }
//         case DIAGONAL_ARM_RAISE: 
//             periodic(2);
//             if (climber.stateJustChanged()) { 
//                 cout << "Test failed: State did change" << " State:  DIAGONAL_ARM_RAISE" << endl; 
//                 return; 
//             } 
//             else { 
//                 cout << "Test Passes: State did not change" << " State: DIAGONAL_ARM_RAISE" << endl; 
//             }
//             periodic(0); 
//             if (climber.stateJustChanged()) { 
//                 cout << "Test Passed: State did change" << " State: DIAGONAL_ARM_RAISE" << endl; 
//                 return; 
//             } 
//             else { 
//                 cout << "Test failed: State did not change" << " State: DIAGONAL_ARM_RAISE" <<  endl; 
//                 return;
//             }
//         case DIAGONAL_ARM_RETRACT: 
//             periodic(2);
//             if (climber.stateJustChanged()) { 
//                 cout << "Test failed: State did change" << " State: DIAGONAL_ARM_RETRACT" << endl; 
//                 return; 
//             } 
//             else { 
//                 cout << "Test Passes: State did not change" << " State: DIAGONAL_ARM_RETRACT" << endl; 
//             }
//             periodic(0); 
//             if (climber.stateJustChanged()) { 
//                 cout << "Test Passed: State did change" << " State: DIAGONAL_ARM_RETRACT" << endl; 
//                 return; 
//             } 
//             else { 
//                 cout << "Test failed: State did not change" << " State: DIAGONAL_ARM_RETRACT" << endl; 
//                 return;
//             }
//     }
// }

// void testPassStateEnoughTime::setState(State newState) 
// { 
//     prevState = state;
//     state = newState;
// }


