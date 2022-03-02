#include "FramePeriods.h"

FramePeriods::FramePeriods() {

}

void FramePeriods::addTalon(WPI_TalonFX& talon) {
   talons.push_back(&talon);
  //  setFramePeriods(talon);
}

void FramePeriods::periodic() {
    for (WPI_TalonFX* talon: talons) {
        checkFramePeriods(talon);
    }
}

void FramePeriods::setFramePeriods(WPI_TalonFX& talon) {
    ErrorCollection err;
    for (int i = 0; i < statusFrameAttempts; i++) {
     //   err.NewError(talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, st));
      //  err.NewError(talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, st));
        err.NewError(talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, lt));
        err.NewError(talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, lt));
        err.NewError(talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, lt));
        err.NewError(talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, lt));
        err.NewError(talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, lt));
     //   err.NewError(talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, st)); //idk if we actually use
        err.NewError(talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, lt));

        //don't think we use 4 or 5, but i'm keeping control frame 3 (general) the default time
        err.NewError(talon.SetControlFramePeriod(ControlFrame::Control_4_Advanced, lt));
        err.NewError(talon.SetControlFramePeriod(ControlFrame::Control_6_MotProfAddTrajPoint, lt));

        if (err.GetFirstNonZeroError() == ErrorCode::OK) return;
    }
    
}

void FramePeriods::checkFramePeriods(WPI_TalonFX* talon) {
    if ((*talon).HasResetOccurred()) setFramePeriods(*talon);   
}