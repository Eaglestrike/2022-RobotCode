#pragma once

#include "Constants.h"
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <rev/ColorMatch.h>
#include <rev/ColorSensorV3.h>


class Channel{
    public:
        enum State{
            IDLE,
            RUN
        };

    Channel();
    void Periodic();
    void Run();
    void Stop();
    void setState(State newState);

    private:
        WPI_TalonFX m_channel{ChannelConstants::channelMotorPort};

        State state = IDLE;
};