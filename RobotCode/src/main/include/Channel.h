#pragma once

#include "Constants.h"
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>


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

        // // 2 Photogates
        // frc::DigitalInput photogate1{ChannelConstants::photogate1};
        // frc::DigitalInput photogate2{ChannelConstants::photogate2};
        State state;
};