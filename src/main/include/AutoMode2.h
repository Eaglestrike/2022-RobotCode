#pragma once

#include "Intake.h"
#include "Shooter.h"
#include "SwerveDrive.h"
#include "Traj2.h"
#include "Trajectories.h"

class AutoMode2 {
   public:
    AutoMode2(SwerveDrive* swerve, Shooter* shooter, Intake* intake) {
        m_swerve = swerve;
        m_shooter = shooter;
        m_intake = intake;
        m_autostate = -1;
    }

    virtual void tick(double time) = 0;
    void start(double time) { m_autostate = 0; };

   protected:
    // utilities for waiting
    virtual void beginWait(double time) { m_wait_start = time; }

    virtual bool hasElapsed(double timeout, double time) {
        return time - m_wait_start > timeout;
    }
    // Non-owning pointers to robot subsystems to run the mode
    SwerveDrive* m_swerve;
    Shooter* m_shooter;
    Intake* m_intake;
    int m_autostate{0};

    double m_wait_start{0.0};
};

class TwoBallAuto : public AutoMode2 {
   public:
    TwoBallAuto(SwerveDrive* swerve, Shooter* shooter, Intake* intake)
        : AutoMode2(swerve, shooter, intake) {
        segment1 = trajectories::twoball_segment1();
    }

    virtual void tick(double time) override {
        switch (m_autostate) {
            case 0:
                m_intake->setState(Intake::IDLE);
                m_shooter->setState(Shooter::IDLE);
                m_swerve->Drive(0, 0, 0, 0, true);
                beginWait(time);
                m_autostate++;
                break;
            case 1:
                if (hasElapsed(0.5, time)) m_autostate++;
                break;
            case 2:
                m_shooter->setState(Shooter::SHOOT);
                beginWait(time);
                m_autostate++;
                break;
            case 3:
                if (hasElapsed(0.5, time)) {
                    m_shooter->setState(Shooter::IDLE);
                    m_autostate++;
                }
                break;
            case 4:
                // Intake and path follow
                m_intake->setState(Intake::RUN);
                m_shooter->setState(Shooter::LOAD);
                segment1->start(time);
                m_autostate++;
                break;
            case 5:
                if (segment1->is_done(time)) {
                    m_autostate++;
                }
                segment1->drive(time, m_swerve);
                break;
            case 6:
                m_swerve->Drive(0, 0, 0, 0, true);
                m_shooter->setState(Shooter::SHOOT);
                beginWait(time);
                m_autostate++;
                break;
            case 7:
                if (hasElapsed(0.5, time)) m_autostate++;
                break;
            case 8:
                m_shooter->setState(Shooter::IDLE);
                break;
        }
    }

    std::unique_ptr<RobotTraj> segment1;
};
