#pragma once

#include "RobotIO.h"
#include <frc/Timer.h>
#include <string>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "Intake.h"
#include "Magazine.h"
#include "Shooter.h"

namespace auton01 
{
    enum eState
    {
        STATE_START = 0,
        STATE_MOVE_1 = 1,
        STATE_DONE = 2,
        STATE_ERROR = 99
    };


    //Timers
    static constexpr double dMove1Timer = 50; // I made this very large so it never ends -Cory

    //Paths
    static const std::string path1Name = "Test Path";
}

class Auton01 
{
    public:
        // Constructor / Deconstructor
        Auton01(subsystems::CommandSwerveDrivetrain * drivetrain, Intake *intake);
        ~Auton01()
            {  }
        

        // Class Methods
        void Initialize( RobotIO *p_pRobotIO );
        void Execute();


    private:
        auton01::eState m_eState;

        RobotIO *m_pRobotIO;
        frc::Timer *m_pTimeoutTimer;

        subsystems::CommandSwerveDrivetrain *m_Drivetrain;
        Intake *m_intake;
        Magazine m_Magazine;
        Shooter m_Shooter;
};