#pragma once

#include "RobotIO.h"
#include <frc/Timer.h>
#include <string>
#include "Drivetrain.h"
#include "Intake.h"
#include "Magazine.h"
#include "Shooter.h"

namespace auton03 
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
    static const std::string path1Name = "Right Side Path";
}

class Auton03 
{
    public:
        // Constructor / Deconstructor
        Auton03(Drivetrain * drivetrain, Intake *intake);
        ~Auton03()
            {  }
        

        // Class Methods
        void Initialize( RobotIO *p_pRobotIO );
        void Execute();


    private:
        auton03::eState m_eState;

        RobotIO *m_pRobotIO;
        frc::Timer *m_pTimeoutTimer;

        Drivetrain *m_Drivetrain;
        Intake *m_intake;
        Magazine m_Magazine;
        Shooter m_Shooter;
};