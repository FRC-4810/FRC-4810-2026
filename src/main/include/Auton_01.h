#pragma once

#include "RobotIO.h"
#include <frc/Timer.h>
#include <string>
#include "Drivetrain.h"

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
    static constexpr double dMove1Timer = 5;

    //Paths
    static const std::string path1Name = "Test Path";
}

class Auton01 
{
    public:
        // Constructor / Deconstructor
        Auton01();
        ~Auton01()
            {  }
        

        // Class Methods
        void Initialize( RobotIO *p_pRobotIO );
        void Execute();


    private:
        auton01::eState m_eState;

        RobotIO *m_pRobotIO;
        frc::Timer *m_pTimeoutTimer;

        Drivetrain m_Drivetrain;
};