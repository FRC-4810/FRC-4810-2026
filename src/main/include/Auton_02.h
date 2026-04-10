#pragma once

#include "RobotIO.h"
#include <frc/Timer.h>
#include <string>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "Intake.h"
#include "Magazine.h"
#include "Shooter.h"
#include <frc/controller/PIDController.h>

namespace auton02 
{
    enum eState
    {
        STATE_START = 0,
        STATE_MOVE_1 = 1,
        STATE_RAMP_UP = 2,
        STATE_SHOOT = 3,
        STATE_DONE = 4,
        STATE_ERROR = 99
    };


    //Timers
    static constexpr double dMove1Timer = 2.3;    //Todo - tune this timer
    static constexpr double dShootTimer = 4;    //Todo - tune this timer

    //Setpoints
    static constexpr double dTargetGyro = 3.162013;    //Radians - todo tune this - current: -15 deg
    static constexpr double xMoveSpeed = -0.5;  //m/s - todo tune this. Is x or y towards driver?
    static constexpr double yMoveSpeed = 0.0;   //m/s - todo tune this. Is x or y towards driver?

    
}

class Auton02 
{
    public:
        // Constructor / Deconstructor
        Auton02(subsystems::CommandSwerveDrivetrain *m_Drivetrain);
        ~Auton02()
            {  }
        

        // Class Methods
        void Initialize( RobotIO *p_pRobotIO );
        void Execute();


    private:
        auton02::eState m_eState;

        RobotIO *m_pRobotIO;
        frc::Timer *m_pTimeoutTimer;

        //-GMS - todo tune this
        frc::PIDController m_RotationPIDController
        {
            -1.0,    //Kp
            0.0,    //Ki
            0.0     //Kd
        };

        subsystems::CommandSwerveDrivetrain *m_Drivetrain;
        Intake m_Intake;
        Magazine m_Magazine;
        Shooter m_Shooter;
};