//Generated From Template


#pragma once

#include <frc/Timer.h>
#include <string>

#include "RobotIO.h"
#include "Drivetrain.h"

namespace auton
{
    enum eState
    {
        STATE_START = 0,
        STATE_MOVE_1 = 1,
        STATE_DONE = 2,
        STATE_ERROR = 99
    };

    //TODO - Timeout Constants
    static constexpr double dMove1Timeout = 10.0;

    //TODO - Motor Speed Constants
    
    //TODO - Setpoint Constants

    // Paths
    std::string move_1_path = "Example Path";
    
}

class Auton
{
public:

    // Constructor/Destructor
    Auton();
    ~Auton()
        {  }


    // Class Methods
    void Initialize( RobotIO *p_pRobotIO );
    void Execute();
    void UpdateInputStatus();
    
private:
    auton::eState m_eState;

    RobotIO *m_pRobotIO;

    frc::Timer *m_pTimeoutTimer;

    //State Machine Instances
    Drivetrain m_Drivetrain;

};