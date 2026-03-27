//Generated From Template

#include "Auton.h"

// Constructor
Auton::Auton()
{
    m_eState = auton::eState::STATE_START;
    m_pRobotIO = nullptr;
}

void Auton::Initialize( RobotIO *p_pRobotIO )
{
    m_pRobotIO = p_pRobotIO;

    m_pTimeoutTimer = new frc::Timer();
    m_pTimeoutTimer->Reset();

    m_Drivetrain.Initialize(p_pRobotIO);
}

void Auton::UpdateInputStatus()
{

}

void Auton::Execute()
{
    //Check if m_pRobotIO has been assigned
    if(m_pRobotIO != nullptr)
    {
        // ***************
        // * Start State *
        // ***************
        if(m_eState == auton::eState::STATE_START)
        {
            m_Drivetrain.LoadPath(auton::move_1_path, true); 
            m_eState = auton::eState::STATE_MOVE_1;
            m_pTimeoutTimer->Reset();
            m_pTimeoutTimer->Start();
        }

        else if(m_eState == auton::eState::STATE_MOVE_1)
        {
            //TODO - write move logic and check if completed
            m_Drivetrain.FollowPath();
            if(m_Drivetrain.IsPathFinished() || (double)m_pTimeoutTimer->Get() >= auton::dMove1Timeout)
            {
                m_eState = auton::eState::STATE_DONE;
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Stop();
                m_Drivetrain.Stop();
            }
        }

        else if(m_eState == auton::eState::STATE_DONE)
        {
                //TODO - Add any logic for when auton is done if needed
        }

        // Handle Error State or unknown state
        else
        {
            // Error Logic Here if needed
            printf("ERROR in Auton.cpp - Error or unknown state\n");
        }
    }
    else
    {
        // Handle RobotIO nullptr error
        printf("ERROR in Auton.cpp - Robot IO Pointer Null\n");
    }
}