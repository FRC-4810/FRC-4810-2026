#include "Auton_01.h"

// Constructor
Auton01::Auton01()
{
    m_eState = auton01::eState::STATE_START;

    m_pRobotIO = nullptr;
}

// Initialize Shooter
void Auton01::Initialize( RobotIO *p_pRobotIO )
{
    m_pRobotIO = p_pRobotIO;

    m_Drivetrain.Initialize( p_pRobotIO );

    m_pTimeoutTimer = new frc::Timer();
    m_pTimeoutTimer->Reset();
}

void Auton01::Execute()
{
    // Check that m_pRobotIO has been assigned
    if ( m_pRobotIO != nullptr )
    {
        // ***************
        // * Start State *
        // ***************

        if ( m_eState == auton01::eState::STATE_START )
        {
            // Move path Logic
            m_Drivetrain.LoadPath( auton01::path1Name );
            m_Drivetrain.FollowPath();
            m_pTimeoutTimer->Reset();
            m_pTimeoutTimer->Start();

            m_eState = auton01::eState::STATE_MOVE_1;
        }

        // ****************
        // * Move 1 State *
        // ****************
        else if ( m_eState == auton01::eState::STATE_MOVE_1 )
        {
            // Move path Logic
            m_Drivetrain.FollowPath();

            // Check if path done
            if(m_Drivetrain.IsPathFinished() || (double)m_pTimeoutTimer->Get() >= auton01::dMove1Timer)
            {
                m_Drivetrain.Stop();
                m_eState = auton01::eState::STATE_DONE;
            }
        }

        else if ( m_eState == auton01::eState::STATE_DONE )
        {
            
        }
        

        // Error or unkown state
        else
        {
            printf("Auton_01.cpp: Error state or unknown state\n");
        }
    }
    // m_pRobotIO is nullptr
    else
    {
        printf("Auton_01.cpp: m_pRobotIO is nullptr\n");
    }
}
