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

            m_eState = auton01::eState::STATE_MOVE_1;
        }

        // ****************
        // * Move 1 State *
        // ****************
        else if ( m_eState == auton01::eState::STATE_MOVE_1 )
        {
            // Move path Logic

            // Check if path done
            if(false)
            {
                m_eState = auton01::eState::STATE_MOVE_1;
            }
        }

        else if ( m_eState == auton01::eState::STATE_DONE )
        {

        }
        

        // Error or unkown state
        else
        {
            printf("Shooter.cpp: Error state or unknown state\n");
        }
    }
    // m_pRobotIO is nullptr
    else
    {
        printf("Shooter.cpp: m_pRobotIO is nullptr\n");
    }
}
