#include "Intake.h"

// Constructor
Intake::Intake()
{
    m_eState = intake::eState::STATE_START;
    m_eCommand = intake::eCommand::COMMAND_NONE;

    m_pRobotIO = nullptr;
}

// Initialize Intake
void Intake::Initialize( RobotIO *p_pRobotIO )
{
    m_pRobotIO = p_pRobotIO;

    m_pTimeoutTimer = new frc::Timer();
    m_pTimeoutTimer->Reset();
}

void Intake::Execute()
{
    // Check that m_pRobotIO has been assigned
    if ( m_pRobotIO != nullptr )
    {
        // ***************
        // * Start State *
        // ***************
        if ( m_eState == intake::eState::STATE_START )
        {

        }

        // **************
        // * Idle State *
        // **************
        else if ( m_eState == intake::eState::STATE_IDLE )
        {
            // *----------------------*
            // * Manual Raise Command *
            // *----------------------*
            if ( m_eCommand == intake::eCommand::COMMAND_MANUAL_RAISE )
            {

            }

            // *----------------------*
            // * Manual Lower Command *
            // *----------------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_MANUAL_LOWER )
            {
                
            }

            // *--------------------*
            // * Auto Raise Command *
            // *--------------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_AUTO_RAISE )
            {

            }

            // *--------------------*
            // * Auto Lower Command *
            // *--------------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_AUTO_LOWER )
            {
                
            }

            // *-----------------------*
            // * Manual Intake Command *
            // *-----------------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_MANUAL_INTAKE )
            {

            }

            // *------------------------*
            // * Manual Outtake Command *
            // *------------------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_MANUAL_OUTTAKE )
            {
                
            }

            // Handle unrecognized command
            else if ( m_eCommand != intake::eCommand::COMMAND_NONE && m_eCommand != intake::eCommand::COMMAND_STOP )
            {
                printf("Intake.cpp: Unrecognized command");
            }
        }

        // **********************
        // * Manual Raise State *
        // **********************
        else if ( m_eState == intake::eState::STATE_MANUAL_RAISE )
        {
            
        }

        // **********************
        // * Manual Lower State *
        // **********************
        else if ( m_eState == intake::eState::STATE_MANUAL_LOWER )
        {
            
        }

        // ********************
        // * Auto Raise State *
        // ********************
        else if ( m_eState == intake::eState::STATE_AUTO_RAISE )
        {
            
        }

        // ********************
        // * Auto Lower State *
        // ********************
        else if ( m_eState == intake::eState::STATE_AUTO_LOWER )
        {
            
        }

        // ***********************
        // * Manual Intake State *
        // ***********************
        else if ( m_eState == intake::eState::STATE_MANUAL_INTAKE )
        {
            
        }

        // ************************
        // * Manual Outtake State *
        // ************************
        else if ( m_eState == intake::eState::STATE_MANUAL_OUTTAKE )
        {
            
        }
        // Handle unknown or error state
        else
        {
            printf("Intake.cpp: Unknown state or error state \n");
        }
    } 
    else
    {
        printf("Intake.cpp: m_pRobotIO is nullptr \n");
    }
}