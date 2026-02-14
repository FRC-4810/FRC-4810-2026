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

    // Refresh arm motor configurator with m_MotorConfigs
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
            // Go to idle state
            m_eState = intake::eState::STATE_IDLE;
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
                // Check upper limit switch
                if ( /*Upper limit hit*/ )
                {
                    m_eCommand = intake::eCommand::COMMAND_NONE;
                    return;
                }
                // Enable coast mode

                // Set speed on arm motor

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to manual raise
                m_eState = intake::eState::STATE_MANUAL_RAISE
            }

            // *----------------------*
            // * Manual Lower Command *
            // *----------------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_MANUAL_LOWER )
            {
                // Check lower limit switch
                if ( /*Lower limit hit*/ )
                {
                    m_eCommand = intake::eCommand::COMMAND_NONE;
                    return;
                }
                // Enable coast mode

                // Set speed on arm motor

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to manual lower
                m_eState = intake::eState::STATE_MANUAL_LOWER
            }

            // *--------------------*
            // * Auto Raise Command *
            // *--------------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_AUTO_RAISE )
            {
                // Check upper limit switch
                if ( /*Upper limit hit*/ )
                {
                    m_eCommand = intake::eCommand::COMMAND_NONE;
                    return;
                }
                // Enable coast mode

                // Set speed on arm motor

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to manual raise
                m_eState = intake::eState::STATE_MANUAL_RAISE
            }

            // *--------------------*
            // * Auto Lower Command *
            // *--------------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_AUTO_LOWER )
            {
                // Check lower limit switch
                if ( /*Lower limit hit*/ )
                {
                    m_eCommand = intake::eCommand::COMMAND_NONE;
                    return;
                }
                // Enable coast mode

                // Set speed on arm motor

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to auto lower
                m_eState = intake::eState::STATE_AUTO_LOWER
            }

            // *-----------------------*
            // * Manual Intake Command *
            // *-----------------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_MANUAL_INTAKE )
            {
                // Set speed on intake/outtake motor

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to manual intake
                m_eState = intake::eState::STATE_MANUAL_INTAKE
            }

            // *------------------------*
            // * Manual Outtake Command *
            // *------------------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_MANUAL_OUTTAKE )
            {
                // Set speed on intake/outtake motor

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to manual outtake
                m_eState = intake::eState::STATE_MANUAL_OUTTAKE
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
            // Check timeout timer
            bool bIsTimedOut = false;
            if ( (double)m_pTimeoutTimer->Get() >= intake::dManualRaiseTimeout )
            {
                bIsTimedOut = true;
            }

            if ( m_eCommand == intake::eCommand::COMMAND_STOP || bIsTimedOut == true || /* Upper limit hit*/ )
            {
                // Stop arm motors

                // Enable brake mode

                // Reset state and command
                m_eState = intake::eState::STATE_IDLE;
                m_eCommand = intake::eCommand::COMMAND_NONE;
            }
        }

        // **********************
        // * Manual Lower State *
        // **********************
        else if ( m_eState == intake::eState::STATE_MANUAL_LOWER )
        {
            // Check timeout timer
            bool bIsTimedOut = false;
            if ( (double)m_pTimeoutTimer->Get() >= intake::dManualLowerTimeout )
            {
                bIsTimedOut = true;
            }

            if ( m_eCommand == intake::eCommand::COMMAND_STOP || bIsTimedOut == true || /* Lower limit hit*/ )
            {
                // Stop arm motors

                // Enable brake mode

                // Reset state and command
                m_eState = intake::eState::STATE_IDLE;
                m_eCommand = intake::eCommand::COMMAND_NONE;
            }
        }

        // ********************
        // * Auto Raise State *
        // ********************
        else if ( m_eState == intake::eState::STATE_AUTO_RAISE )
        {
            // Check timeout timer
            bool bIsTimedOut = false;
            if ( (double)m_pTimeoutTimer->Get() >= intake::dAutoRaiseTimeout )
            {
                bIsTimedOut = true;
            }

            if ( m_eCommand == intake::eCommand::COMMAND_STOP || bIsTimedOut == true || /* Upper limit hit*/ )
            {
                // Stop arm motors

                // Enable brake mode

                // Reset state and command
                m_eState = intake::eState::STATE_IDLE;
                m_eCommand = intake::eCommand::COMMAND_NONE;
            }
        }

        // ********************
        // * Auto Lower State *
        // ********************
        else if ( m_eState == intake::eState::STATE_AUTO_LOWER )
        {
            // Check timeout timer
            bool bIsTimedOut = false;
            if ( (double)m_pTimeoutTimer->Get() >= intake::dAutoLowerTimeout )
            {
                bIsTimedOut = true;
            }

            if ( m_eCommand == intake::eCommand::COMMAND_STOP || bIsTimedOut == true || /* Lower limit hit*/ )
            {
                // Stop arm motors

                // Enable brake mode

                // Reset state and command
                m_eState = intake::eState::STATE_IDLE;
                m_eCommand = intake::eCommand::COMMAND_NONE;
            }
        }

        // ***********************
        // * Manual Intake State *
        // ***********************
        else if ( m_eState == intake::eState::STATE_MANUAL_INTAKE )
        {
            // Check timeout timer
            bool bIsTimedOut = false;
            if ( (double)m_pTimeoutTimer->Get() >= intake::dManualIntakeTimeout )
            {
                bIsTimedOut = true;
            }

            if ( m_eCommand == intake::eCommand::COMMAND_STOP || bIsTimedOut == true )
            {
                // Stop intake/outtake motors

                // Reset state and command
                m_eState = intake::eState::STATE_IDLE;
                m_eCommand = intake::eCommand::COMMAND_NONE;
            }
        }

        // ************************
        // * Manual Outtake State *
        // ************************
        else if ( m_eState == intake::eState::STATE_MANUAL_OUTTAKE )
        {
            // Check timeout timer
            bool bIsTimedOut = false;
            if ( (double)m_pTimeoutTimer->Get() >= intake::dManualOuttakeTimeout )
            {
                bIsTimedOut = true;
            }

            if ( m_eCommand == intake::eCommand::COMMAND_STOP || bIsTimedOut == true )
            {
                // Stop intake/outtake motors

                // Reset state and command
                m_eState = intake::eState::STATE_IDLE;
                m_eCommand = intake::eCommand::COMMAND_NONE;
            }
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