#include "Shooter.h"

// Constructor
Shooter::Shooter()
{
    m_eState = shooter::eState::STATE_START;
    m_eCommand = shooter::eCommand::COMMAND_NONE;

    m_pRobotIO = nullptr;
}

// Initialize Shooter
void Shooter::Initialize( RobotIO *p_pRobotIO )
{
    m_pRobotIO = p_pRobotIO;

    m_pTimeoutTimer = new frc::Timer();
    m_pTimeoutTimer->Reset();

}

void Shooter::Execute()
{
    // Check that m_pRobotIO has been assigned
    if ( m_pRobotIO != nullptr )
    {
        // ***************
        // * Start State *
        // ***************

        if ( m_eState == shooter::eState::STATE_START )
        {
            // Go to idle state
            m_eState = shooter::eState::STATE_IDLE;
        }

        // **************
        // * Idle State *
        // **************
        
        else if ( m_eState == shooter::eState::STATE_IDLE )
        {
            // *-------------------------*
            // * Low Power Shoot Command *
            // *-------------------------*

            if ( m_eCommand == shooter::eCommand::COMMAND_LOW_POWER_SHOOT )
            {
                // Enable coast mode on shooter motors

                // Set speed on shooter motors

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to shoot
                m_eState = shooter::eState::STATE_LOW_POWER_RAMP_UP;

            }
            // *--------------------------*
            // * High Power Shoot Command *
            // *--------------------------*

            else if ( m_eCommand == shooter::eCommand::COMMAND_HIGH_POWER_SHOOT )
            {
                // Enable coast mode on shooter motors

                // Set speed on shooter motors

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to shoot
                m_eState = shooter::eState::STATE_HIGH_POWER_RAMP_UP;

            }

            // Unrecognized command

            else if ( m_eCommand != shooter::eCommand::COMMAND_NONE || m_eCommand != shooter::eCommand::COMMAND_STOP )
            {
                printf("Shooter.cpp: unrecognized command\n");
            }
        }

        // ***************************
        // * Low Power Ramp Up State *
        // ***************************
        
        else if ( m_eState == shooter::eState::STATE_LOW_POWER_RAMP_UP )
        {
            // Check Timeout Timer
            bool bIsTimedOut = false;
            if ( (double)m_pTimeoutTimer->Get() >= shooter::dRampUpTimeout )
            {
                bIsTimedOut = true;
            }

            if ( m_eCommand == shooter::eCommand::COMMAND_STOP || bIsTimedOut == true )
            {
                // Stop shooter motors

                // Enable brake mode

                // Reset State and Command
                m_eState = shooter::eState::STATE_IDLE;
                m_eCommand = shooter::eCommand::COMMAND_NONE;
            }

            // *------------------------*
            // * Motor Velocity Reached *
            // *------------------------*

            else if ( /*Motor Velocity*/  >= shooter::dLowPowerVelocitySetpoint )
            {
                // Enable coast mode on feeder motors

                // Set speed on feeder motors

                // Transition to shoot state
                m_eState = shooter::eState::STATE_SHOOT;
            }

        }

        // ****************************
        // * High Power Ramp Up State *
        // ****************************
        
        else if ( m_eState == shooter::eState::STATE_HIGH_POWER_RAMP_UP )
        {
            // Check Timeout Timer
            bool bIsTimedOut = false;
            if ( (double)m_pTimeoutTimer->Get() >= shooter::dRampUpTimeout )
            {
                bIsTimedOut = true;
            }

            if ( m_eCommand == shooter::eCommand::COMMAND_STOP || bIsTimedOut == true )
            {
                // Stop shooter motors

                // Enable brake mode

                // Reset State and Command
                m_eState = shooter::eState::STATE_IDLE;
                m_eCommand = shooter::eCommand::COMMAND_NONE;
            }

            // *------------------------*
            // * Motor Velocity Reached *
            // *------------------------*

            else if ( /*Motor Velocity*/  >= shooter::dHighPowerVelocitySetpoint )
            {
                // Enable coast mode on feeder motors

                // Set speed on feeder motors

                // Transition to shoot state
                m_eState = shooter::eState::STATE_SHOOT;
            }
        }

        // ***************
        // * Shoot State *
        // ***************
        
        else if ( m_eState == shooter::eState::STATE_SHOOT )
        {
            // Check Timeout Timer
            bool bIsTimedOut = false;
            if ( (double)m_pTimeoutTimer->Get() >= shooter::dRampUpTimeout )
            {
                bIsTimedOut = true;
            }

            if ( m_eCommand == shooter::eCommand::COMMAND_STOP || bIsTimedOut == true )
            {
                // Stop shooter and feeder motors

                // Enable brake mode

                // Reset State and Command
                m_eState = shooter::eState::STATE_IDLE;
                m_eCommand = shooter::eCommand::COMMAND_NONE;
            }
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