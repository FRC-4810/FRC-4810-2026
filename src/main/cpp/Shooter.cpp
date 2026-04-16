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
        
        if ( m_eState == shooter::eState::STATE_IDLE )
        {
            // *-------------------------*
            // * Low Power Shoot Command *
            // *-------------------------*

            if ( m_eCommand == shooter::eCommand::COMMAND_LOW_POWER_SHOOT )
            {

                // Set speed on shooter motors
                m_pRobotIO->m_LeftShooterMotor_Master.Set( shooter::dLowPowerRampUpSpeed );

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to low power ramp up
                m_eState = shooter::eState::STATE_LOW_POWER_RAMP_UP;

            }
            // *--------------------------*
            // * High Power Shoot Command *
            // *--------------------------*

            else if ( m_eCommand == shooter::eCommand::COMMAND_HIGH_POWER_SHOOT )
            {

                // Set speed on shooter motors
                m_pRobotIO->m_LeftShooterMotor_Master.Set( shooter::dHighPowerRampUpSpeed );

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to high power ramp up
                m_eState = shooter::eState::STATE_HIGH_POWER_RAMP_UP;

            }

            // *---------------------------*
            // * Auton Power Shoot Command *
            // *---------------------------*

            else if ( m_eCommand == shooter::eCommand::COMMAND_AUTON_SHOOT )
            {
                // Set speed on shooter motors using auton-specific constant
                m_pRobotIO->m_LeftShooterMotor_Master.Set( shooter::dAutonPowerRampUpSpeed );

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to auton power ramp up
                m_eState = shooter::eState::STATE_AUTON_POWER_RAMP_UP;
            }

            // Unrecognized command

            //else if ( m_eCommand != shooter::eCommand::COMMAND_NONE || m_eCommand != shooter::eCommand::COMMAND_STOP ) < -GMS This should be AND, not OR
            else if ( m_eCommand != shooter::eCommand::COMMAND_NONE && m_eCommand != shooter::eCommand::COMMAND_STOP )
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
                m_pRobotIO->m_LeftShooterMotor_Master.Set( 0 );

                // Reset State and Command
                m_eState = shooter::eState::STATE_IDLE;
                m_eCommand = shooter::eCommand::COMMAND_NONE;
            }

            // *------------------------*
            // * Motor Velocity Reached *
            // *------------------------*

            else if ( m_pRobotIO->GetShooterSpeed()  >= shooter::dLowPowerVelocitySetpoint )
            {
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
                m_pRobotIO->m_LeftShooterMotor_Master.Set( 0 );

                // Reset State and Command
                m_eState = shooter::eState::STATE_IDLE;
                m_eCommand = shooter::eCommand::COMMAND_NONE;
            }

            // *------------------------*
            // * Motor Velocity Reached *
            // *------------------------*

            else if ( m_pRobotIO->GetShooterSpeed()  >= shooter::dHighPowerVelocitySetpoint )
            {
                // Transition to shoot state
                m_eState = shooter::eState::STATE_SHOOT;
            }
        }

        // ****************************
        // * Auton Power Ramp Up State *
        // ****************************
        
        else if ( m_eState == shooter::eState::STATE_AUTON_POWER_RAMP_UP )
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
                m_pRobotIO->m_LeftShooterMotor_Master.Set( 0 );

                // Reset State and Command
                m_eState = shooter::eState::STATE_IDLE;
                m_eCommand = shooter::eCommand::COMMAND_NONE;
            }

            // *------------------------*
            // * Motor Velocity Reached *
            // *------------------------*

            else if ( m_pRobotIO->GetShooterSpeed()  >= shooter::dAutonPowerVelocitySetpoint )
            {
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
            if ( (double)m_pTimeoutTimer->Get() >= shooter::dShootTimeout )
            {
                bIsTimedOut = true;
            }

            if ( m_eCommand == shooter::eCommand::COMMAND_STOP || bIsTimedOut == true )
            {
                /// Stop shooter motors
                m_pRobotIO->m_LeftShooterMotor_Master.Set( 0 );

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
    //    printf("Shooter.cpp: m_pRobotIO is nullptr\n");
    }
}
