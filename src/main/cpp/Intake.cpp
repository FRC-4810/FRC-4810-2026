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
    m_pRobotIO->m_IntakeMoveMotor.GetConfigurator().Refresh( m_MotorConfigs );
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
                // Check upper limit
                if ( m_pRobotIO->IsIntakeRaised() )
                {
                    m_eCommand = intake::eCommand::COMMAND_NONE;
                    return;
                }
                // Enable coast mode
                m_MotorConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
                m_pRobotIO->m_IntakeMoveMotor.GetConfigurator().Apply( m_MotorConfigs );

                // Set speed on arm motor
                m_pRobotIO->m_IntakeMoveMotor.Set( intake::dManualRaiseSpeed );

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to manual raise
                m_eState = intake::eState::STATE_MANUAL_RAISE;
            }

            // *----------------------*
            // * Manual Lower Command *
            // *----------------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_MANUAL_LOWER )
            {
                // Check lower limit
                if ( m_pRobotIO->IsIntakeLowered() )
                {
                    m_eCommand = intake::eCommand::COMMAND_NONE;
                    return;
                }
                // Enable coast mode
                m_MotorConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
                m_pRobotIO->m_IntakeMoveMotor.GetConfigurator().Apply( m_MotorConfigs );

                // Set speed on arm motor
                m_pRobotIO->m_IntakeMoveMotor.Set( intake::dManualLowerSpeed );

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to manual lower
                m_eState = intake::eState::STATE_MANUAL_LOWER;
            }

            // *--------------------*
            // * Auto Raise Command *
            // *--------------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_AUTO_RAISE )
            {
                // Check upper limit
                if ( m_pRobotIO->IsIntakeRaised() )
                {
                    m_eCommand = intake::eCommand::COMMAND_NONE;
                    return;
                }
                // Enable coast mode
                m_MotorConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
                m_pRobotIO->m_IntakeMoveMotor.GetConfigurator().Apply( m_MotorConfigs );

                // Set speed on arm motor
                m_pRobotIO->m_IntakeMoveMotor.Set( intake::dAutoRaiseSpeed );

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to auto raise
                m_eState = intake::eState::STATE_AUTO_RAISE;
            }

            // *--------------------*
            // * Auto Lower Command *
            // *--------------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_AUTO_LOWER )
            {
                // Check lower limit
                if ( m_pRobotIO->IsIntakeLowered() )
                {
                    m_eCommand = intake::eCommand::COMMAND_NONE;
                    return;
                }
                // Enable coast mode
                m_MotorConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
                m_pRobotIO->m_IntakeMoveMotor.GetConfigurator().Apply( m_MotorConfigs );

                // Set speed on arm motor
                m_pRobotIO->m_IntakeMoveMotor.Set( intake::dAutoLowerSpeed );

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to auto lower
                m_eState = intake::eState::STATE_AUTO_LOWER;
            }
            
            // *-----------------*
            // * Agitate Command *
            // *-----------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_AGITATE )
            {
                // Check if arm is above setpoint
                if ( m_pRobotIO->m_IntakeMoveMotor.GetPosition().GetValueAsDouble() <= intake::dCenterSetpoint ) 
                {
                    m_eCommand = intake::eCommand::COMMAND_NONE;
                    return;
                }

                // Enable coast mode
                m_MotorConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
                m_pRobotIO->m_IntakeMoveMotor.GetConfigurator().Apply( m_MotorConfigs );

                // Set speed on arm motor
                m_pRobotIO->m_IntakeMoveMotor.Set( intake::dAgitateSpeed );

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to agitate
                m_eState = intake::eState::STATE_AGITATE;
            }

            // *-----------------------*
            // * Manual Intake Command *
            // *-----------------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_MANUAL_INTAKE )
            {
                // Set speed on intake/outtake motor
                m_pRobotIO->m_IntakeRunMotor.Set( intake::dManualIntakeSpeed );

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to manual intake
                m_eState = intake::eState::STATE_MANUAL_INTAKE;
            }

            // *------------------------*
            // * Manual Outtake Command *
            // *------------------------*
            else if ( m_eCommand == intake::eCommand::COMMAND_MANUAL_OUTTAKE )
            {
                // Set speed on intake/outtake motor
                m_pRobotIO->m_IntakeRunMotor.Set( intake::dManualOuttakeSpeed );

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set state to manual outtake
                m_eState = intake::eState::STATE_MANUAL_OUTTAKE;
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
            bool bLimitHit = false;
            if ( m_pRobotIO->IsIntakeRaised() )
            {
                // Reset arm encoders
                //-GMS - don't reset until we have limits on it
                //m_pRobotIO->m_IntakeMoveMotor.SetPosition( units::angle::turn_t{0} );
                bLimitHit = true;
            }

            if ( m_eCommand == intake::eCommand::COMMAND_STOP || bIsTimedOut == true || bLimitHit == true )
            {
                // Stop arm motors
                m_pRobotIO->m_IntakeMoveMotor.Set( 0 );

                // Enable brake mode
                m_MotorConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
                m_pRobotIO->m_IntakeMoveMotor.GetConfigurator().Apply( m_MotorConfigs );

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
            bool bLimitHit = false;
            if ( m_pRobotIO->IsIntakeLowered() )
            {
                bLimitHit = true;
            }

            if ( m_eCommand == intake::eCommand::COMMAND_STOP || bIsTimedOut == true || bLimitHit == true )
            {
                // Stop arm motors
                m_pRobotIO->m_IntakeMoveMotor.Set( 0 );

                // Enable brake mode
                m_MotorConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
                m_pRobotIO->m_IntakeMoveMotor.GetConfigurator().Apply( m_MotorConfigs );

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

            bool bLimitHit = false;
            if ( m_pRobotIO->IsIntakeRaised() )
            {
                // Reset arm encoders
                //-GMS - don't reset until we have limits on it
                //m_pRobotIO->m_IntakeMoveMotor.SetPosition( units::angle::turn_t{0} );
                bLimitHit = true;
            }

            if ( m_eCommand == intake::eCommand::COMMAND_STOP || bIsTimedOut == true || bLimitHit == true )
            {
                // Stop arm motors
                m_pRobotIO->m_IntakeMoveMotor.Set( 0 );

                // Enable brake mode
                m_MotorConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
                m_pRobotIO->m_IntakeMoveMotor.GetConfigurator().Apply( m_MotorConfigs );

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

            bool bLimitHit = false;
            if ( m_pRobotIO->IsIntakeLowered() )
            {
                bLimitHit = true;
            }

            if ( m_eCommand == intake::eCommand::COMMAND_STOP || bIsTimedOut == true || bLimitHit == true )
            {
                // Stop arm motors
                m_pRobotIO->m_IntakeMoveMotor.Set( 0 );

                // Enable brake mode
                m_MotorConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
                m_pRobotIO->m_IntakeMoveMotor.GetConfigurator().Apply( m_MotorConfigs );

                // Reset state and command
                m_eState = intake::eState::STATE_IDLE;
                m_eCommand = intake::eCommand::COMMAND_NONE;
            }
        }

        // *****************
        // * Agitate State *
        // *****************
        else if ( m_eState == intake::eState::STATE_AGITATE )
        {
            // Check timeout timer
            bool bIsTimedOut = false;
            if ( (double)m_pTimeoutTimer->Get() >= intake::dAgitateTimeout )
            {
                bIsTimedOut = true;
            }

            bool bLimitHit = false;
            if ( m_pRobotIO->IsIntakeRaised() || m_pRobotIO->m_IntakeMoveMotor.GetPosition().GetValueAsDouble() <= intake::dCenterSetpoint  )
            {
                bLimitHit = true;
            }

            if ( m_eCommand == intake::eCommand::COMMAND_STOP || bIsTimedOut == true || bLimitHit == true )
            {
                // Stop arm motors
                m_pRobotIO->m_IntakeMoveMotor.Set( 0 );

                //-GMS - Do not transition into coast mode - let it fall

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
                m_pRobotIO->m_IntakeRunMotor.Set( 0 );

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
                m_pRobotIO->m_IntakeRunMotor.Set( 0 );

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