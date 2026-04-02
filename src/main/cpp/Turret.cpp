#include "Turret.h"

// Constructor
Turret::Turret()
{
    m_eState = turret::eState::STATE_START;
    m_eCommand = turret::eCommand::COMMAND_NONE;

    m_pRobotIO = nullptr;
    m_pDrivetrain = nullptr;

    m_dErrorSum = 0.0;
    m_dLastTime = 0.0;
}

// Initialize turret
void Turret::Initialize( RobotIO *p_pRobotIO, Drivetrain *p_pDrivetrain )
{
    m_pRobotIO = p_pRobotIO;
    m_pDrivetrain = p_pDrivetrain;

    m_pTimeoutTimer = new frc::Timer();
    m_pTimeoutTimer->Reset();

    m_pPIDTimer = new frc::Timer();
    m_pPIDTimer->Reset();
}

void Turret::Execute()
{
    // Check that m_pRobotIO and m_pDrivetrain have been asigned
    if ( m_pRobotIO != nullptr && m_pDrivetrain != nullptr )
    {
        // ***************
        // * Start State *
        // ***************
        if ( m_eState == turret::eState::STATE_START )
        {
            // Go to idle state
            m_eState == turret::eState::STATE_IDLE;
        }

        // **************
        // * Idle State *
        // **************
        if ( m_eState == turret::eState::STATE_IDLE )
        {
            // *----------------------*
            // * Manual Right Command *
            // *----------------------*
            if ( m_eCommand == turret::eCommand::COMMAND_MANUAL_RIGHT )
            {   
                // Check limit hit
                if ( m_pRobotIO->m_TurretRotationMotor.GetPosition().GetValueAsDouble() <= turret::dRightLimitSetpoint )
                {
                    m_eCommand = turret::eCommand::COMMAND_NONE;
                    return;
                }

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set speed on motors
                m_pRobotIO->m_TurretRotationMotor.Set( turret::dManualRightSpeed );

                // Set state to manual right
                m_eState = turret::eState::STATE_MANUAL_RIGHT;
            }
            
            // *---------------------*
            // * Manual Left Command *
            // *---------------------*
            else if ( m_eCommand == turret::eCommand::COMMAND_MANUAL_LEFT )
            {
                // Check limit hit
                if ( m_pRobotIO->m_TurretRotationMotor.GetPosition().GetValueAsDouble() >= turret::dLeftLimitSetpoint )
                {
                    m_eCommand = turret::eCommand::COMMAND_NONE;
                    return;
                }

                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set speed on motors
                m_pRobotIO->m_TurretRotationMotor.Set( turret::dManualLeftSpeed );

                // Set state to manual left
                m_eState = turret::eState::STATE_MANUAL_LEFT;
            }

            // *-------------------*
            // * Track Hub Command *
            // *-------------------*
            else if ( m_eCommand == turret::eCommand::COMMAND_TRACK_HUB )
            {
                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Reset PID Controller
                m_pPIDTimer->Reset();
                m_pPIDTimer->Start();

                m_dErrorSum = 0.0;
                m_dLastTime = 0.0;

                // Set state to track hub
                m_eState = turret::eState::STATE_TRACK_HUB;

            }

            // *----------------------*
            // * Track Corner Command *
            // *----------------------*
            else if ( m_eCommand == turret::eCommand::COMMAND_TRACK_CORNER )
            {
                // Reset timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Reset PID Controller
                m_pPIDTimer->Reset();
                m_pPIDTimer->Start();

                m_dErrorSum = 0.0;
                m_dLastTime = 0.0;

                // Set state to track corner
                m_eState = turret::eState::STATE_TRACK_CORNER;
            }
            // Handle unrecognized command error
            else if ( m_eCommand != turret::eCommand::COMMAND_NONE && m_eCommand != turret::eCommand::COMMAND_STOP )
            {
                printf("Turret.cpp: Unrecognized command");
            }
        }

        // **********************
        // * Manual Right State *
        // **********************        
        else if ( m_eState == turret::eState::STATE_MANUAL_RIGHT )
        {
            // Check timeout timer
            bool bIsTimedOut = false;
            if ( (double)m_pTimeoutTimer->Get() >= turret::dManualTurnTimeout )
            {
                bIsTimedOut = true;
            }

            // Check limit hit
            bool bLimitHit = false;
            if ( m_pRobotIO->m_TurretRotationMotor.GetPosition().GetValueAsDouble() <= turret::dRightLimitSetpoint )
            {
                bLimitHit = true;
            }

            if ( m_eCommand == turret::eCommand::COMMAND_STOP || bLimitHit == true || bIsTimedOut == true )
            {
                // Stop turn motors
                m_pRobotIO->m_TurretRotationMotor.Set( 0 );

                // Reset state and command
                m_eState = turret::eState::STATE_IDLE;
                m_eCommand = turret::eCommand::COMMAND_NONE;
            }
        }

        // *********************
        // * Manual Left State *
        // *********************        
        else if ( m_eState == turret::eState::STATE_MANUAL_LEFT )
        {
            // Check timeout timer
            bool bIsTimedOut = false;
            if ( (double)m_pTimeoutTimer->Get() >= turret::dManualTurnTimeout )
            {
                bIsTimedOut = true;
            }

            // Check limit hit
            bool bLimitHit = false;
            if ( m_pRobotIO->m_TurretRotationMotor.GetPosition().GetValueAsDouble() >= turret::dLeftLimitSetpoint )
            {
                bLimitHit = true;
            }

            if ( m_eCommand == turret::eCommand::COMMAND_STOP || bLimitHit == true || bIsTimedOut == true )
            {
                // Stop turn motors
                m_pRobotIO->m_TurretRotationMotor.Set( 0 );

                // Reset state and command
                m_eState = turret::eState::STATE_IDLE;
                m_eCommand = turret::eCommand::COMMAND_NONE;
            }
        }

        // *******************
        // * Track Hub State *
        // *******************        
        else if ( m_eState == turret::eState::STATE_TRACK_HUB )
        {
            // Check timeout timer
            bool bIsTimedOut = false;
            if ( (double)m_pTimeoutTimer->Get() >= turret::dTrackHubTimeout )
            {
                bIsTimedOut = true;
            }

            if ( m_eCommand == turret::eCommand::COMMAND_STOP || bIsTimedOut == true )
            {
                // Stop turn motors
                m_pRobotIO->m_TurretRotationMotor.Set( 0 );

                // Reset state and command
                m_eState = turret::eState::STATE_IDLE;
                m_eCommand = turret::eCommand::COMMAND_NONE;
            }
            // Tracking logic
            else
            {
                // Get bot position
                frc::Pose2d BotPose = m_pDrivetrain->GetBotPose();
                double dBotX = BotPose.X().value();
                double dBotY = BotPose.Y().value();
                double dBotAngle = BotPose.Rotation().Radians().value();

                // Get shooter position
                dShooterX = dBotX + ( turret::dShooterOffset * cos( dBotAngle ) );
                dShooterY = dBotY + ( turret::dShooterOffset * sin( dBotAngle ) );

                // Determine target position
                dTargetX = 0.0;
                dTargetY = 0.0;
                if ( frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ) // Blue alliance
                {
                    dTargetX = turret::dBlueHubX;
                    dTargetY = turret::dBlueHubY;
                }
                else // red alliance
                {
                    dTargetX = turret::dRedHubX;
                    dTargetY = turret::dRedHubY;
                }

                // Determine shooter angle needed (radians)
                double dXdistance = dTargetX - dShooterX;
                double dYdistance = dTargetY - dShooterY;
                double dTargetAngle = atan2( dYdistance, dXdistance ) + ( dBotAngle - std::numbers::pi ); // flip bot angle because shooter is at the back of the robot

                // Determine motor turn count for angle
                double dTargetAngleTurns = std::clamp( turret::dMotorRotationsPerPiRadians * ( dTargetAngle / std::numbers::pi ), turret::dRightLimitSetpoint, turret::dLeftLimitSetpoint );
                
                // Dont run if close enough to target
                if ( fabs( m_pRobotIO->m_TurretRotationMotor.GetPosition().GetValueAsDouble() - dTargetAngleTurns ) < 0.2 )
                {
                    // Stop Motors
                    m_pRobotIO->m_TurretRotationMotor.Set( 0 );
                }
                else
                {
                    // Get motor speed
                    double dMotorSpeed = -PIDOutput( dTargetAngleTurns, m_pRobotIO->m_TurretRotationMotor.GetPosition().GetValueAsDouble() ); // negated to make CCW positive
                    double dSpeed = std::clamp( dMotorSpeed, turret::dAutoRightMaxSpeed, turret::dAutoLeftMaxSpeed );

                    // Prevent Running past limits
                    if ( m_pRobotIO->m_TurretRotationMotor.GetPosition().GetValueAsDouble() >= turret::dLeftLimitSetpoint && dSpeed > 0 )
                    {
                        // Stop Motors
                        m_pRobotIO->m_TurretRotationMotor.Set( 0 );
                    }
                    else if ( m_pRobotIO->m_TurretRotationMotor.GetPosition().GetValueAsDouble() <= turret::dRightLimitSetpoint && dSpeed < 0 )
                    {
                        // Stop Motors
                        m_pRobotIO->m_TurretRotationMotor.Set( 0 );
                    }
                    else 
                    {
                        m_pRobotIO->m_TurretRotationMotor.Set( dSpeed );
                    }
                }                
            }
        }

        // **********************
        // * Track Corner State *
        // **********************        
        else if ( m_eState == turret::eState::STATE_TRACK_CORNER )
        {
            // Check timeout timer
            bool bIsTimedOut = false;
            if ( (double)m_pTimeoutTimer->Get() >= turret::dTrackCornerTimeout )
            {
                bIsTimedOut = true;
            }

            if ( m_eCommand == turret::eCommand::COMMAND_STOP || bIsTimedOut == true )
            {
                // Stop turn motors
                m_pRobotIO->m_TurretRotationMotor.Set( 0 );

                // Reset state and command
                m_eState = turret::eState::STATE_IDLE;
                m_eCommand = turret::eCommand::COMMAND_NONE;
            }
            // Tracking logic
            else
            {
                // Get bot position
                frc::Pose2d BotPose = m_pDrivetrain->GetBotPose();
                double dBotX = BotPose.X().value();
                double dBotY = BotPose.Y().value();
                double dBotAngle = BotPose.Rotation().Radians().value();

                // Get shooter position
                dShooterX = dBotX + ( turret::dShooterOffset * cos( dBotAngle ) );
                dShooterY = dBotY + ( turret::dShooterOffset * sin( dBotAngle ) );

                // Determine target position
                dTargetX = 0.0;
                dTargetY = 0.0;
                if ( frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ) // Blue alliance
                {
                    // Pick best corner
                    if ( dShooterY < turret::dCenterY )
                    {
                        dTargetX = turret::dLeftBlueCornerX;
                        dTargetY = turret::dLeftBlueCornerY;
                    }
                    else
                    {
                        dTargetX = turret::dRightBlueCornerX;
                        dTargetY = turret::dRightBlueCornerY;
                    }
                }
                else // red alliance
                {
                    // Pick best corner
                    if ( dShooterY < turret::dCenterY )
                    {
                        dTargetX = turret::dLeftRedCornerX;
                        dTargetY = turret::dLeftRedCornerY;
                    }
                    else
                    {
                        dTargetX = turret::dRightRedCornerX;
                        dTargetY = turret::dRightRedCornerY;
                    }
                }

                // Determine shooter angle needed (radians)
                double dXdistance = dTargetX - dShooterX;
                double dYdistance = dTargetY - dShooterY;
                double dTargetAngle = atan2( dYdistance, dXdistance ) + dBotAngle;

                // Determine motor turn count for angle
                double dTargetAngleTurns = std::clamp( turret::dMotorRotationsPerPiRadians * ( dTargetAngle / std::numbers::pi ), turret::dRightLimitSetpoint, turret::dLeftLimitSetpoint );
                
                // Dont run if close enough to target
                if ( fabs( m_pRobotIO->m_TurretRotationMotor.GetPosition().GetValueAsDouble() - dTargetAngleTurns ) < 0.2 )
                {
                    // Stop Motors
                    m_pRobotIO->m_TurretRotationMotor.Set( 0 );
                }
                else
                {
                    // Get motor speed
                    double dMotorSpeed = -PIDOutput( dTargetAngleTurns, m_pRobotIO->m_TurretRotationMotor.GetPosition().GetValueAsDouble() ); // negated to make CCW positive
                    double dSpeed = std::clamp( dMotorSpeed, turret::dAutoRightMaxSpeed, turret::dAutoLeftMaxSpeed );

                    // Prevent Running past limits
                    if ( m_pRobotIO->m_TurretRotationMotor.GetPosition().GetValueAsDouble() >= turret::dLeftLimitSetpoint && dSpeed > 0 )
                    {
                        // Stop Motors
                        m_pRobotIO->m_TurretRotationMotor.Set( 0 );
                    }
                    else if ( m_pRobotIO->m_TurretRotationMotor.GetPosition().GetValueAsDouble() <= turret::dRightLimitSetpoint && dSpeed < 0 )
                    {
                        // Stop Motors
                        m_pRobotIO->m_TurretRotationMotor.Set( 0 );
                    }
                    else 
                    {
                        m_pRobotIO->m_TurretRotationMotor.Set( dSpeed );
                    }
                }                
            }
        }
        // Handle unknown or error state
        else
        {
            printf("Turret.cpp: Unknown state or error state \n");
        }

    }
    // handle m_pRobotIO nullptr error
    else
    {
        printf("Turret.cpp: m_pRobotIO is nullptr \n");
    }
}

// Get Shooter Distance Accessor Method
double Turret::GetTargetDistance()
{
    double dXDistance = dTargetX - dShooterX;
    double dYDistance = dTargetY - dShooterY;

    double dShooterDistance = sqrt( ( dXDistance * dXDistance ) + ( dYDistance * dYDistance ) );

    return( dShooterDistance );
}


// PID Controller 
double Turret::PIDOutput( double dTarget, double dCurrent )
{
    // Proportional
    double dError = dTarget - dCurrent;

    double dProportional = turret::Kp * dError;

    //Integral
    double dt = (double)m_pPIDTimer->Get() - m_dLastTime;

    if ( dError < 2.0 )
    {
        m_dErrorSum += dError * std::clamp( dt, 0.0, 1.0 );
    }
    else
    {
        m_dErrorSum = 0.0;
    }
    
    double dIntegral = turret::Ki * m_dErrorSum;

    m_dLastTime = (double)m_pPIDTimer->Get();

    //Derivative
    double dE = dError - m_dLastError;

    double dDerivative = turret::Kd * ( dE / dt );

    m_dLastError = dError;

    return ( std::clamp( ( dProportional + dIntegral + dDerivative ), turret::dAutoLeftMaxSpeed, turret::dAutoRightMaxSpeed ) );
}