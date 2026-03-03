#include "Turret.h"

// Constructor
Turret::Turret()
{
    m_eState = turret::eState::STATE_START;
    m_eCommand = turret::COMMAND_NONE;
    m_dTurretSetpoint = 0.0;

    m_pRobotIO = nullptr;
    m_pDrivetrain = nullptr;
}

void Turret::Initialize( RobotIO *p_pRobotIO, Drivetrain *p_pDrivetrain )
{
    m_pRobotIO = p_pRobotIO;
    m_pDrivetrain = p_pDrivetrain;

    m_pTimeoutTimer = new frc::Timer();
    m_pTimeoutTimer->Reset();

    m_Request.WithSlot(0);

    m_pRobotIO->m_TurretMotor.GetConfigurator().Refresh(m_MotorConfigs);
}

void Turret::UpdateInputStatus()
{

}

void Turret::Execute()
{
    //Check if m_pRobotIO or m_pTurretSetpoint has been assigned
    if(m_pRobotIO != nullptr && m_pDrivetrain != nullptr)
    {
        // ***************
        // * Start State *
        // ***************
        if(m_eState == turret::eState::STATE_START)
        {
            m_eState = turret::eState::STATE_IDLE;
        }

        // **************
        // * Idle State *
        // **************
        if(m_eState == turret::eState::STATE_IDLE)
        {
            // *--------------*
            // * Home Command *
            // *--------------*
            if(m_eCommand == turret::COMMAND_HOME)
            {
                if(m_pRobotIO->IsTurretHomed())
                {
                    m_pRobotIO->m_TurretMotor.SetPosition(0_tr);
                    m_eCommand = turret::COMMAND_NONE;
                    return;
                }

                m_MotorConfigs.NeutralMode = signals::NeutralModeValue::Coast;
                m_pRobotIO->m_TurretMotor.GetConfigurator().Apply(m_MotorConfigs);
                
                m_pRobotIO->m_TurretMotor.Set(turret::dHomingSpeed);

                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                m_eState = turret::eState::STATE_HOMING;
            }

            // *---------------------*
            // * Manual Left Command *
            // *---------------------*
            else if(m_eCommand == turret::COMMAND_MANUAL_LEFT)
            {
                if(m_pRobotIO->IsTurretMax())
                {
                    m_eCommand = turret::COMMAND_NONE;
                    return;
                }

                m_MotorConfigs.NeutralMode = signals::NeutralModeValue::Coast;
                m_pRobotIO->m_TurretMotor.GetConfigurator().Apply(m_MotorConfigs);
                
                m_pRobotIO->m_TurretMotor.Set(turret::dManualLeftSpeed);

                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                m_eState = turret::eState::STATE_MANUAL_LEFT;
            }

            // *----------------------*
            // * Manual Right Command *
            // *----------------------*
            else if(m_eCommand == turret::COMMAND_MANUAL_RIGHT)
            {
                if(m_pRobotIO->IsTurretHomed())
                {
                    m_eCommand = turret::COMMAND_NONE;
                    return;
                }

                m_MotorConfigs.NeutralMode = signals::NeutralModeValue::Coast;
                m_pRobotIO->m_TurretMotor.GetConfigurator().Apply(m_MotorConfigs);
                
                m_pRobotIO->m_TurretMotor.Set(turret::dManualRightSpeed);

                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                m_eState = turret::eState::STATE_MANUAL_RIGHT;
            }

            // *-------------------*
            // * Auto Move Command *
            // *-------------------*
            else if(m_eCommand == turret::COMMAND_AUTO_MOVE)
            {
                if(m_dTurretSetpoint > 0 && m_dTurretSetpoint < turret::dMaxRotations)
                {
                    if(IsAtTarget())
                    {
                        m_eCommand = turret::COMMAND_NONE;
                        return;
                    }

                    m_pRobotIO->m_TurretMotor.SetControl(m_Request.WithPosition( units::angle::turn_t{m_dTurretSetpoint} ));

                    m_pTimeoutTimer->Reset();
                    m_pTimeoutTimer->Start();

                    m_eState = turret::eState::STATE_AUTO_MOVE;
                }
            }

            // Error - unrecognized command
            else if(m_eCommand != turret::COMMAND_NONE && m_eCommand != turret::COMMAND_STOP)
            {
                printf("ERROR in Turret.cpp - Unknown command\n");
            }
        }

        // ****************
        // * Homing State *
        // ****************
        else if(m_eState == turret::eState::STATE_HOMING)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= turret::dHomingTimeout)
            {
                bIsTimedOut = true;
            }

            if(m_eCommand == turret::COMMAND_STOP || bIsTimedOut)
            {
                m_pRobotIO->m_TurretMotor.Set(0);

                m_MotorConfigs.NeutralMode = signals::NeutralModeValue::Brake;
                m_pRobotIO->m_TurretMotor.GetConfigurator().Apply(m_MotorConfigs);

                m_eState = turret::eState::STATE_IDLE;
                m_eCommand = turret::COMMAND_NONE;
            }
            else if(m_pRobotIO->IsTurretHomed())
            {
                m_pRobotIO->m_TurretMotor.Set(0);

                m_MotorConfigs.NeutralMode = signals::NeutralModeValue::Brake;
                m_pRobotIO->m_TurretMotor.GetConfigurator().Apply(m_MotorConfigs);

                m_eState = turret::eState::STATE_DEBOUNCE;
            }
        }

        // ******************
        // * Debounce State *
        // ******************
        else if(m_eState == turret::eState::STATE_DEBOUNCE)
        {
            if((double)m_pTimeoutTimer->Get() >= turret::dDebounceTimeout)
            {
                m_pRobotIO->m_TurretMotor.SetPosition(0_tr);

                m_eState = turret::eState::STATE_IDLE;
                m_eCommand = turret::COMMAND_NONE;
            }
        }

        
        // *********************
        // * Manual Left State *
        // *********************
        else if(m_eState == turret::eState::STATE_MANUAL_LEFT)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= turret::dManualMoveTimeout)
            {
                bIsTimedOut = true;
            }

            if(m_eCommand == turret::COMMAND_STOP || bIsTimedOut || m_pRobotIO->IsTurretMax())
            {
                m_pRobotIO->m_TurretMotor.Set(0);

                m_MotorConfigs.NeutralMode = signals::NeutralModeValue::Brake;
                m_pRobotIO->m_TurretMotor.GetConfigurator().Apply(m_MotorConfigs);

                m_eState = turret::eState::STATE_IDLE;
                m_eCommand = turret::COMMAND_NONE;
            }
        }

        // **********************
        // * Manual Right State *
        // **********************
        else if(m_eState == turret::eState::STATE_MANUAL_RIGHT)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= turret::dManualMoveTimeout)
            {
                bIsTimedOut = true;
            }

            if(m_eCommand == turret::COMMAND_STOP || bIsTimedOut || m_pRobotIO->IsTurretHomed())
            {
                m_pRobotIO->m_TurretMotor.Set(0);

                m_MotorConfigs.NeutralMode = signals::NeutralModeValue::Brake;
                m_pRobotIO->m_TurretMotor.GetConfigurator().Apply(m_MotorConfigs);

                m_eState = turret::eState::STATE_IDLE;
                m_eCommand = turret::COMMAND_NONE;
            }
        }

        // *******************
        // * Auto Move State *
        // *******************
        else if(m_eState == turret::eState::STATE_AUTO_MOVE)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= turret::dAutoMoveTimeout)
            {
                bIsTimedOut = true;
            }

            if(IsAtTarget() || bIsTimedOut || m_eCommand == turret::COMMAND_STOP)
            {
                m_pRobotIO->m_TurretMotor.Set(0);

                m_MotorConfigs.NeutralMode = signals::NeutralModeValue::Brake;
                m_pRobotIO->m_TurretMotor.GetConfigurator().Apply(m_MotorConfigs);

                m_eState = turret::eState::STATE_IDLE;
                m_eCommand = turret::COMMAND_NONE;
            }
        }

        // ********************
        // * Follow Hub State *
        // ********************
        else if(m_eState == turret::eState::STATE_FOLLOW)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= turret::dFollowTimeout)
            {
                bIsTimedOut = true;
            }

            //Update Motor Control
            m_pRobotIO->m_TurretMotor.SetControl(m_Request.WithPosition(units::angle::turn_t{ GetTurretAngleRotations() }));

            if(bIsTimedOut || m_eCommand == turret::COMMAND_STOP)
            {
                m_pRobotIO->m_TurretMotor.Set(0);

                m_MotorConfigs.NeutralMode = signals::NeutralModeValue::Brake;
                m_pRobotIO->m_TurretMotor.GetConfigurator().Apply(m_MotorConfigs);

                m_eState = turret::eState::STATE_IDLE;
                m_eCommand = turret::COMMAND_NONE;
            }
        }

        // Handle Error State or unknown state
        else
        {
            // Error Logic Here if needed
            printf("ERROR in Turret.cpp - Error or unknown state\n");
        }
    }
    else
    {
        // Handle RobotIO nullptr error
        printf("ERROR in Turret.cpp - Robot IO Pointer Null or Turret Setpoint Pointer Null\n");
    }
}

double Turret::GetTurretAngleRotations()
{
    //Todo Calculate turret target rotations based on drivetrain location

    

    return 0.0;
}