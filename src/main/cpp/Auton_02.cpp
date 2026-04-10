#include "Auton_02.h"

// Constructor
Auton02::Auton02(subsystems::CommandSwerveDrivetrain *m_Drivetrain)
{
    m_eState = auton02::eState::STATE_START;

    m_pRobotIO = nullptr;
    this->m_Drivetrain = m_Drivetrain;
}

// Initialize Shooter
void Auton02::Initialize( RobotIO *p_pRobotIO )
{
    m_pRobotIO = p_pRobotIO;
    // m_Drivetrain->Initialize( p_pRobotIO );
    m_Intake.Initialize( p_pRobotIO );
    m_Magazine.Initialize( p_pRobotIO );
    m_Shooter.Initialize( p_pRobotIO );

    m_pTimeoutTimer = new frc::Timer();
    m_pTimeoutTimer->Reset();
}

void Auton02::Execute()
{
    // Check that m_pRobotIO has been assigned
    if ( m_pRobotIO != nullptr )
    {
        // ***************
        // * Start State *
        // ***************

        if ( m_eState == auton02::eState::STATE_START )
        {
            double rotSpeed = m_RotationPIDController.Calculate((double)m_Drivetrain->GetState().Pose.Rotation().Radians(), auton02::dTargetGyro);
            m_Drivetrain->SetControl(ctre::phoenix6::swerve::requests::RobotCentric{}.WithVelocityX(auton02::xMoveSpeed * 1_mps).WithVelocityY(auton02::yMoveSpeed * 1_mps).WithRotationalRate(rotSpeed * 1_tps));

            m_pTimeoutTimer->Reset();
            m_pTimeoutTimer->Start();

            m_eState = auton02::eState::STATE_MOVE_1;
        }

        // ****************
        // * Move 1 State *
        // ****************
        else if ( m_eState == auton02::eState::STATE_MOVE_1 )
        {
            // Standard Move Logic
            double rotSpeed = m_RotationPIDController.Calculate((double)m_Drivetrain->GetState().Pose.Rotation().Radians(), auton02::dTargetGyro);
            m_Drivetrain->SetControl(ctre::phoenix6::swerve::requests::FieldCentric{}.WithVelocityX(auton02::xMoveSpeed * 1_mps).WithVelocityY(auton02::yMoveSpeed * 1_mps).WithRotationalRate(rotSpeed * 1_tps));

            m_Intake.AutoLower();

            // Check if timed out
            if((double)m_pTimeoutTimer->Get() >= auton02::dMove1Timer)
            {
                m_Drivetrain->Stop();

                m_Shooter.LowPowerShoot();
                m_Shooter.Execute();

                m_eState = auton02::eState::STATE_RAMP_UP;
            }
        }

        // *****************
        // * Ramp Up State *
        // *****************
        else if ( m_eState == auton02::eState::STATE_RAMP_UP )
        {
            m_Shooter.Execute();

            if(m_Shooter.isShooting())
            {
                m_Magazine.RunIn();
                m_Magazine.Execute();
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();
                m_eState = auton02::eState::STATE_SHOOT;
            }
        }

        // ******************
        // * Shooting State *
        // ******************
        else if ( m_eState == auton02::eState::STATE_SHOOT )
        {
            if((double)m_pTimeoutTimer->Get() >= auton02::dShootTimer)
            {
                m_Shooter.Stop();
                m_Magazine.Stop();
            }

            m_Shooter.Execute();
            m_Magazine.Execute();

            if(m_Shooter.isIdle() && m_Magazine.IsIdle())
            {
                m_eState = auton02::eState::STATE_DONE;
            }
        }

        else if ( m_eState == auton02::eState::STATE_DONE )
        {
            
        }
        

        // Error or unkown state
        else
        {
            printf("Auton_02.cpp: Error state or unknown state\n");
        }
    }
    // m_pRobotIO is nullptr
    else
    {
     //   printf("Auton_02.cpp: m_pRobotIO is nullptr\n");
    }
}
