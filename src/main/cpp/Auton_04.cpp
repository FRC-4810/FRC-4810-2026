#include "Auton_04.h"

// Constructor
// Drivetrain pointer is an ugly hack by Cory so we don't have multiple instances of drivetrain floating around.
// We create a new odometry for each drivetrain which really screws with autos
Auton04::Auton04(Drivetrain * drivetrain, Intake *intake)
{
    m_eState = auton04::eState::STATE_START;

    m_pRobotIO = nullptr;
    m_Drivetrain = drivetrain;
    m_intake = intake;

}

// Initialize Shooter
void Auton04::Initialize( RobotIO *p_pRobotIO )
{
    m_pRobotIO = p_pRobotIO;

    if(m_Drivetrain->Initialized() == false) {
        m_Drivetrain->Initialize( p_pRobotIO );
    }
    m_Magazine.Initialize( p_pRobotIO );
    m_Shooter.Initialize( p_pRobotIO );
    m_pTimeoutTimer = new frc::Timer();
    m_pTimeoutTimer->Reset();
    m_eState = auton04::eState::STATE_START;
}

void Auton04::Execute()
{
    // Check that m_pRobotIO has been assigned
    if ( m_pRobotIO != nullptr )
    {
        // ***************
        // * Start State *
        // ***************


        /* Cheap and dirty way to run other subsystems while the robot is following its path */
        if (m_pTimeoutTimer->Get() < 0.75_s) {
            /* Lower the intake */
            m_intake->Stop();
        }else if (m_pTimeoutTimer->Get() < 1.5_s) {
            m_intake->AutoLower();
        } else if (m_pTimeoutTimer->Get() < 10_s) {
            /* Run the intake between 2 and 10 seconds into auto */
            m_intake->ManualIntake();
        } else if (m_pTimeoutTimer->Get() < 11_s) {
            /* Stop running the intake */
            m_intake->Stop();
        }
        m_intake->Execute();
        

        //Shooting logic - backwards because we are using greater rather than less
        if (m_pTimeoutTimer->Get() > 18_s) {
            // stop magazine and shooter after 6 seconds of shooting
            m_Magazine.Stop();
            m_Magazine.Execute();
            m_Shooter.Stop();
            m_Shooter.Execute();
        }

        //TODO test agitate logic
        else if (m_pTimeoutTimer->Get() > 14_s) {
            // Agitate intake 1.5s after shooting starts
            if(m_pRobotIO->IsIntakeLowered())
            {
                m_intake->Agitate();
                //don't need to call execute, done at bottom
            }
        }
        
        else if (m_pTimeoutTimer->Get() > 10.5_s) {
            // run magazine after shooter reaches speed
            m_Magazine.RunIn();
            m_Magazine.Execute();
        }
        else if (m_pTimeoutTimer->Get() > 10_s) {
            /* Run the shooter to score the balls */
            m_Shooter.LowPowerShoot();
            m_Shooter.Execute();
        }   


        if ( m_eState == auton04::eState::STATE_START )
        {
            // Move path Logic
            m_Drivetrain->LoadPath( auton04::path1Name );
            m_Drivetrain->FollowPath();
            m_pTimeoutTimer->Reset();
            m_pTimeoutTimer->Start();

            m_eState = auton04::eState::STATE_MOVE_1;
        }

        // ****************
        // * Move 1 State *
        // ****************
        else if ( m_eState == auton04::eState::STATE_MOVE_1 )
        {

                // Move path Logic
                m_Drivetrain->FollowPath();

                // Check if path done
                if(m_Drivetrain->IsPathFinished() || (double)m_pTimeoutTimer->Get() >= auton04::dMove1Timer)
                {
                    m_Drivetrain->Stop();
                    m_eState = auton04::eState::STATE_DONE;
                }
            
        }

        else if ( m_eState == auton04::eState::STATE_DONE )
        {
            m_Drivetrain->Stop();
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
       // printf("Auton_01.cpp: m_pRobotIO is nullptr\n");
    }
}
