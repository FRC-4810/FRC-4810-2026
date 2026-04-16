#include "Auton_04.h"

// Constructor
// Drivetrain pointer is an ugly hack by Cory so we don't have multiple instances of drivetrain floating around.
// We create a new odometry for each drivetrain which really screws with autos
Auton04::Auton04(subsystems::CommandSwerveDrivetrain * drivetrain, Intake *intake)
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

    // if(m_Drivetrain->Initialized() == false) {
    //     m_Drivetrain->Initialize( p_pRobotIO );
    // }
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
        if (m_pTimeoutTimer->Get() < 0.7_s) {
            /* Lower the intake */
            m_intake->Stop();
        }else if (m_pTimeoutTimer->Get() < 0.9_s) {
            m_intake->AutoLower();
        } else if (m_pTimeoutTimer->Get() < 6_s) {
            /* Run the intake between 2 and 10 seconds into auto */
            m_intake->ManualIntake();
        } else if (m_pTimeoutTimer->Get() < 6.5_s) { //Might Need to Change Was 9.5
            /* Stop running the intake */
            m_intake->Stop();
        }
        

        //Shooting logic - backwards because we are using greater rather than less
        if (m_pTimeoutTimer->Get() > 19.75_s) {
            // stop magazine and shooter after 6 seconds of shooting
            m_Magazine.Stop();
            m_Magazine.Execute();
            m_Shooter.Stop();
            m_Shooter.Execute();
        }
        else if (m_pTimeoutTimer->Get() > 18.0_s) {
            // Agitate intake 1.5s after shooting starts
            if(m_pRobotIO->IsIntakeLowered())
            {
                m_intake->Agitate();
            }
        }
        else if (m_pTimeoutTimer->Get() > 16.5_s) {
            // run magazine after shooter reaches speed
            m_Magazine.RunIn();
            m_Magazine.Execute();
        }
        else if (m_pTimeoutTimer->Get() > 16.0_s) {
            /* Run the shooter to score the balls */
            m_Shooter.AutonShoot();
            m_Shooter.Execute();
        }
        else if (m_pTimeoutTimer->Get() > 14.0_s) {
            m_intake->Stop();
        }
        else if (m_pTimeoutTimer->Get() > 12.5_s) {
            m_intake->ManualIntake();
        }
        else if (m_pTimeoutTimer->Get() > 12.0_s) { // This will now run
            m_intake->AutoLower();
        }
        else if (m_pTimeoutTimer->Get() > 9.5_s) {
            /* Stop Shooter*/
            m_Magazine.Stop();
            m_Magazine.Execute();
            m_Shooter.Stop();
            m_Shooter.Execute();
        }
        //TODO test agitate logic -- Fix statement order -- fixed statement order - SSP
        else if (m_pTimeoutTimer->Get() > 8_s) {
            // Agitate intake 1.5s after shooting starts
            if(m_pRobotIO->IsIntakeLowered())
            {
                m_intake->AutoRaise(); //Agitate to Auto Raise
                //don't need to call execute, done at bottom
            }
        }
        
        else if (m_pTimeoutTimer->Get() > 6.5_s) {
            // run magazine after shooter reaches speed
            m_Magazine.RunIn();
            m_Magazine.Execute();
        }
        else if (m_pTimeoutTimer->Get() > 6_s) {
            /* Run the shooter to score the balls */
            m_Shooter.AutonShoot();
            m_Shooter.Execute();
        }

        m_intake->Execute();



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
