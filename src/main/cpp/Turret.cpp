#include "Turret.h"

// Constructor
Turret::Turret()
{
    m_eState = turret::eState::STATE_START;
    m_eCommand = turret::COMMAND_NONE;

    m_pRobotIO = nullptr;
    m_pTurretSetpoint = nullptr;
}

void Turret::Initialize( RobotIO *p_pRobotIO, double *p_pTurretSetpoint )
{
    m_pRobotIO = p_pRobotIO;
    m_pTurretSetpoint = p_pTurretSetpoint;

    m_pTimeoutTimer = new frc::Timer();
    m_pTimeoutTimer->Reset();

    m_Request.WithSlot(0);
}

void Turret::UpdateInputStatus()
{

}

void Turret::Execute()
{
    //Check if m_pRobotIO or m_pTurretSetpoint has been assigned
    if(m_pRobotIO != nullptr && m_pTurretSetpoint != nullptr)
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
                //Homing logic
            }

            // *---------------------*
            // * Manual Left Command *
            // *---------------------*
            else if(m_eCommand == turret::COMMAND_MANUAL_LEFT)
            {
                //Manual Left logic
            }

            // *----------------------*
            // * Manual Right Command *
            // *----------------------*
            else if(m_eCommand == turret::COMMAND_MANUAL_RIGHT)
            {
                //Manual right logic
            }

            // *-------------------*
            // * Auto Move Command *
            // *-------------------*
            else if(m_eCommand == turret::COMMAND_AUTO_MOVE)
            {
                //Auto move logic - motion magic with m_pTurretSetpoint
                //To access m_pTurretSetpoint, use the following notation:
                *m_pTurretSetpoint;

                //Brief explanation: MainStateMachine will have a method of calculating a desired turret angle
                // (limelight/odometry/hardcoded) that it will store as m_dTurretTargetPosition. This class will
                // then have a pointer to that double that it will utilize (read only, can't be const due to nullptr
                // at construction) to aim the turret. Therefore, multiple commands are not needed for various 
                // setpoints, and this will allow for potential future expansion of aiming directly at the hub. - GMS
                
                // We can make it a const pointer if we declare it at construction rather than in Initialize, however
                // this may be beyond the scope of what we need.
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
            //Homing state logic
        }

        //Will we need debounce?

        
        // *********************
        // * Manual Left State *
        // *********************
        else if(m_eState == turret::eState::STATE_MANUAL_LEFT)
        {
            //Manual left state logic
        }

        // **********************
        // * Manual Right State *
        // **********************
        else if(m_eState == turret::eState::STATE_MANUAL_LEFT)
        {
            //Manual left state logic
        }

        // *******************
        // * Auto Move State *
        // *******************
        else if(m_eState == turret::eState::STATE_MANUAL_LEFT)
        {
            //Auto Move logic using motion magic
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