//Generated From Template

#include "Magazine.h"

// Constructor
Magazine::Magazine()
{
    m_eState = magazine::eState::STATE_START;
    m_eCommand = magazine::COMMAND_NONE;

    m_pRobotIO = nullptr;
}

void Magazine::Initialize( RobotIO *p_pRobotIO )
{
    m_pRobotIO = p_pRobotIO;

    m_pTimeoutTimer = new frc::Timer();
    m_pTimeoutTimer->Reset();

    //TODO - Refresh Motor Configs if necessary
}

void Magazine::UpdateInputStatus()
{

}

void Magazine::Execute()
{
    //Check if m_pRobotIO has been assigned
    if(m_pRobotIO != nullptr)
    {
        // ***************
        // * Start State *
        // ***************
        if(m_eState == magazine::eState::STATE_START)
        {
            m_eState = magazine::eState::STATE_IDLE;
        }

        // **************
        // * Idle State *
        // **************
        if(m_eState == magazine::eState::STATE_IDLE)
        {

            //TODO - Add your command processing here



            // Error - unrecognized command
            else if(m_eCommand != magazine::COMMAND_NONE && m_eCommand != magazine::COMMAND_STOP)
            {
                printf("ERROR in Magazine.cpp - Unknown command\n");
            }
        }

        //TODO - Add your state processing here




        // Handle Error State or unknown state
        else
        {
            // Error Logic Here if needed
            printf("ERROR in Magazine.cpp - Error or unknown state\n");
        }
    }
    else
    {
        // Handle RobotIO nullptr error
        printf("ERROR in Magazine.cpp - Robot IO Pointer Null\n");
    }
}