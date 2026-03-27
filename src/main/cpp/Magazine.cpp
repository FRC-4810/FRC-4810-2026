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

            if(m_eCommand == magazine::COMMAND_RUN_IN)
            {
                m_pRobotIO->m_FeederMotor.Set( magazine::dFeederMotorInSpeed );
                m_pRobotIO->m_KickerMotor.Set( magazine::dKickerMotorInSpeed );

                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                m_eState = magazine::eState::STATE_RUN_IN;
            }

            else if(m_eCommand == magazine::COMMAND_RUN_OUT)
            {
                m_pRobotIO->m_FeederMotor.Set( magazine::dFeederMotorOutSpeed );
                m_pRobotIO->m_KickerMotor.Set( magazine::dKickerMotorOutSpeed );

                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                m_eState = magazine::eState::STATE_RUN_OUT;
            }


            // Error - unrecognized command
            else if(m_eCommand != magazine::COMMAND_NONE && m_eCommand != magazine::COMMAND_STOP)
            {
                printf("ERROR in Magazine.cpp - Unknown command\n");
            }
        }

        else if(m_eState == magazine::eState::STATE_RUN_IN)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= magazine::dMagazineTimeout)
            {
                bIsTimedOut = true;
            }

            if(bIsTimedOut || m_eCommand == magazine::COMMAND_STOP)
            {
                m_pRobotIO->m_FeederMotor.Set( 0.0 );
                m_pRobotIO->m_KickerMotor.Set( 0.0 );

                m_eCommand = magazine::eCommand::COMMAND_NONE;
                m_eState = magazine::eState::STATE_IDLE;
            }
        }

        else if(m_eState == magazine::eState::STATE_RUN_OUT)
        {
            bool bIsTimedOut = false;
            if((double)m_pTimeoutTimer->Get() >= magazine::dMagazineTimeout)
            {
                bIsTimedOut = true;
            }

            if(bIsTimedOut || m_eCommand == magazine::COMMAND_STOP)
            {
                m_pRobotIO->m_FeederMotor.Set( 0.0 );
                m_pRobotIO->m_KickerMotor.Set( 0.0 );

                m_eCommand = magazine::eCommand::COMMAND_NONE;
                m_eState = magazine::eState::STATE_IDLE;
            }
        }

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