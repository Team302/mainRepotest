//====================================================================================================================================================
// Copyright 2020 Lake Orion Robotics FIRST Team 302 
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE 
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes
#include <map>
#include <memory>
#include <vector>

// FRC includes

// Team 302 includes
#include <states/IState.h>
#include <states/intake/IntakeStateMgr.h>
#include <xmlmechdata/StateDataDefn.h>
#include <controllers/MechanismTargetData.h>
#include <utils/Logger.h>
#include <team302/gamepad/TeleopControl.h>
#include <states/intake/IntakeState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>


// Third Party Includes

using namespace std;
using namespace team302::gamepad;

IntakeStateMgr* IntakeStateMgr::m_instance = nullptr;
IntakeStateMgr* IntakeStateMgr::GetInstance()
{
	if ( IntakeStateMgr::m_instance == nullptr )
	{
		IntakeStateMgr::m_instance = new IntakeStateMgr();
	}
	return IntakeStateMgr::m_instance;
}

/// @brief    initialize the state manager, parse the configuration file and create the states.
IntakeStateMgr::IntakeStateMgr() : m_states(),
                                   m_currentState()
{
    // Parse the configuration file 
    auto stateXML = make_unique<StateDataDefn>();
    vector<MechanismTargetData*> targetData = stateXML.get()->ParseXML( MechanismTypes::MECHANISM_TYPE::INTAKE );

    // initialize the xml string to state map
    map<string, INTAKE_STATE> stateStringToEnumMap;
    stateStringToEnumMap["INTAKEOFF"] = INTAKE_STATE::OFF;
    stateStringToEnumMap["INTAKEON"]  = INTAKE_STATE::ON;

    for (auto inx=0; inx<MAX_INTAKE_STATES; ++inx)
    {
       m_states[inx] = nullptr;
    }
    
    // create the states passing the configuration data
    for ( auto td: targetData )
    {
        auto stateString = td->GetStateString();
        auto stateStringToEnumItr = stateStringToEnumMap.find( stateString );
        if ( stateStringToEnumItr != stateStringToEnumMap.end() )
        {
            auto stateEnum = stateStringToEnumItr->second;
            if ( m_states[stateEnum] == nullptr )
            {
                auto controlData = td->GetController();
                auto controlData2 = td->GetController2();
                auto target1 = td->GetTarget();
                auto target2 = td->GetSecondTarget();
                switch ( stateEnum )
                {
                    case INTAKE_STATE::ON:
                    {   
                        auto thisState = new IntakeState(controlData, controlData2, target1, target2);
                        m_states[stateEnum] = thisState;
                    }
                    break;

                    case INTAKE_STATE::OFF:
                    {   
                        auto thisState = new IntakeState(controlData, controlData2, target1, target2);
                        m_states[stateEnum] = thisState;
                        m_currentState = thisState;
                        m_currentStateEnum = stateEnum;
                        m_currentState->Init();
                    }
                    break;

                    default:
                    {
                        Logger::GetLogger()->LogError( string("IntakeStateMgr::IntakeStateMgr"), string("unknown state"));
                    }
                    break;
                }
            }
            else
            {
                Logger::GetLogger()->LogError( string("IntakeStateMgr::IntakeStateMgr"), string("multiple mechanism state info for state"));                
            }
        }
        else
        {
            Logger::GetLogger()->LogError( string("IntakeStateMgr::IntakeStateMgr"), string("state not found"));
        }
    }
}

/// @brief  run the current state
/// @return void
void IntakeStateMgr::RunCurrentState()
{
    if ( MechanismFactory::GetMechanismFactory()->GetIntake().get() != nullptr )
    {
            if ( m_currentStateEnum == INTAKE_STATE::ON)
            {
                SetCurrentState( INTAKE_STATE::ON, false );
            }
            else if ( m_currentStateEnum == INTAKE_STATE::OFF)
            {
                SetCurrentState( INTAKE_STATE::OFF, false );
            }

        Logger::GetLogger()->OnDash(string("Intake State"), to_string(m_currentStateEnum));

        // run the current state
        if ( m_currentState != nullptr )
        {
            m_currentState->Run();
        }
    }
}

/// @brief  set the current state, initialize it and run it
/// @return void
void IntakeStateMgr::SetCurrentState
(
    INTAKE_STATE    stateEnum,
    bool            run
)
{
    auto state = m_states[stateEnum];
    if ( state != nullptr && state != m_currentState )
    {
        m_currentState = state;
        m_currentStateEnum = stateEnum;
        m_currentState->Init();
        if ( run )
        {
            if ( MechanismFactory::GetMechanismFactory()->GetIntake().get() != nullptr )
            {
                m_currentState->Run();
            }
        }
        
    }
}


