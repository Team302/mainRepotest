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

#pragma once

//C++ Includes
#include <vector>

//Team 302 Includes
#include <states/IState.h>
#include <states/ballhopper/BallHopperSlowRelease.h>

class BallHopperStateMgr
{
    public:
        /// @enum the states of BallHopper
        enum BALL_HOPPER_STATE
        {
            OFF,
            HOLD,
            RAPID_RELEASE,
            SLOW_RELEASE,
            MAX_BALL_HOPPER_STATES
        };

        /// @brief Find or create the state manager
        /// @return BallHopperStateMgr* pointer to the state manager
        static BallHopperStateMgr* GetInstance();

        /// @brief run the current state
        /// @param [in] BALL_HOPPER_STATE - state to run
        /// @return void
        void RunCurrentState();

        /// @brief set the current state, initialize it and run it
        /// @param [in]     BALL_HOPPER_STATE - state to set
        /// @param [in]     run - true mean run, false just initialize
        /// @return void
        void SetCurrentState
        (
            BALL_HOPPER_STATE   state,
            bool                run
        );

        /// @brief return the current state
        /// @return BALL_HOPPER_STATE - the current state
        inline BALL_HOPPER_STATE GetCurrentState() const { return m_currentStateEnum; };

        inline IState* GetState( BALL_HOPPER_STATE state ) { return m_stateVector[state];}
        

    private:

        IState* m_currentState;
        IState* m_holdState;
        IState* m_releaseState;

        std::vector<IState*> m_stateVector;
        BALL_HOPPER_STATE m_currentStateEnum;

        BallHopperStateMgr();
        ~BallHopperStateMgr() = default;

        bool m_isSlowRelease = false;

        static BallHopperStateMgr*  m_instance;
};