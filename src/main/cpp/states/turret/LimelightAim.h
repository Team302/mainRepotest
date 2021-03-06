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
#include <states/IState.h>
#include <states/Mech1MotorState.h>
#include <controllers/MechanismTargetData.h>
#include <hw/DragonLimelight.h>
#include <subsys/Turret.h>

class ControlData;


class LimelightAim : public IState
{
    public:
        LimelightAim
        (
            ControlData* controlData,
            double target
        );
        void Init() override;
        void Run() override;
        bool AtTarget() const override;
        void UpdateTarget( double target );
    private:
        std::unique_ptr<Mech1MotorState>    m_motorState;
        std::shared_ptr<Turret>             m_turret;
        //DragonLimelight*                    m_limelight;

        ControlData* m_controlData;
        bool m_atTarget;
        double m_target;
        double m_targetPosition;
        bool m_start;

        //const double m_min    = -0.237;
        //const double m_max    = 0.237;
        //const double m_min    = -42.0;
        //const double m_max    = 42.0;
        const double m_min    = -20.0;
        const double m_max    = 20.0;
        const double m_zero   = 0.0;
        const double m_increment = 1.0;
};
