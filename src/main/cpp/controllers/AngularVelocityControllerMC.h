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

// C++ includes
#include <units/angular_velocity.h>

// Team 302 includes
#include <controllers/ControlData.h>
#include <hw/interfaces/IDragonMotorController.h>

class AngularVelocityControllerMC 
{
    public:
        AngularVelocityControllerMC() = delete;
        AngularVelocityControllerMC
        (
            std::shared_ptr<IDragonMotorController>             motor,
            units::angular_velocity::revolutions_per_minute_t   target,
            std::shared_ptr<ControlData>                        controlData
        );

        ~AngularVelocityControllerMC() = default;

        void Init();
        void UpdateTarget
        (
            units::angular_velocity::revolutions_per_minute_t   target
        );

        void Run();

        units::angular_velocity::revolutions_per_minute_t GetAngularVelocity();

    private:
        std::shared_ptr<IDragonMotorController>             m_motor;
        units::angular_velocity::revolutions_per_minute_t   m_target;
        std::shared_ptr<ControlData>                        m_control;
        double                                              m_unitsPer100MS;



};