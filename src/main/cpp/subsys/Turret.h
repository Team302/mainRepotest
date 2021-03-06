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

// C++ Includes
#include <memory>
// FRC includes

// Team 302 includes
//#include <hw/DragonDigitalInput.h>
#include <subsys/Mech1IndMotor.h>

// Third Party Includes

class IDragonMotorController;

class Turret : public Mech1IndMotor
{
	public:
        /// @brief Create the Turret mechanism
        Turret() = delete;

        /// @brief Create the Turret mechanism
        /// @param [in] IDragonMotorController* the motor controller that will run the turret
        Turret
        (
            std::shared_ptr<IDragonMotorController>   motorController//,
//            std::shared_ptr<DragonDigitalInput>       minTurnSensor,
//            std::shared_ptr<DragonDigitalInput>       maxTurnSensor
        );

        /// @brief Destroy the object and free memory
        ~Turret() override = default;

        /// @brief update the output to the mechanism using the current controller and target value(s)
        /// @return void 
        //void Update() override;

    private:
//        std::shared_ptr<DragonDigitalInput>             m_min;
//        std::shared_ptr<DragonDigitalInput>             m_max;
};
