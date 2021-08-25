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
#include <memory>

// FRC Includes
#include <frc/Encoder.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include <frc/kinematics/SwerveModuleState.h>

#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <wpi/math>

// Team 302 Includes
#include <hw/DragonFalcon.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <subsys/PoseEstimatorEnum.h>

// Third Party Includes
#include <ctre/phoenix/sensors/CANCoder.h>


class SwerveModule 
{
    public:
        enum ModuleID
        {
            LEFT_FRONT,
            RIGHT_FRONT,
            LEFT_BACK,
            RIGHT_BACK
        };

        /// @brief Constructs a Swerve Module.  This is assuming 2 TalonFX (Falcons) with a CanCoder for the turn angle
        /// @param [in] ModuleID                                                type:           Which Swerve Module is it
        /// @param [in] shared_ptr<IDragonMotorController>                      driveMotor:     Motor that makes the robot move  
        /// @param [in] shared_ptr<IDragonMotorController>                      turnMotor:      Motor that turns the swerve module 
        /// @param [in] std::shared_ptr<ctre::phoenix::sensors::CANCoder>		canCoder:       Sensor for detecting the angle of the wheel
        SwerveModule( ModuleID                                                  type, 
                      std::shared_ptr<IDragonMotorController>                   driveMotor, 
                      std::shared_ptr<IDragonMotorController>                   turningMotor,
                      std::shared_ptr<ctre::phoenix::sensors::CANCoder>		    canCoder, 
                      double                                                    turnP,
                      double                                                    turnI,
                      double                                                    turnD,
                      double                                                    turnF,
                      double                                                    turnNominalPos,
                      double                                                    turnNominalNeg,
                      double                                                    turnMaxAcc,
                      double                                                    turnCruiseVel
                    );

        void Init
        (
            units::length::inch_t                                       wheelDiameter,
            units::velocity::meters_per_second_t                        maxVelocity,
            units::angular_velocity::radians_per_second_t               maxAngularVelocity,
            units::acceleration::meters_per_second_squared_t            maxAcceleration,
            units::angular_acceleration::radians_per_second_squared_t   maxAngularAcceleration,
            frc::Translation2d                                          offsetFromRobotCenter
        );
        
        /// @brief Turn all of the wheel to zero degrees yaw according to the pigeon
        /// @returns void
        void ZeroAlignModule();

        /// @brief Set all motor encoders to zero
        /// @returns void
        void SetEncodersToZero();

        ///@brief
        /// @returns
        double GetEncoderValues();

        /// @brief Get the current state of the module (speed of the wheel and angle of the wheel)
        /// @returns SwerveModuleState
        frc::SwerveModuleState GetState() const;

        /// @brief Set the current state of the module (speed of the wheel and angle of the wheel)
        /// @param [in] const SwerveModuleState& referenceState:   state to set the module to
        /// @returns void
        void SetDesiredState(const frc::SwerveModuleState& state);

        void RunCurrentState();

        /// @brief Return which module this is
        /// @returns ModuleID
        ModuleID GetType() {return m_type;}
        units::length::inch_t GetWheelDiameter() const {return m_wheelDiameter;}

        void SetDriveScale(double scale) { m_scale = scale; }
        void SetBoost(double boost) { m_boost=boost;}
        void SetBrake(double brake) { m_brake=brake;}

        void StopMotors();

        frc::Pose2d GetCurrentPose(PoseEstimationMethod opt);

        void UpdateCurrPose
        (
            units::length::meter_t  x,
            units::length::meter_t  y
        );
        
    private:
        // Note:  the following was taken from the WPI code and tweaked because we were seeing some weird 
        //        reversals that we believe was due to not using a tolerance
        frc::SwerveModuleState Optimize
        ( 
            const frc::SwerveModuleState& desiredState,
            const frc::Rotation2d& currentAngle
        );


        void SetDriveSpeed( units::velocity::meters_per_second_t speed );
        void SetTurnAngle( units::angle::degree_t angle );


        ModuleID                                            m_type;

        std::shared_ptr<IDragonMotorController>             m_driveMotor;
        std::shared_ptr<IDragonMotorController>             m_turnMotor;
        std::shared_ptr<ctre::phoenix::sensors::CANCoder>   m_turnSensor;

        units::length::inch_t                               m_wheelDiameter;

        std::shared_ptr<nt::NetworkTable>                   m_nt;     

        frc::SwerveModuleState                              m_activeState;
        frc::Pose2d                                         m_currentPose;
        units::angular_velocity::revolutions_per_minute_t   m_currentSpeed;
        double                                              m_currentRotations;
        //frc2::Timer                                         m_timer;



        units::velocity::meters_per_second_t                m_maxVelocity;

        double                                              m_scale;
        double                                              m_boost;
        double                                              m_brake;
        bool                                                m_runClosedLoopDrive;
};