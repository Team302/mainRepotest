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

//C++ Includes
#include <algorithm>
#include <memory>

//FRC includes
#include <units/velocity.h>
#include <units/angular_velocity.h>

//Team 302 Includes
#include <states/chassis/SwerveDrive.h>
#include <hw/DragonPigeon.h>
#include <team302/gamepad/IDragonGamePad.h>
#include <team302/gamepad/TeleopControl.h>
#include <states/IState.h>
#include <subsys/SwerveChassisFactory.h>
#include <hw/factories/PigeonFactory.h>
#include <utils/Logger.h>
#include <auton/shooterlevels/DriveToShooterLevel.h>


using namespace std;
using namespace team302::gamepad;

/// @brief initialize the object and validate the necessary items are not nullptrs
SwerveDrive::SwerveDrive() : IState(),
                             m_chassis( SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis() ),
                             m_controller( TeleopControl::GetInstance() ),
                             m_usePWLinearProfile(false),
                             m_lastUp(false),
                             m_lastDown(false),
                             m_shooterLevel(),
                             m_offset(frc::Translation2d())
                             //m_shooterLevel(new DriveToShooterLevel())
{
    if ( m_controller == nullptr )
    {
        Logger::GetLogger()->LogError( string("SwerveDrive::SwerveDrive"), string("TeleopControl is nullptr"));
    }

    if ( m_chassis.get() == nullptr )
    {
        Logger::GetLogger()->LogError( string("SwerveDrive::SwerveDrive"), string("Chassis is nullptr"));
    }
}

/// @brief initialize the profiles for the various gamepad inputs
/// @return void
void SwerveDrive::Init()
{
    auto controller = GetController();
    if ( controller != nullptr )
    {
        auto profile = (m_usePWLinearProfile) ? IDragonGamePad::AXIS_PROFILE::PIECEWISE_LINEAR :  IDragonGamePad::AXIS_PROFILE::CUBED; 
        controller->SetAxisProfile( TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE, profile );
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisScaleFactor(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE, -2.0);
        //controller->SetSlewRateLimiter(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE, 3.0);

        controller->SetAxisProfile( TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER, profile );
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisScaleFactor(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER, -2.0);
        //controller->SetSlewRateLimiter(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER, 3.0);

        controller->SetAxisProfile( TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE, profile );
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisScaleFactor(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE, 2.0);
        //controller->SetSlewRateLimiter(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE, 3.0);
        
        controller->SetAxisProfile( TeleopControl::FUNCTION_IDENTIFIER::DRIVE_TURBO, IDragonGamePad::AXIS_PROFILE::LINEAR );
        controller->SetAxisProfile( TeleopControl::FUNCTION_IDENTIFIER::DRIVE_BRAKE, IDragonGamePad::AXIS_PROFILE::LINEAR );

        m_chassis.get()->RunWPIAlgorithm(false);
   }
}



/// @brief calculate the output for the wheels on the chassis from the throttle and steer components
/// @return void
void SwerveDrive::Run( )
{
    double drive = 0.0;
    double steer = 0.0;
    double rotate = 0.0;
    auto controller = GetController();
    if ( controller != nullptr )
    {
        if ( controller->IsButtonPressed( TeleopControl::FUNCTION_IDENTIFIER::REZERO_PIGEON))
        {
            auto factory = PigeonFactory::GetFactory();
            auto m_pigeon = factory->GetPigeon();
            m_pigeon->ReZeroPigeon( 0, 0);
            m_chassis.get()->ZeroAlignSwerveModules();
            m_lastUp   = false;
            m_lastDown = false;
        }
        else if (controller->IsButtonPressed( TeleopControl::DRIVE_FULL))
        {
            //m_chassis->SetDriveScaleFactor(1.0);
            m_chassis->SetDriveScaleFactor(0.1);
            m_lastUp   = false;
            m_lastDown = false;
        }
        else if (controller->IsButtonPressed(TeleopControl::DRIVE_75PERCENT))
        {
            m_chassis->SetDriveScaleFactor(0.75);
            m_lastUp   = false;
            m_lastDown = false;
        }
        else if (controller->IsButtonPressed(TeleopControl::DRIVE_50PERCENT))
        {
            m_chassis->SetDriveScaleFactor(0.50);
            m_lastUp   = false;
            m_lastDown = false;
        }
        else if (controller->IsButtonPressed(TeleopControl::DRIVE_25PERCENT))
        {
            //m_chassis->SetDriveScaleFactor(0.25);
            m_chassis->SetDriveScaleFactor(0.35);
            m_lastUp   = false;
            m_lastDown = false;
        }
        else if (controller->IsButtonPressed(TeleopControl::DRIVE_SHIFT_UP))
        {
            if (!m_lastUp)
            {
                auto scale = m_chassis->GetScaleFactor();
                scale += 0.25;
                auto newscale = clamp(scale, 0.25, 1.0);
                m_chassis->SetDriveScaleFactor(newscale);
            }
            m_lastUp = true;
        }        
        else if (controller->IsButtonPressed(TeleopControl::DRIVE_SHIFT_DOWN))
        {
            if (!m_lastDown)
            {
                auto scale = m_chassis->GetScaleFactor();
                scale -= 0.25;
                auto newscale = clamp(scale, 0.25, 1.0);
                m_chassis->SetDriveScaleFactor(newscale);
            }
            m_lastDown = true;
        }
        else if (controller->IsButtonPressed(TeleopControl::AUTO_DRIVE_TO_YELLOW))
        {
            //Want to drive 172 inches backwards
            //m_shooterLevel->DriveToLevel(-172, 39.7);  //First arg is distance in inches, second is speed in inches per second 
            m_shooterLevel = new DriveToShooterLevel();
            m_shooterLevel->Init(-172, 39.7);
        }
        else if (controller->IsButtonPressed(TeleopControl::AUTO_DRIVE_TO_LOADING_ZONE))
        {
            //Want to drive 172 inches forwards
            //m_shooterLevel->DriveToLevel(172, 39.7);  //First arg is distance in inches, second is speed in inches per second
            m_shooterLevel = new DriveToShooterLevel();
            m_shooterLevel->Init(172, 39.7);
        }
        else if (controller->IsButtonPressed(TeleopControl::TURN_AROUND_FRONT_RIGHT))
        {
            //Offset L and W values in swerve module position calculations to turn around front right wheel
            //Each wheel is half of wheelbase and half of track away
            //FL = (L + Wheelbase W - Track)                  FR = (L + Wheelbase W + Track)
            //                                      Center = (L W)
            //BL = (L - Wheelbase W - Track)                  BR = (L - 5Wheelbase W + Track)
            double xOffset = 1;     //percent of wheel base to offset rotate point by
            double yOffset = 1;     //percent of track to offset rotate point by

            units::inch_t xOffsetInches = units::inch_t(xOffset * m_chassis->GetWheelBase().to<double>());
            units::inch_t yOffsetInches = units::inch_t(yOffset * m_chassis->GetTrack().to<double>());

            units::meter_t xOffsetMeters = units::meter_t(xOffsetInches);
            units::meter_t yOffsetMeters = units::meter_t(yOffsetInches);

            frc::Translation2d offset = frc::Translation2d(xOffsetMeters, yOffsetMeters);

            m_offset = offset;

            Logger::GetLogger()->ToNtTable("ATurnAbout", "Is A Pressed?", "True");

        }
        else
        {
            m_lastUp   = false;
            m_lastDown = false;

            //Debugging for TurnAboutPoint
            Logger::GetLogger()->ToNtTable("ATurnAbout", "Is A Pressed?", "False");

            //Auto shooter level driving logic
            if (m_shooterLevel != nullptr)
            {
                m_shooterLevel->Run();
                if(m_shooterLevel->IsDone())
                {
                    delete m_shooterLevel;
                }
            }
        }
        
        drive  = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE) ;
        steer  = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER);
        rotate = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE);
        rotate = abs(rotate)<0.3 ? 0.0 : rotate;

        auto boost = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::DRIVE_TURBO);
        boost *= 0.50;
        boost = clamp(boost, 0.0, 0.50);
        m_chassis->SetBoost(boost);

        auto brake = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::DRIVE_BRAKE);
        brake *= 0.25;
        brake = clamp(brake, 0.0, 0.25);
        m_chassis->SetBrake(brake);
    }

    /**
    Logger::GetLogger()->ToNtTable("Swerve Drive", "drive", drive);
    Logger::GetLogger()->ToNtTable("Swerve Drive", "steer", steer);
    Logger::GetLogger()->ToNtTable("Swerve Drive", "rotate", rotate);
    **/
   
    m_chassis.get()->Drive(drive, steer, rotate, true, m_offset);
}

/// @brief indicates that we are not at our target
/// @return bool
bool SwerveDrive::AtTarget() const
{
    return false;
}

