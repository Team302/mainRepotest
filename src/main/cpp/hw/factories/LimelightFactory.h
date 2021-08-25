
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

#include <memory>
#include <map>

#include <hw/DragonLimelight.h>
#include <hw/interfaces/IDragonSensor.h>


class LimelightFactory 
{
  public:
    static LimelightFactory* GetLimelightFactory();

    DragonLimelight* GetLimelight() const { return m_limelight;}

    DragonLimelight* CreateLimelight
    (
      std::string                     tableName,                  /// <I> - network table name
      units::length::inch_t           mountingHeight,             /// <I> - mounting height of the limelight
      units::length::inch_t           mountingHorizontalOffset,   /// <I> - mounting horizontal offset from the middle of the robot
      units::angle::degree_t          rotation,                   /// <I> - clockwise rotation of limelight
      units::angle::degree_t          mountingAngle,              /// <I> - mounting angle of the camera
      units::length::inch_t           targetHeight,               /// <I> - height the target
      units::length::inch_t           targetHeight2,              /// <I> - height of second target
      DragonLimelight::LED_MODE       ledMode,
      DragonLimelight::CAM_MODE       camMode,
      DragonLimelight::STREAM_MODE    streamMode,
      DragonLimelight::SNAPSHOT_MODE  snapMode,
      double                          defaultXHairX,
      double                          defaultXHairY,
      double                          secXHairX,
      double                          secXHairY
    );

  private:
    LimelightFactory();
    ~LimelightFactory() = default;

    static LimelightFactory* m_limelightFactory;
    DragonLimelight* m_limelight;
};
