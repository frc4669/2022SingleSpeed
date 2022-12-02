// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/geometry/Transform2d.h>
#include <units/units.h>

#include <photonlib/PhotonUtils.h>
#include <photonlib/PhotonCamera.h>

#include <frc/geometry/Pose2d.h>

using namespace photonlib; 

class Vision : public frc2::SubsystemBase {
 public:
  Vision();

  struct TrackingInfo {

    PhotonTrackedTarget target;

    // transformation to go to the target
    frc::Transform3d pose;

    /** as the ball gets closer, the % that it takes up is going to increase, can be used to 
     *  estimate the relative distance of the target from the robot 
     * 
     *  NOTE: The percentage is going to instantly decrease once the ball gets too close
    **/ 
    units::percent_t cameraSpaceTaken = 0;

    // are we sure the info's correct, ie: is ambiguity above 0.2 or not
    bool isCertain = false;
  };

  // returns data of an tracked target
  TrackingInfo GetTargetTrackingInfo();

  // returns the best target that's tracked, nullptr if it doesn't exist
  std::unique_ptr<PhotonTrackedTarget> GetBestTarget(); 

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:

  // cameras
  PhotonCamera m_frontCamera { "m_frontCamera" }; 

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
