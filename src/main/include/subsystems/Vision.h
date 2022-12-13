// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/units.h>

#include <photonlib/PhotonUtils.h>
#include <photonlib/PhotonCamera.h>

#include "Constants.h"

using namespace photonlib; 

class Vision : public frc2::SubsystemBase {
 public:
  Vision();

  PhotonPipelineResult GetPipelineResult();

  // returns the best target that's tracked, nullptr if it doesn't exist
  std::unique_ptr<PhotonTrackedTarget> GetBestTarget(); 

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  PhotonCamera m_frontCamera { "Front Camera" };

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
