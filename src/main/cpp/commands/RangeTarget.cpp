// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>

#include <photonlib/PhotonPipelineResult.h>
#include <photonlib/PhotonUtils.h>

#include "commands/RangeTarget.h"

using namespace photonlib;

RangeTarget::RangeTarget(Drivetrain* drivetrain, Vision* vision) : m_drivetrain(drivetrain), m_vision(vision) {
  AddRequirements({ drivetrain, vision });
}

// Called when the command is initially scheduled.
void RangeTarget::Initialize() {
  frc::SmartDashboard::PutData("Range PID", &m_controller);
}

// Called repeatedly when this Command is scheduled to run
void RangeTarget::Execute() {
  double forward;

  PhotonPipelineResult result = m_vision->GetPipelineResult();

  frc::SmartDashboard::PutNumber("Pitch", result.GetBestTarget().GetPitch());

  if (result.HasTargets()) {
    units::meter_t range = PhotonUtils::CalculateDistanceToTarget(
      Dimensions::kCameraHeight, Dimensions::kTargetHeight, Dimensions::kCameraPitch,
      units::degree_t(result.GetBestTarget().GetPitch())
    );

    frc::SmartDashboard::PutNumber("Range", range.value());

    forward = -m_controller.Calculate(range.value(), Dimensions::kDesiredRange.value());

    frc::SmartDashboard::PutNumber("Desired Fwd Velocity", forward);
  } else {
    forward = 0;

    frc::SmartDashboard::PutNumber("Range", -1);
  }

  m_drivetrain->CurvatureDrive(forward, 0);
}

// Called once the command ends or is interrupted.
void RangeTarget::End(bool interrupted) {}

// Returns true when the command should end.
bool RangeTarget::IsFinished() {
  return false;
}
