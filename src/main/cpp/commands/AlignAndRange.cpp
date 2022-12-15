// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignAndRange.h"

AlignAndRange::AlignAndRange(Drivetrain* drivetrain, Vision* vision) : m_drivetrain(drivetrain), m_vision(vision) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({ drivetrain, vision });
}

// Called when the command is initially scheduled.
void AlignAndRange::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AlignAndRange::Execute() {
  double rotation;
  double forward;

  photonlib::PhotonPipelineResult result = m_vision->GetPipelineResult();

  if (result.HasTargets()) {
    units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
      Dimensions::kCameraHeight, Dimensions::kTargetHeight, Dimensions::kCameraPitch,
      units::degree_t(result.GetBestTarget().GetPitch())
    );
    
    rotation = -m_angularControl.Calculate(result.GetBestTarget().GetYaw(), 0);
    forward = -m_linearControl.Calculate(range.value(), Dimensions::kDesiredRange.value());
  } else {
    rotation = 0;
    forward = 0;
  }

  m_drivetrain->CurvatureDrive(forward, rotation);
}

// Called once the command ends or is interrupted.
void AlignAndRange::End(bool interrupted) {}

// Returns true when the command should end.
bool AlignAndRange::IsFinished() {
  return false;
}
