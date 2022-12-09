// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignToTarget.h"

#include <photonlib/PhotonPipelineResult.h>

AlignToTarget::AlignToTarget(Drivetrain* drivetrain, Vision* vision) : m_drivetrain(drivetrain), m_vision(vision) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({ drivetrain, vision });
}

// Called when the command is initially scheduled.
void AlignToTarget::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AlignToTarget::Execute() {
  double rotation;

  photonlib::PhotonPipelineResult result = m_vision->GetPipelineResult();

  if (result.HasTargets()) {
    rotation = -m_controller.Calculate(result.GetBestTarget().GetYaw(), 0);
  } else {
    rotation = 0;
  }

  m_drivetrain->CurvatureDrive(0, rotation);
}

// Called once the command ends or is interrupted.
void AlignToTarget::End(bool interrupted) {}

// Returns true when the command should end.
bool AlignToTarget::IsFinished() {
  return false;
}
