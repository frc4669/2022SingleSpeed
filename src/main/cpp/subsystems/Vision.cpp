// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"

Vision::Vision() = default;

PhotonPipelineResult Vision::GetPipelineResult() {
    return m_frontCamera.GetLatestResult();
}

// This method will be called once per scheduler run
void Vision::Periodic() {
    frc::SmartDashboard::PutBoolean("Target Detected", GetPipelineResult().HasTargets());
}
