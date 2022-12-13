// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"

Vision::Vision() = default;


PhotonPipelineResult Vision::GetPipelineResult() {
    return m_frontCamera.GetLatestResult();
}

std::unique_ptr<PhotonTrackedTarget> Vision::GetBestTarget() {
    auto latest = m_frontCamera.GetLatestResult(); 
    if (!latest.HasTargets()) return nullptr;

    return std::make_unique<PhotonTrackedTarget>(latest.GetBestTarget()); 
}

// This method will be called once per scheduler run
void Vision::Periodic() {
    frc::SmartDashboard::PutBoolean("Target Detected", GetPipelineResult().HasTargets());
}
