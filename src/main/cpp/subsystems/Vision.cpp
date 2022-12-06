// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"

Vision::Vision() = default;


Vision::TrackingInfo Vision::GetTargetTrackingInfo() {
    auto bestTarget = GetBestTarget(); 

    TrackingInfo targetInfo;  

    if (bestTarget == nullptr) return targetInfo; 

    targetInfo.target = *bestTarget;

    targetInfo.cameraSpaceTaken = bestTarget->GetArea(); 
    targetInfo.pose = bestTarget->GetBestCameraToTarget(); 

    // bestTarget.

    if ((-1 < bestTarget->GetPoseAmbiguity()) && (bestTarget->GetPoseAmbiguity() < 0.2)) targetInfo.isCertain = true; 
    
    return targetInfo; 
}

std::unique_ptr<PhotonTrackedTarget> Vision::GetBestTarget() {
    auto latest = m_frontCamera.GetLatestResult(); 
    if (!latest.HasTargets()) return nullptr;

    return std::make_unique<PhotonTrackedTarget>(latest.GetBestTarget()); 
}

// This method will be called once per scheduler run
void Vision::Periodic() {}
