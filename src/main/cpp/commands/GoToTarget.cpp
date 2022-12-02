// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/GoToTarget.h"

GoToTarget::GoToTarget(
  Drivetrain* drivetrain, 
  Vision* vision
  ){
  AddRequirements({ drivetrain, vision }); 

  this->m_drivetrain = drivetrain; 
  this->m_vision = vision; 
}

// Called when the command is initially scheduled.
void GoToTarget::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void GoToTarget::Execute() {

  auto target = m_vision->GetTargetTrackingInfo(); 

  // checking if target exist
  if (target.cameraSpaceTaken.value() == 0) return;

  // converting 3d tracking to 2d translation and rotation
  auto translationToTarget = target.pose.Translation().ToTranslation2d(); 
  auto rotationToTarget = target.pose.Rotation().ToRotation2d();
  
  // generating a 2d target pose (not actually to go there, only for calculations)
  auto targetPose = frc::Pose2d(translationToTarget, rotationToTarget); 
  
  auto distance = units::meter_t(std::hypot(translationToTarget.X().value(), translationToTarget.Y().value()));
  
  auto linearVelocity = desieredVelocity; 
  // purpousely lowering speed 
  if (!target.isCertain) linearVelocity /= 2;

  auto estimatedTime = distance / linearVelocity;  

  auto angularVelocity = linearVelocity * rotationToTarget.Radians(); 

  // this's most likely not gonna work, it's more like a closed looped controller rn cuz it'll be reacting based on the 
  // next to impossible results of wished linear no movement of target & accurate info 

  auto chassieSpeeds = m_controller.Calculate(frc::Pose2d(), targetPose, linearVelocity, angularVelocity); 
  auto targetWheelSpeeds = m_kinematics.ToWheelSpeeds(chassieSpeeds); 

  m_output(targetWheelSpeeds.left, targetWheelSpeeds.right); 
}

// Called once the command ends or is interrupted.
void GoToTarget::End(bool interrupted) {}

// Returns true when the command should end.
bool GoToTarget::IsFinished() {
  return false;
}
