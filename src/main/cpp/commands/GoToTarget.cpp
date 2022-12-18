// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/GoToTarget.h"

GoToTarget::GoToTarget(Drivetrain *drivetrain, Vision *vision) 
: m_drivetrain(drivetrain), m_vision(vision), m_previousTime(-1_s), m_desiredVelocity(0.5_mps) {

  AddRequirements({ drivetrain, vision });
}

// Called when the command is initially scheduled.
void GoToTarget::Initialize() {
  m_previousTime = units::second_t(-1);

  m_previousSpeed = m_kinematics.ToWheelSpeeds(frc::ChassisSpeeds());

  m_leftControl.Reset();
  m_rightControl.Reset();

  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void GoToTarget::Execute() {
  auto currentTime = m_timer.Get();
  auto dt = currentTime - m_previousTime;
  m_previousTime = currentTime;

  photonlib::PhotonPipelineResult result = m_vision->GetPipelineResult();

  if (!result.HasTargets()) {
    m_drivetrain->TankDriveVolts(0_V, 0_V);
    m_previousSpeed = frc::DifferentialDriveWheelSpeeds();
    return; 
  }

  photonlib::PhotonTrackedTarget target = result.GetBestTarget();

  // calculating the position of the target relative to the robot

  frc::Rotation2d rotationToTarget(units::degree_t(-target.GetYaw()));
  frc::Translation2d translationToTarget(1_m, rotationToTarget);
  frc::Pose2d targetPose(translationToTarget, rotationToTarget);

  units::meter_t distance = photonlib::PhotonUtils::CalculateDistanceToTarget(
    Dimensions::kCameraHeight, Dimensions::kTargetHeight, Dimensions::kCameraPitch,
    units::degree_t(result.GetBestTarget().GetPitch())
  );

  // calculating the angular velocity the robot should travel at to ensure a smooth curve

  units::second_t timeToTarget = distance / m_desiredVelocity;
  units::radians_per_second_t angularVelocity = rotationToTarget.Radians() / timeToTarget;

  // calculating the overall velocity the robot should travel at
  // accounting for the desired linear and angular velocities and the target's position

  frc::DifferentialDriveWheelSpeeds currentSpeed = m_drivetrain->GetWheelSpeeds();
  frc::DifferentialDriveWheelSpeeds targetSpeed = m_kinematics.ToWheelSpeeds(m_ramsete.Calculate(frc::Pose2d(), targetPose, m_desiredVelocity, angularVelocity));

  // feedback and feedforward control, accounting for future and past inaccuracies in robot movement

  units::volt_t leftFF = m_feedforward.Calculate(targetSpeed.left, (targetSpeed.left - m_previousSpeed.left) / dt);
  units::volt_t rightFF = m_feedforward.Calculate(targetSpeed.right, (targetSpeed.right - m_previousSpeed.right) / dt);

  units::volt_t leftOut = units::volt_t(m_leftControl.Calculate(currentSpeed.left.value(), targetSpeed.left.value())) + leftFF;
  units::volt_t rightOut = units::volt_t(m_rightControl.Calculate(currentSpeed.right.value(), targetSpeed.right.value())) + rightFF;

  m_previousSpeed = targetSpeed;

  m_drivetrain->TankDriveVolts(leftOut, rightOut);
}

// Called once the command ends or is interrupted.
void GoToTarget::End(bool interrupted) {}

// Returns true when the command should end.
bool GoToTarget::IsFinished() {
  return false;
}
