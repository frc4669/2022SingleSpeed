// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/NewGoToTarget.h"

NewGoToTarget::NewGoToTarget(Drivetrain* drivetrain, Vision* vision) : m_drivetrain(drivetrain), m_vision(vision) {
  AddRequirements({ drivetrain, vision });
}

// Called when the command is initially scheduled.
void NewGoToTarget::Initialize() {
  m_finished = false;
  m_previousTime = -1_s;

  PhotonPipelineResult result = m_vision->GetPipelineResult();

  if (!result.HasTargets()) {
    m_finished = true;
    return;
  }

  frc::Transform3d bestCameraToTarget = result.GetBestTarget().GetBestCameraToTarget(); // 3d position of the target relative to the robot

  frc::Translation2d translationToTarget(bestCameraToTarget.X(), bestCameraToTarget.Y());
  frc::Rotation2d rotationOfTarget(bestCameraToTarget.Rotation().Z());
  frc::Transform2d transformToTarget(translationToTarget, rotationOfTarget); // 2d position of the target relative to the robot
  frc::Transform2d transformToEnd(frc::Translation2d(Dimensions::kDesiredRange, rotationOfTarget), frc::Rotation2d()); // position of the desired end point relative to the target

  frc::Pose2d robotPose = m_drivetrain->GetOdometryPose(); // starting position of the robot relative to the field
  frc::Pose2d endPose = robotPose.TransformBy(transformToTarget).TransformBy(transformToEnd); // desired end position of the robot relative to the field

  m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    robotPose, std::vector<frc::Translation2d>(), endPose, m_trajectoryConfig
  );

  m_drivetrain->GetField()->GetObject("trajectory")->SetTrajectory(m_trajectory); // displaying the trajectory on the driver station

  m_leftPID.Reset();
  m_rightPID.Reset();

  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void NewGoToTarget::Execute() {
  units::second_t currentTime = m_timer.Get();
  units::second_t delta = currentTime - m_previousTime; // time since the previous iteration of the command
  m_previousTime = currentTime;

  if (m_trajectory.TotalTime() > currentTime) {
    m_finished = true;
    return;
  }

  frc::DifferentialDriveWheelSpeeds currentSpeed = m_drivetrain->GetWheelSpeeds(); // current speed of the robot
  frc::DifferentialDriveWheelSpeeds newSpeed = m_kinematics.ToWheelSpeeds(m_ramsete.Calculate(m_drivetrain->GetOdometryPose(), m_trajectory.Sample(currentTime))); // desired speed of the robot at the next point in the trajectory

  // feedforward: accounting for known imperfections in motor output
  units::volt_t leftFF = m_feedforward.Calculate(currentSpeed.left, newSpeed.left, delta);
  units::volt_t rightFF = m_feedforward.Calculate(currentSpeed.right, newSpeed.right, delta);

  // PID control: accounting for the imprecision of the robot and its speed
  units::volt_t leftOut = units::volt_t(m_leftPID.Calculate(currentSpeed.left.value(), newSpeed.left.value())) + leftFF;
  units::volt_t rightOut = units::volt_t(m_rightPID.Calculate(currentSpeed.right.value(), newSpeed.right.value())) + rightFF;

  m_drivetrain->TankDriveVolts(leftOut, rightOut);
}

// Called once the command ends or is interrupted.
void NewGoToTarget::End(bool interrupted) {
  m_drivetrain->TankDriveVolts(0_V, 0_V);
}

// Returns true when the command should end.
bool NewGoToTarget::IsFinished() {
  return m_finished;
}
