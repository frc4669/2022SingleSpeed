// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FollowTrajectory.h"

FollowTrajectory::FollowTrajectory(Drivetrain *drivetrain, std::string pathName, units::meters_per_second_t maxSpeed, units::meters_per_second_squared_t maxAcceleration)
    : frc2::RamseteCommand(
          LoadTrajectory(pathName, maxSpeed, maxAcceleration),
          [this, drivetrain] { return drivetrain->GetOdometryPose(); },
          frc::RamseteController(),
          frc::SimpleMotorFeedforward<units::meters>(DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
          frc::DifferentialDriveKinematics(DriveConstants::kTrackWidth),
          [this, drivetrain] { return drivetrain->GetWheelSpeeds(); },
          frc2::PIDController(DriveConstants::kp, DriveConstants::ki, DriveConstants::kd),
          frc2::PIDController(DriveConstants::kp, DriveConstants::ki, DriveConstants::kd),
          [this, drivetrain](auto left, auto right) { return drivetrain->TankDriveVolts(left, right); },
          { drivetrain })
{
  AddRequirements({ drivetrain });

  frc::Trajectory trajectory = LoadTrajectory(pathName, maxSpeed, maxAcceleration);

  drivetrain->ResetOdometry(trajectory.InitialPose(), trajectory.InitialPose().Rotation());
  drivetrain->GetField()->GetObject("trajectory")->SetTrajectory(trajectory);
}

frc::Trajectory FollowTrajectory::LoadTrajectory(std::string pathName, units::meters_per_second_t maxSpeed, units::meters_per_second_squared_t maxAcceleration) {
  pathplanner::PathPlannerTrajectory path = pathplanner::PathPlanner::loadPath(pathName, maxSpeed, maxAcceleration);
  return path.asWPILibTrajectory();
}