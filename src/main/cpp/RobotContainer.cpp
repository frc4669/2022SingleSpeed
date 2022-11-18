// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/RamseteCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPlanner.h>

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Setup F310 joystick bindings 
  m_drivetrain.SetDefaultCommand(frc2::RunCommand(
    [this] 
      {  m_drivetrain.CurvatureDrive(i_f310.getLeftJoyY(), i_f310.getRightJoyX() * OperatorConstants::kTurningSpeedMutiplier); },
      {  &m_drivetrain  }
  ));

  m_drivetrain.ResetEncoders();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  pathplanner::PathPlannerTrajectory autoTrajectory = pathplanner::PathPlanner::loadPath(TRAJECTORY_NAME, DriveConstants::kMaxAutoSpeed, DriveConstants::kMaxAutoAccel);

  m_drivetrain.ResetOdometry(autoTrajectory.getInitialPose(), autoTrajectory.getInitialPose().Rotation());

  frc2::RamseteCommand followTrajectory {
    autoTrajectory.asWPILibTrajectory(),
    &m_drivetrain.GetOdometryPose,
    frc::RamseteController(),
    frc::SimpleMotorFeedforward<units::meters>(DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
    frc::DifferentialDriveKinematics(DriveConstants::kTrackWidth),
    &m_drivetrain.GetWheelSpeeds,
    frc2::PIDController(DriveConstants::kp, DriveConstants::ki, DriveConstants::kd),
    frc2::PIDController(DriveConstants::kp, DriveConstants::ki, DriveConstants::kd),
    &m_drivetrain.TankDriveVolts,
    { &m_drivetrain }
  };

  return new frc2::SequentialCommandGroup(
    std::move(followTrajectory),
    frc2::InstantCommand([this] { m_drivetrain.TankDriveVolts(0_V, 0_V); })
  );
}
