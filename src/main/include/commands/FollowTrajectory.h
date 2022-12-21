// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/RamseteCommand.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>

#include "Constants.h"
#include "subsystems/Drivetrain.h"

class FollowTrajectory : public frc2::RamseteCommand {
 public:
  FollowTrajectory(Drivetrain *drivetrain, std::string pathName, units::meters_per_second_t maxSpeed, units::meters_per_second_squared_t maxAcceleration);

 private:
  static frc::Trajectory LoadTrajectory(std::string pathName, units::meters_per_second_t maxSpeed, units::meters_per_second_squared_t maxAcceleration);
};
