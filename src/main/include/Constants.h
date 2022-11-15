// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {
  constexpr bool kCanTurnInPlace = true; // curvature drive turning in place

  constexpr double kTurningSpeedMutiplier = 0.5; // slows down movement as joystick is too sensentive. 
} 

namespace DriveConstants {
  // Drivetrain Talon FX CAN IDs
  constexpr int kLeftMain = 11;    // Leading left motor
  constexpr int kLeftSecondary = 12;     // Following left motor

  constexpr int kRightMain = 21;   // Leading right motor
  constexpr int kRightSecondary = 22;    // Following right motor
  
} // namespace DriveConstants

