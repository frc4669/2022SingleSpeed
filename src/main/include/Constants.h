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

#define TRAJECTORY_NAME "OneMeter"//"EventAutoChallenge"

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/time.h>
#include <units/voltage.h>
#include <units/length.h>
#include <units/constants.h>

namespace OperatorConstants {
  constexpr bool kCanTurnInPlace = true; // curvature drive turning in place

  constexpr double kTurningSpeedMutiplier = 0.25; // slows down movement as joystick is too sensentive. 
} 

namespace DriveConstants {
  constexpr int kLeftMain = 21;          // Leading left motor
  constexpr int kLeftSecondary = 22;     // Following left motor

  constexpr int kRightMain = 11;         // Leading right motor
  constexpr int kRightSecondary = 12;    // Following right motor

  constexpr auto ks = 0.55805_V;//0.56801_V;
  constexpr auto kv = 2.5552_V * 1_s / 1_m;//1.2256_V * 1_s / 1_m;
  constexpr auto ka = 0.17303_V * 1_s * 1_s / 1_m; //0.071355_V

  constexpr double kp = 2.7863;//1.2487;
  constexpr double ki = 0;
  constexpr double kd = 0;

  constexpr double kGearRatio = 11.25;

  constexpr auto kTrackWidth = 20.75_in;
  constexpr double kWheelCircumference = 6 * units::constants::pi;
  constexpr double kInchesPerTick = kWheelCircumference / (2048 * kGearRatio);

  constexpr auto kMaxAutoSpeed = 0.3_mps;
  constexpr auto kMaxAutoAccel = 1_mps_sq;
}

