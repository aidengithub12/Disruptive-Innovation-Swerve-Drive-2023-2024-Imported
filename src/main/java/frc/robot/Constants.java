// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM;
  public static final double ANGLE_START_POSITION = -.75;
  public static final double ANGLE_CLIMB_UP = 110;
  public static final double ANGLE_CLIMB_DOWN = 10;
  public static final double ANGLE_AMP = 100;
  public static final double ANGLE_SPEAKER = 0;
  // public static final double ARM_GEAR_RATIO = 152;
  // ((11.0 / 28.0) * (1.0 / 40.0) * (36.0 / 54.0));
  public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
  public static final int LED_LENGTH = 46;
  public static final int LED2Length = 42;
  public static final int DRIVE_ENCODER_FL = 16;
  public static final int DRIVE_ENCODER_FR = 19;
  public static final int DRIVE_ENCODER_BL = 17;
  public static final int DRIVE_ENCODER_BR = 18;
  public static final int DRIVE_FRONT_LEFT = 7;
  public static final int TURN_FRONT_LEFT = 8;
  public static final int DRIVE_FRONT_RIGHT = 2;
  public static final int TURN_FRONT_RIGHT = 3;
  public static final int DRIVE_BACK_RIGHT = 11;
  public static final int TURN_BACK_RIGHT = 12;
  public static final int DRIVE_BACK_LEFT = 9;
  public static final int TURN_BACK_LEFT = 10;
  public static final int SHOOTER_TOP = 4;
  public static final int SHOOTER_BOTTOM = 6;
  public static final int INDEXER = 5;
  public static final int INTAKE = 14;
  public static final int RIGHT_WORM_GEAR = 13;
  public static final int LEFT_WORM_GEAR = 15;
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(10);
  public static final double MAX_LINEAR_SPEED_TURBO = Units.feetToMeters(40);
  public static final double MAX_LINEAR_SPEED_AUTO = Units.feetToMeters(25);
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(20.0);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(20.0);
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double DRIVEDEADBAND = 0.2;
  public static final int RED_SPEAKER_ID = 4;
  public static final int BLUE_SPEAKER_ID = 7;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
