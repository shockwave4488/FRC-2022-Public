// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * <p>Further, this class should only be used for constants that are universal across all of our
 * robots, put robot specifc constants in the correct Constants class in the robotspecifics folder
 */
public final class Constants {

  public static final class DriveTrainConstants {
    // Drive Train
    public static final int LEFT_FRONT_PORT = 0;
    public static final int RIGHT_FRONT_PORT = 1;
    public static final int LEFT_BACK_PORT = 2;
    public static final int RIGHT_BACK_PORT = 3;

    public static final double DRIVE_TRAIN_SPEED = 0.5;
    public static final double SWERVE_DRIVE_MAX_SPEED = 3.5;
    public static final double SWERVE_DRIVE_MAX_ACCEL = 3.5 / 2;
    public static final double SWERVE_ROTATION_SPEED = 2 * Math.PI;

    public static final double SWERVE_DRIVE_ROTATION_P = 0.018;
    public static final double SWERVE_DRIVE_ROTATION_I = 0;
    public static final double SWERVE_DRIVE_ROTATION_D = 0;
  }

  public static final class OIConstants {
    // Joysticks
    public static final int XBOX_LEFT_Y_AXIS = 1; // double check
    public static final int XBOX_LEFT_X_AXIS = 0; // double check
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int BUTTON_BOX_PORT = 1;
    public static final double DEFAULT_CONTROLLER_DEADZONE = 0.3;
    public static final double BIG_CONTROLLER_DEADZONE = 0.6;
  }

  public static final class ShooterConstants {
    public static final int MASTER_PORT = 0;
    public static final int FOLLOWER_PORT = 0;
    public static final double RAMP_RATE = 0.1;

    public static final double FENDER_RPM = 2050; // Needs to be adjusted/tested
    public static final double FENDER_HOOD_INPUT = 8; // 6.6 -> 8
    public static final double BACK_OF_TARMAC_RPM = 2225; // Adjusted but untested
    public static final double BACK_OF_TARMAC_HOOD_INPUT = 40; // Adjusted but untested
    public static final double MIN_HOOD_POSITION = 6.6; // Based on the CAD
    public static final double FRONT_LAUNCHPAD_RPM = 2675; // Theoretically correct but untested
    public static final double FRONT_LAUNCHPAD_HOOD_INPUT =
        59.8; // Theoretically correct but untested
    public static final double SIDE_LAUNCHPAD_RPM = 2775; // Theoretically correct but untested
    public static final double SIDE_LAUNCHPAD_HOOD_INPUT = 65; // Theoretically correct but untested
  }

  public static final class FieldConstants {
    public static final Translation2d HUB_CENTER = new Translation2d(8.22, 4.15);
    public static final double HUB_RADIUS_METERS =
        0.63; // This is the distance between the center of the HUB and the INSIDE edge (0.61) plus
    // 0.02 (for the width of what the reflective tape is on, this is an approximation).
    public static final double HUB_SAFE_SHOT_RADIUS_METERS = 0.3; // 0.4 -> 0.3
  }

  public static final class AutonomousConstants {
    public static final double POS_X_PATH_P = 4.6;
    public static final double POS_X_PATH_I = 0.0;
    public static final double POS_X_PATH_D = 0.0;

    public static final double POS_Y_PATH_P = 4.6;
    public static final double POS_Y_PATH_I = 0.0;
    public static final double POS_Y_PATH_D = 0.0;

    public static final double AUTO_TURN_P = 5;

    public static final double PRACTICE_SHOOTER_RPM_OUTSIDE_TARMAC = 1500;
    public static final double PRACTICE_SHOOTER_RPM_INSIDE_TARMAC = 1500; // 2800 when in comp

    public static final double COMP_SHOOTER_RPM_INSIDE_TARMAC = 1974; // 2050 -> 2025 -> :)
    public static final double COMP_SHOOTER_HOOD_INPUT_INSIDE_TARMAC = 33; // updated
    public static final double COMP_SHOOTER_RPM_OUTSIDE_TARMAC =
        2265; // 2375 -> 2300 -> 2275 -> 2265
    public static final double COMP_SHOOTER_HOOD_INPUT_OUTSIDE_TARMAC = 52; // Adjusted but untested
  }

  public static final class LEDControllerConstants {
    public static final int ledDio1 = 11;
    public static final int ledDio2 = 12;
    public static final int ledDio3 = 13;
  }

  public static final class UnitConversionConstants {
    public static final double INCHES_TO_METERS = 0.0254;
  }
}
