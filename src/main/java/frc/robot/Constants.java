// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DriveConstants {
    public static final double AutoModeSpeed = -0.8;  // Move backward to driver station
    public static final double AutoModeDriveTime = 2.2;  // In seconds
  }

  public static final class LoggingConstants {
    /** Enable debug console logging. Set to false to disable all debug output. */
    public static final boolean ENABLE_DEBUG_LOGGING = true;

    /** Default log level: DEBUG, INFO, WARN, ERROR, or OFF. */
    public static final String DEFAULT_LOG_LEVEL = "INFO";
  }

  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;
    public static final int OperatorControllerPort = 1;

    public static final boolean UseTwoControllers = true;

    public static final double TriggerThreshold = 0.2;

    public static final double xStartPos = 0.02;
    public static final double xMiddlePos = 0.6;
    public static final double yStartPos = 0.1;
    public static final double yMiddlePos = 0.4;
    public static final double yMaxPos = 0.8;

    public static final double xStartRot = 0.02;
    public static final double xMiddleRot = 0.6;
    public static final double yStartRot = 0.1;
    public static final double yMiddleRot = 0.6;
    public static final double yMaxRot = 1.0;

    public static final double LinCoef = 0.15;
    public static final double Threshold = 0.0;
    public static final double CuspX = 0.9;
    public static final double MinLimit = 0.4;

    public static final double RotLinCoef = 0.2;
    public static final double RotThreshold = 0.0;
    public static final double RotCuspX = 0.5;
  }
}
