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
        public static final double AutoModeSpeed = -0.8; // Move backward to driver station
        public static final double AutoModeDriveTime = 2.2; // In seconds
    }

    public static final class LoggingConstants {
        /** Enable debug console logging. Set to false to disable all debug output. */
        public static final boolean ENABLE_DEBUG_LOGGING = true;

        /** Default log level: DEBUG, INFO, WARN, ERROR, or OFF. */
        public static final String DEFAULT_LOG_LEVEL = "INFO";

        /** Enable file-based logging for development sessions. */
        public static final boolean ENABLE_FILE_LOGGING = true;

        /** Directory for log files on real robot (RoboRIO). */
        public static final String FILE_LOG_DIRECTORY_REAL = "/home/lvuser/logs";

        /** Directory for log files in simulation/development. */
        public static final String FILE_LOG_DIRECTORY_SIM = "logs";
    }

    public static final class SimulationConstants {
        /** Robot mass in kilograms (approximately 125 lbs with battery). */
        public static final double ROBOT_MASS_KG = 56.7;

        /** Robot moment of inertia for rotation in kg·m². */
        public static final double ROBOT_MOI_KG_M2 = 6.0;

        /** Wheel radius in meters (2 inches). */
        public static final double WHEEL_RADIUS_METERS = 0.0508;

        /** Drive motor gear ratio (motor rotations per wheel rotation). */
        public static final double DRIVE_GEAR_RATIO = 6.394736842105262;

        /** Steer motor gear ratio. */
        public static final double STEER_GEAR_RATIO = 12.1;

        /** Drive motor moment of inertia in kg·m². */
        public static final double DRIVE_MOTOR_INERTIA_KG_M2 = 0.01;

        /** Steer motor moment of inertia in kg·m². */
        public static final double STEER_MOTOR_INERTIA_KG_M2 = 0.01;

        /** Track width (left-right distance between wheels) in meters. */
        public static final double TRACK_WIDTH_METERS = 0.5842;

        /** Wheel base (front-back distance between wheels) in meters. */
        public static final double WHEEL_BASE_METERS = 0.5842;

        /** Maximum speed at 12V in meters per second. */
        public static final double MAX_SPEED_MPS = 4.99;

        /** Simulated loop period in seconds (5ms = 200Hz). */
        public static final double SIM_LOOP_PERIOD_SECONDS = 0.005;
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

    /**
     * Safety-related constants for emergency stop, current limits, and brownout protection.
     */
    public static final class SafetyConstants {
        // Current Limits - Drive Motors
        /** Drive motor stator current limit (continuous) in Amps. */
        public static final double DRIVE_STATOR_CURRENT_LIMIT_AMPS = 40.0;

        /** Drive motor stator current limit (peak) in Amps. */
        public static final double DRIVE_STATOR_CURRENT_LIMIT_PEAK_AMPS = 60.0;

        /** Peak current duration in seconds. */
        public static final double DRIVE_PEAK_CURRENT_DURATION_SECONDS = 0.1;

        /** Drive motor supply current limit in Amps. */
        public static final double DRIVE_SUPPLY_CURRENT_LIMIT_AMPS = 35.0;

        // Current Limits - Steer Motors
        /** Steer motor stator current limit (continuous) in Amps. */
        public static final double STEER_STATOR_CURRENT_LIMIT_AMPS = 20.0;

        /** Steer motor stator current limit (peak) in Amps. */
        public static final double STEER_STATOR_CURRENT_LIMIT_PEAK_AMPS = 30.0;

        // Brownout Protection
        /** Voltage threshold below which speed is reduced. */
        public static final double BROWNOUT_VOLTAGE_THRESHOLD = 10.5;

        /** Speed multiplier when in brownout protection mode (0.0 to 1.0). */
        public static final double BROWNOUT_SPEED_MULTIPLIER = 0.5;

        /** Voltage threshold to exit brownout protection (hysteresis). */
        public static final double BROWNOUT_RECOVERY_VOLTAGE = 11.0;

        // SmartDashboard keys for telemetry
        /** SmartDashboard key for brownout warning. */
        public static final String BROWNOUT_WARNING_KEY = "Safety/BrownoutActive";

        /** SmartDashboard key for emergency stop state. */
        public static final String EMERGENCY_STOP_KEY = "Safety/EmergencyStopActive";

        /** SmartDashboard key for battery voltage. */
        public static final String BATTERY_VOLTAGE_KEY = "Safety/BatteryVoltage";
    }
}
