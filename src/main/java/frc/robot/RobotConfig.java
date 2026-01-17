package frc.robot;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

/**
 * Hardware configuration container for swerve drive robots.
 *
 * <p>Defines the complete hardware layout for a swerve drivetrain, including:
 * <ul>
 *   <li>CAN bus assignments for drivetrain and system devices</li>
 *   <li>Pigeon 2 IMU configuration and CAN ID</li>
 *   <li>Swerve module configurations for all four corners</li>
 *   <li>Motor inversion settings for left/right sides</li>
 * </ul>
 *
 * <p>Pre-configured instances are provided for team robots:
 * <ul>
 *   <li>{@link #CoralRobot} - Competition robot with elevator</li>
 *   <li>{@link #PracticeRobot} - Practice robot configuration</li>
 * </ul>
 *
 * <p>Each robot configuration specifies CAN IDs, encoder offsets, and physical
 * positions for all four swerve modules, allowing the same codebase to run on
 * different physical robots.
 *
 * @see SwerveModuleConfig
 */
public final class RobotConfig {
    /** CAN bus name for system devices (typically "rio" for RoboRIO-integrated CAN). */
    public final String systemCANBus;

    /** CAN bus name for drivetrain devices (e.g., "Drivetrain" or "CANdace" for CANivore). */
    public final String driveCANBus;

    /** CAN ID of the Pigeon 2 IMU used for heading and orientation. */
    public final int pigeonId;

    /** Optional Pigeon 2 configuration settings. Set to null to use default configuration. */
    public final Pigeon2Configuration pigeonConfigs;

    /** Configuration for the front-left swerve module. */
    public final SwerveModuleConfig frontLeft;

    /** Configuration for the front-right swerve module. */
    public final SwerveModuleConfig frontRight;

    /** Configuration for the back-left swerve module. */
    public final SwerveModuleConfig backLeft;

    /** Configuration for the back-right swerve module. */
    public final SwerveModuleConfig backRight;

    /** Motor inversion setting for left-side drive motors (typically false). */
    public static final boolean InvertLeftSide = false;

    /** Motor inversion setting for right-side drive motors (typically true). */
    public static final boolean InvertRightSide = true;

    /**
     * Coral robot configuration (competition robot with elevator).
     * Uses "Drivetrain" CANivore bus with Pigeon ID 20.
     */
    public static final RobotConfig CoralRobot = new RobotConfig(
            "Drivetrain",
            "rio",
            20,
            new SwerveModuleConfig(1, 2, 24, -0.03173828125, 11.5, 11.5, InvertLeftSide, false, false),
            new SwerveModuleConfig(19, 18, 23, -0.455322265625, 11.5, -11.5, InvertRightSide, false, false),
            new SwerveModuleConfig(4, 3, 25, -0.12744140625, -11.5, 11.5, InvertLeftSide, false, false),
            new SwerveModuleConfig(16, 17, 26, 0.115478515625, -11.5, -11.5, InvertRightSide, false, false));

    /**
     * Practice robot configuration.
     * Uses "CANdace" CANivore bus with Pigeon ID 24.
     */
    public static final RobotConfig PracticeRobot = new RobotConfig(
            "CANdace",
            "rio",
            24,
            new SwerveModuleConfig(7, 8, 23, 0.124267578125, 11.5, 11.5, InvertLeftSide, false, false),
            new SwerveModuleConfig(1, 2, 20, -0.291015625, 11.5, -11.5, InvertRightSide, false, false),
            new SwerveModuleConfig(5, 6, 22, 0.048828125, -11.5, 11.5, InvertLeftSide, false, false),
            new SwerveModuleConfig(3, 4, 21, -0.371826171875, -11.5, -11.5, InvertRightSide, false, false));

    /**
     * Creates a new robot configuration with the specified hardware layout.
     *
     * @param driveCANBus name of the CAN bus for drivetrain devices (e.g., "Drivetrain" for CANivore)
     * @param systemCANBus name of the CAN bus for system devices (typically "rio")
     * @param pigeonId CAN ID of the Pigeon 2 IMU
     * @param fl front-left swerve module configuration
     * @param fr front-right swerve module configuration
     * @param bl back-left swerve module configuration
     * @param br back-right swerve module configuration
     */
    public RobotConfig(
            String driveCANBus,
            String systemCANBus,
            int pigeonId,
            SwerveModuleConfig fl,
            SwerveModuleConfig fr,
            SwerveModuleConfig bl,
            SwerveModuleConfig br) {
        // Validate CAN bus names
        if (driveCANBus == null || driveCANBus.isEmpty()) {
            throw new IllegalArgumentException("driveCANBus must not be null or empty");
        }
        if (systemCANBus == null || systemCANBus.isEmpty()) {
            throw new IllegalArgumentException("systemCANBus must not be null or empty");
        }

        // Validate Pigeon ID (CTRE valid range is 0-62)
        if (pigeonId < 0 || pigeonId > 62) {
            throw new IllegalArgumentException("pigeonId must be 0-62, got: " + pigeonId);
        }

        // Validate module configs
        if (fl == null) {
            throw new IllegalArgumentException("frontLeft module config must not be null");
        }
        if (fr == null) {
            throw new IllegalArgumentException("frontRight module config must not be null");
        }
        if (bl == null) {
            throw new IllegalArgumentException("backLeft module config must not be null");
        }
        if (br == null) {
            throw new IllegalArgumentException("backRight module config must not be null");
        }

        this.driveCANBus = driveCANBus;
        this.systemCANBus = systemCANBus;
        this.pigeonId = pigeonId;
        pigeonConfigs = null;
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
    }
}
