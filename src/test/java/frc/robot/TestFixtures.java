package frc.robot;

import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DrivetrainIOMock;

/**
 * Test fixtures for FRC robot unit tests.
 * Provides HAL initialization, test configuration, and mock subsystem helpers.
 */
public final class TestFixtures {
    private static boolean halInitialized = false;

    private TestFixtures() {
        // Utility class - prevent instantiation
    }

    /**
     * Initialize the WPILib HAL for simulation mode.
     * Thread-safe and idempotent - safe to call multiple times.
     */
    public static synchronized void initializeHAL() {
        if (!halInitialized) {
            HAL.initialize(500, 0);
            halInitialized = true;
        }
    }

    /**
     * Create a RobotConfig suitable for testing.
     * Uses unique CAN IDs that won't conflict with real hardware.
     *
     * @return A valid RobotConfig for test use
     */
    public static RobotConfig createTestRobotConfig() {
        return new RobotConfig(
                "TestBus",
                "rio",
                1,
                createTestSwerveModuleConfig(1, 2, 10, 0.0, 10.0, 10.0), // Front Left
                createTestSwerveModuleConfig(3, 4, 11, 0.0, 10.0, -10.0), // Front Right
                createTestSwerveModuleConfig(5, 6, 12, 0.0, -10.0, 10.0), // Back Left
                createTestSwerveModuleConfig(7, 8, 13, 0.0, -10.0, -10.0) // Back Right
                );
    }

    /**
     * Create a SwerveModuleConfig for testing.
     *
     * @param driveId   Drive motor CAN ID
     * @param steerId   Steer motor CAN ID
     * @param encoderId Encoder CAN ID
     * @param offset    Encoder offset in rotations
     * @param xPos      X position in inches
     * @param yPos      Y position in inches
     * @return A configured SwerveModuleConfig
     */
    public static SwerveModuleConfig createTestSwerveModuleConfig(
            int driveId, int steerId, int encoderId, double offset, double xPos, double yPos) {
        return new SwerveModuleConfig(driveId, steerId, encoderId, offset, xPos, yPos, false, false, false);
    }

    /**
     * Create a SwerveModuleConfig with custom inversion settings.
     *
     * @param driveId            Drive motor CAN ID
     * @param steerId            Steer motor CAN ID
     * @param encoderId          Encoder CAN ID
     * @param offset             Encoder offset in rotations
     * @param xPos               X position in inches
     * @param yPos               Y position in inches
     * @param invertSide         Invert the drive motor
     * @param steerMotorInverted Invert the steer motor
     * @param encoderInverted    Invert the encoder
     * @return A configured SwerveModuleConfig
     */
    public static SwerveModuleConfig createTestSwerveModuleConfig(
            int driveId,
            int steerId,
            int encoderId,
            double offset,
            double xPos,
            double yPos,
            boolean invertSide,
            boolean steerMotorInverted,
            boolean encoderInverted) {
        return new SwerveModuleConfig(
                driveId, steerId, encoderId, offset, xPos, yPos, invertSide, steerMotorInverted, encoderInverted);
    }

    // ========== Mock Subsystem Factory Methods ==========

    /**
     * Creates a new DrivetrainIOMock with default configuration.
     * The mock starts with all values at zero and gyro connected.
     *
     * @return A new DrivetrainIOMock instance
     */
    public static DrivetrainIOMock createMockDrivetrainIO() {
        return new DrivetrainIOMock();
    }

    /**
     * Creates a DrivetrainIOMock configured with a specific position.
     *
     * @param pose The initial odometry pose
     * @return A configured DrivetrainIOMock instance
     */
    public static DrivetrainIOMock createMockDrivetrainIO(Pose2d pose) {
        return new DrivetrainIOMock().withOdometry(pose);
    }

    /**
     * Creates a Mockito mock of CommandSwerveDrivetrain.
     * The mock is configured with common stubs for basic functionality.
     *
     * @return A Mockito mock of CommandSwerveDrivetrain
     */
    public static CommandSwerveDrivetrain createMockDrivetrain() {
        CommandSwerveDrivetrain mock = mock(CommandSwerveDrivetrain.class);
        // Configure common stubs
        when(mock.getPosition()).thenReturn(new Pose2d());
        return mock;
    }

    /**
     * Creates a Mockito mock of CommandSwerveDrivetrain at a specific position.
     *
     * @param pose The position to return from getPosition()
     * @return A Mockito mock of CommandSwerveDrivetrain
     */
    public static CommandSwerveDrivetrain createMockDrivetrain(Pose2d pose) {
        CommandSwerveDrivetrain mock = mock(CommandSwerveDrivetrain.class);
        when(mock.getPosition()).thenReturn(pose);
        return mock;
    }
}
