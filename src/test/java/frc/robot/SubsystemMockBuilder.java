package frc.robot;

import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DrivetrainIOMock;

/**
 * Builder for creating customized mock subsystems with fluent API.
 * Use this for complex test scenarios requiring specific mock configurations.
 *
 * <p>Example usage:
 * <pre>
 * DrivetrainIOMock mockIO = new SubsystemMockBuilder()
 *     .atPosition(new Pose2d(1, 2, Rotation2d.kZero))
 *     .withGyro(45.0, true)
 *     .withDriveCurrents(new double[]{10, 10, 10, 10})
 *     .buildDrivetrainIO();
 * </pre>
 */
public class SubsystemMockBuilder {

    // Drivetrain configuration
    private Pose2d position = new Pose2d();
    private double gyroYaw = 0.0;
    private double gyroYawRate = 0.0;
    private double gyroPitch = 0.0;
    private double gyroRoll = 0.0;
    private boolean gyroConnected = true;
    private double[] drivePositions = new double[4];
    private double[] driveVelocities = new double[4];
    private double[] driveCurrents = new double[4];
    private double[] steerPositions = new double[4];
    private double[] steerVelocities = new double[4];
    private double[] steerCurrents = new double[4];

    /**
     * Sets the robot position for the mock.
     *
     * @param pose The position
     * @return This builder for chaining
     */
    public SubsystemMockBuilder atPosition(Pose2d pose) {
        this.position = pose;
        return this;
    }

    /**
     * Sets the robot position using x, y, and rotation.
     *
     * @param x X position in meters
     * @param y Y position in meters
     * @param rotationDegrees Rotation in degrees
     * @return This builder for chaining
     */
    public SubsystemMockBuilder atPosition(double x, double y, double rotationDegrees) {
        this.position = new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));
        return this;
    }

    /**
     * Sets the gyro yaw and connection status.
     *
     * @param yawDegrees Yaw in degrees
     * @param connected Whether gyro is connected
     * @return This builder for chaining
     */
    public SubsystemMockBuilder withGyro(double yawDegrees, boolean connected) {
        this.gyroYaw = yawDegrees;
        this.gyroConnected = connected;
        return this;
    }

    /**
     * Sets the gyro yaw rate.
     *
     * @param yawRateDegPS Yaw rate in degrees per second
     * @return This builder for chaining
     */
    public SubsystemMockBuilder withGyroYawRate(double yawRateDegPS) {
        this.gyroYawRate = yawRateDegPS;
        return this;
    }

    /**
     * Sets the gyro pitch.
     *
     * @param pitchDegrees Pitch in degrees
     * @return This builder for chaining
     */
    public SubsystemMockBuilder withGyroPitch(double pitchDegrees) {
        this.gyroPitch = pitchDegrees;
        return this;
    }

    /**
     * Sets the gyro roll.
     *
     * @param rollDegrees Roll in degrees
     * @return This builder for chaining
     */
    public SubsystemMockBuilder withGyroRoll(double rollDegrees) {
        this.gyroRoll = rollDegrees;
        return this;
    }

    /**
     * Sets the drive motor positions for all modules.
     *
     * @param positions Array of 4 positions in radians (FL, FR, BL, BR)
     * @return This builder for chaining
     */
    public SubsystemMockBuilder withDrivePositions(double[] positions) {
        System.arraycopy(positions, 0, this.drivePositions, 0, Math.min(positions.length, 4));
        return this;
    }

    /**
     * Sets uniform drive positions for all modules.
     *
     * @param position Position in radians for all modules
     * @return This builder for chaining
     */
    public SubsystemMockBuilder withDrivePositions(double position) {
        for (int i = 0; i < 4; i++) {
            this.drivePositions[i] = position;
        }
        return this;
    }

    /**
     * Sets the drive motor velocities for all modules.
     *
     * @param velocities Array of 4 velocities in rad/s (FL, FR, BL, BR)
     * @return This builder for chaining
     */
    public SubsystemMockBuilder withDriveVelocities(double[] velocities) {
        System.arraycopy(velocities, 0, this.driveVelocities, 0, Math.min(velocities.length, 4));
        return this;
    }

    /**
     * Sets uniform drive velocities for all modules.
     *
     * @param velocity Velocity in rad/s for all modules
     * @return This builder for chaining
     */
    public SubsystemMockBuilder withDriveVelocities(double velocity) {
        for (int i = 0; i < 4; i++) {
            this.driveVelocities[i] = velocity;
        }
        return this;
    }

    /**
     * Sets the drive motor currents for all modules.
     *
     * @param currents Array of 4 currents in amps (FL, FR, BL, BR)
     * @return This builder for chaining
     */
    public SubsystemMockBuilder withDriveCurrents(double[] currents) {
        System.arraycopy(currents, 0, this.driveCurrents, 0, Math.min(currents.length, 4));
        return this;
    }

    /**
     * Sets uniform drive currents for all modules.
     *
     * @param current Current in amps for all modules
     * @return This builder for chaining
     */
    public SubsystemMockBuilder withDriveCurrents(double current) {
        for (int i = 0; i < 4; i++) {
            this.driveCurrents[i] = current;
        }
        return this;
    }

    /**
     * Sets the steer motor positions for all modules.
     *
     * @param positions Array of 4 positions in radians (FL, FR, BL, BR)
     * @return This builder for chaining
     */
    public SubsystemMockBuilder withSteerPositions(double[] positions) {
        System.arraycopy(positions, 0, this.steerPositions, 0, Math.min(positions.length, 4));
        return this;
    }

    /**
     * Sets uniform steer positions for all modules.
     *
     * @param position Position in radians for all modules
     * @return This builder for chaining
     */
    public SubsystemMockBuilder withSteerPositions(double position) {
        for (int i = 0; i < 4; i++) {
            this.steerPositions[i] = position;
        }
        return this;
    }

    /**
     * Sets the steer motor velocities for all modules.
     *
     * @param velocities Array of 4 velocities in rad/s (FL, FR, BL, BR)
     * @return This builder for chaining
     */
    public SubsystemMockBuilder withSteerVelocities(double[] velocities) {
        System.arraycopy(velocities, 0, this.steerVelocities, 0, Math.min(velocities.length, 4));
        return this;
    }

    /**
     * Sets the steer motor currents for all modules.
     *
     * @param currents Array of 4 currents in amps (FL, FR, BL, BR)
     * @return This builder for chaining
     */
    public SubsystemMockBuilder withSteerCurrents(double[] currents) {
        System.arraycopy(currents, 0, this.steerCurrents, 0, Math.min(currents.length, 4));
        return this;
    }

    /**
     * Builds a DrivetrainIOMock with the configured values.
     *
     * @return A configured DrivetrainIOMock
     */
    public DrivetrainIOMock buildDrivetrainIO() {
        return new DrivetrainIOMock()
                .withOdometry(position)
                .withGyroYaw(gyroYaw)
                .withGyroYawRate(gyroYawRate)
                .withGyroPitch(gyroPitch)
                .withGyroRoll(gyroRoll)
                .withGyroConnected(gyroConnected)
                .withDrivePositions(drivePositions)
                .withDriveVelocities(driveVelocities)
                .withDriveCurrents(driveCurrents)
                .withSteerPositions(steerPositions)
                .withSteerVelocities(steerVelocities)
                .withSteerCurrents(steerCurrents);
    }

    /**
     * Builds a Mockito mock of CommandSwerveDrivetrain.
     * The mock is configured to return the builder's position from getPosition().
     *
     * @return A configured Mockito mock
     */
    public CommandSwerveDrivetrain buildMockDrivetrain() {
        CommandSwerveDrivetrain mock = mock(CommandSwerveDrivetrain.class);
        when(mock.getPosition()).thenReturn(position);
        return mock;
    }
}
