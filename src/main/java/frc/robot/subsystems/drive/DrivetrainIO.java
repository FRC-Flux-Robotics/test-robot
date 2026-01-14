package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for the swerve drivetrain.
 * Follows AdvantageKit IO layer pattern for simulation and log replay support.
 */
public interface DrivetrainIO {

    /**
     * Sensor inputs from the drivetrain hardware.
     * Annotated with @AutoLog for automatic AdvantageKit logging.
     */
    @AutoLog
    class DrivetrainIOInputs {
        // Gyro readings
        public double gyroYawDegrees = 0.0;
        public double gyroYawRateDegPS = 0.0;
        public double gyroPitchDegrees = 0.0;
        public double gyroRollDegrees = 0.0;
        public boolean gyroConnected = false;

        // Per-module drive motor data (FL, FR, BL, BR)
        public double[] drivePositionsRad = new double[4];
        public double[] driveVelocitiesRadPerSec = new double[4];
        public double[] driveAppliedVolts = new double[4];
        public double[] driveCurrentAmps = new double[4];

        // Per-module steer motor data (FL, FR, BL, BR)
        public double[] steerPositionsRad = new double[4];
        public double[] steerVelocitiesRadPerSec = new double[4];
        public double[] steerAppliedVolts = new double[4];
        public double[] steerCurrentAmps = new double[4];

        // Odometry pose
        public double odometryX = 0.0;
        public double odometryY = 0.0;
        public double odometryRotationRad = 0.0;
    }

    /**
     * Updates all sensor inputs. Called periodically by the drivetrain subsystem.
     *
     * @param inputs The inputs object to populate with current sensor values
     */
    void updateInputs(DrivetrainIOInputs inputs);

    /**
     * Drives the robot using field-centric control.
     *
     * @param vxMetersPerSec X velocity in meters per second (field-relative)
     * @param vyMetersPerSec Y velocity in meters per second (field-relative)
     * @param omegaRadPerSec Angular velocity in radians per second
     */
    default void driveFieldCentric(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {}

    /**
     * Drives the robot using robot-centric control.
     *
     * @param vxMetersPerSec X velocity in meters per second (robot-relative)
     * @param vyMetersPerSec Y velocity in meters per second (robot-relative)
     * @param omegaRadPerSec Angular velocity in radians per second
     */
    default void driveRobotCentric(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {}

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to reset to
     */
    default void resetOdometry(Pose2d pose) {}

    /**
     * Stops all motors immediately.
     */
    default void stop() {}

    /**
     * Sets the operator perspective for field-centric driving.
     *
     * @param rotation The forward direction from the operator's perspective
     */
    default void setOperatorPerspective(Rotation2d rotation) {}
}
