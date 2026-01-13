package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Simulation implementation of DrivetrainIO for AdvantageKit log replay and testing.
 * Provides a simple physics model for swerve drivetrain simulation.
 */
public class DrivetrainIOSim implements DrivetrainIO {

    private static final int MODULE_COUNT = 4;

    // Simulation constants (from TunerConstants)
    private static final double WHEEL_RADIUS_METERS = 0.0508; // 2 inches
    private static final double DRIVE_GEAR_RATIO = 6.394736842105262;
    private static final double TRACK_WIDTH_METERS = 0.5842; // 23 inches
    private static final double WHEEL_BASE_METERS = 0.5842; // 23 inches
    private static final double MAX_SPEED_MPS = 4.99; // From TunerConstants.kSpeedAt12Volts

    // Simulated gyro state
    private double gyroYawDegrees = 0.0;
    private double gyroYawRateDegPS = 0.0;
    private double gyroPitchDegrees = 0.0;
    private double gyroRollDegrees = 0.0;

    // Simulated module state
    private final double[] drivePositionsRad = new double[MODULE_COUNT];
    private final double[] driveVelocitiesRadPerSec = new double[MODULE_COUNT];
    private final double[] driveAppliedVolts = new double[MODULE_COUNT];
    private final double[] driveCurrentAmps = new double[MODULE_COUNT];

    private final double[] steerPositionsRad = new double[MODULE_COUNT];
    private final double[] steerVelocitiesRadPerSec = new double[MODULE_COUNT];
    private final double[] steerAppliedVolts = new double[MODULE_COUNT];
    private final double[] steerCurrentAmps = new double[MODULE_COUNT];

    // Simulated odometry
    private double odometryX = 0.0;
    private double odometryY = 0.0;
    private double odometryRotationRad = 0.0;

    // Requested velocities (field-centric)
    private double requestedVxMps = 0.0;
    private double requestedVyMps = 0.0;
    private double requestedOmegaRadPS = 0.0;
    private boolean isFieldCentric = true;

    // Operator perspective
    private Rotation2d operatorPerspective = Rotation2d.kZero;

    /**
     * Creates a new DrivetrainIOSim with default configuration.
     */
    public DrivetrainIOSim() {
        // Initialize all arrays to zero (already done by default)
    }

    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        // Gyro data
        inputs.gyroConnected = true; // Always connected in simulation
        inputs.gyroYawDegrees = gyroYawDegrees;
        inputs.gyroYawRateDegPS = gyroYawRateDegPS;
        inputs.gyroPitchDegrees = gyroPitchDegrees;
        inputs.gyroRollDegrees = gyroRollDegrees;

        // Module data
        System.arraycopy(drivePositionsRad, 0, inputs.drivePositionsRad, 0, MODULE_COUNT);
        System.arraycopy(driveVelocitiesRadPerSec, 0, inputs.driveVelocitiesRadPerSec, 0, MODULE_COUNT);
        System.arraycopy(driveAppliedVolts, 0, inputs.driveAppliedVolts, 0, MODULE_COUNT);
        System.arraycopy(driveCurrentAmps, 0, inputs.driveCurrentAmps, 0, MODULE_COUNT);

        System.arraycopy(steerPositionsRad, 0, inputs.steerPositionsRad, 0, MODULE_COUNT);
        System.arraycopy(steerVelocitiesRadPerSec, 0, inputs.steerVelocitiesRadPerSec, 0, MODULE_COUNT);
        System.arraycopy(steerAppliedVolts, 0, inputs.steerAppliedVolts, 0, MODULE_COUNT);
        System.arraycopy(steerCurrentAmps, 0, inputs.steerCurrentAmps, 0, MODULE_COUNT);

        // Odometry
        inputs.odometryX = odometryX;
        inputs.odometryY = odometryY;
        inputs.odometryRotationRad = odometryRotationRad;
    }

    @Override
    public void driveFieldCentric(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {
        requestedVxMps = vxMetersPerSec;
        requestedVyMps = vyMetersPerSec;
        requestedOmegaRadPS = omegaRadPerSec;
        isFieldCentric = true;
    }

    @Override
    public void driveRobotCentric(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {
        requestedVxMps = vxMetersPerSec;
        requestedVyMps = vyMetersPerSec;
        requestedOmegaRadPS = omegaRadPerSec;
        isFieldCentric = false;
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        odometryX = pose.getX();
        odometryY = pose.getY();
        odometryRotationRad = pose.getRotation().getRadians();
        gyroYawDegrees = pose.getRotation().getDegrees();
    }

    @Override
    public void stop() {
        requestedVxMps = 0.0;
        requestedVyMps = 0.0;
        requestedOmegaRadPS = 0.0;
    }

    @Override
    public void setOperatorPerspective(Rotation2d rotation) {
        operatorPerspective = rotation;
    }

    /**
     * Steps the physics simulation forward by the given time delta.
     * Call this periodically (e.g., every 20ms in the robot loop).
     *
     * @param dtSeconds Time delta in seconds
     */
    public void updateSimulation(double dtSeconds) {
        // Convert field-centric to robot-centric if needed
        double robotVxMps;
        double robotVyMps;

        if (isFieldCentric) {
            // Rotate field velocities to robot frame
            double robotAngleRad = Math.toRadians(gyroYawDegrees);
            double cos = Math.cos(robotAngleRad);
            double sin = Math.sin(robotAngleRad);
            robotVxMps = requestedVxMps * cos + requestedVyMps * sin;
            robotVyMps = -requestedVxMps * sin + requestedVyMps * cos;
        } else {
            robotVxMps = requestedVxMps;
            robotVyMps = requestedVyMps;
        }

        // Update gyro yaw rate and integrate
        gyroYawRateDegPS = Math.toDegrees(requestedOmegaRadPS);
        gyroYawDegrees += gyroYawRateDegPS * dtSeconds;

        // Normalize gyro yaw to [-180, 180)
        while (gyroYawDegrees >= 180.0) gyroYawDegrees -= 360.0;
        while (gyroYawDegrees < -180.0) gyroYawDegrees += 360.0;

        // Update odometry (integrate in field frame)
        double fieldAngleRad = Math.toRadians(gyroYawDegrees);
        double fieldCos = Math.cos(fieldAngleRad);
        double fieldSin = Math.sin(fieldAngleRad);
        odometryX += (robotVxMps * fieldCos - robotVyMps * fieldSin) * dtSeconds;
        odometryY += (robotVxMps * fieldSin + robotVyMps * fieldCos) * dtSeconds;
        odometryRotationRad = Math.toRadians(gyroYawDegrees);

        // Calculate module states from robot velocities (simplified kinematics)
        // Using differential swerve approximation for each corner
        double halfTrack = TRACK_WIDTH_METERS / 2.0;
        double halfBase = WHEEL_BASE_METERS / 2.0;

        // Module positions relative to center: FL, FR, BL, BR
        double[][] moduleOffsets = {
            {halfBase, halfTrack},   // FL
            {halfBase, -halfTrack},  // FR
            {-halfBase, halfTrack},  // BL
            {-halfBase, -halfTrack}  // BR
        };

        for (int i = 0; i < MODULE_COUNT; i++) {
            // Calculate module velocity from robot motion
            double moduleVx = robotVxMps - requestedOmegaRadPS * moduleOffsets[i][1];
            double moduleVy = robotVyMps + requestedOmegaRadPS * moduleOffsets[i][0];
            double moduleSpeed = Math.sqrt(moduleVx * moduleVx + moduleVy * moduleVy);

            // Steer angle (atan2 of velocity components)
            if (moduleSpeed > 0.001) {
                steerPositionsRad[i] = Math.atan2(moduleVy, moduleVx);
            }
            // Keep previous steer angle if not moving

            // Drive velocity in motor radians per second
            double wheelRadPerSec = moduleSpeed / WHEEL_RADIUS_METERS;
            double motorRadPerSec = wheelRadPerSec * DRIVE_GEAR_RATIO;
            driveVelocitiesRadPerSec[i] = motorRadPerSec;

            // Integrate drive position
            drivePositionsRad[i] += motorRadPerSec * dtSeconds;

            // Estimate applied voltage from velocity (simple linear model)
            double normalizedSpeed = moduleSpeed / MAX_SPEED_MPS;
            driveAppliedVolts[i] = normalizedSpeed * 12.0;

            // Estimate current (rough approximation)
            driveCurrentAmps[i] = Math.abs(normalizedSpeed) * 20.0;
        }
    }

    /**
     * Gets the current simulated pose.
     *
     * @return The current pose
     */
    public Pose2d getPose() {
        return new Pose2d(odometryX, odometryY, new Rotation2d(odometryRotationRad));
    }

    /**
     * Gets the current gyro yaw in degrees.
     *
     * @return Gyro yaw in degrees
     */
    public double getGyroYawDegrees() {
        return gyroYawDegrees;
    }
}
