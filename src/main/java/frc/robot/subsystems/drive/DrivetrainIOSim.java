package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.SimulationConstants;

/**
 * Simulation implementation of DrivetrainIO using WPILib physics simulation.
 * Uses DCMotorSim for realistic motor dynamics including inertia, gear ratios,
 * and current draw estimation.
 */
public class DrivetrainIOSim implements DrivetrainIO {

    private static final int MODULE_COUNT = 4;

    // Motor model - using Kraken X60 (similar to Falcon 500 but newer)
    private static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(1);
    private static final DCMotor STEER_MOTOR = DCMotor.getFalcon500(1);

    // Drive motor simulations (one per module)
    private final DCMotorSim[] driveMotorSims = new DCMotorSim[MODULE_COUNT];
    private final DCMotorSim[] steerMotorSims = new DCMotorSim[MODULE_COUNT];

    // Simulated gyro state
    private double gyroYawDegrees = 0.0;
    private double gyroYawRateDegPS = 0.0;
    private double gyroPitchDegrees = 0.0;
    private double gyroRollDegrees = 0.0;

    // Simulated odometry
    private double odometryX = 0.0;
    private double odometryY = 0.0;
    private double odometryRotationRad = 0.0;

    // Requested velocities (field-centric)
    private double requestedVxMps = 0.0;
    private double requestedVyMps = 0.0;
    private double requestedOmegaRadPS = 0.0;
    private boolean isFieldCentric = true;

    // Target steer angles for each module
    private final double[] targetSteerAnglesRad = new double[MODULE_COUNT];

    // Operator perspective
    private Rotation2d operatorPerspective = Rotation2d.kZero;

    /**
     * Creates a new DrivetrainIOSim with physics-based motor simulation.
     */
    public DrivetrainIOSim() {
        // Initialize drive motor simulations
        for (int i = 0; i < MODULE_COUNT; i++) {
            driveMotorSims[i] = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DRIVE_MOTOR,
                    SimulationConstants.DRIVE_MOTOR_INERTIA_KG_M2,
                    SimulationConstants.DRIVE_GEAR_RATIO
                ),
                DRIVE_MOTOR
            );

            steerMotorSims[i] = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    STEER_MOTOR,
                    SimulationConstants.STEER_MOTOR_INERTIA_KG_M2,
                    SimulationConstants.STEER_GEAR_RATIO
                ),
                STEER_MOTOR
            );
        }
    }

    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        // Gyro data
        inputs.gyroConnected = true; // Always connected in simulation
        inputs.gyroYawDegrees = gyroYawDegrees;
        inputs.gyroYawRateDegPS = gyroYawRateDegPS;
        inputs.gyroPitchDegrees = gyroPitchDegrees;
        inputs.gyroRollDegrees = gyroRollDegrees;

        // Module data from physics simulation
        for (int i = 0; i < MODULE_COUNT; i++) {
            inputs.drivePositionsRad[i] = driveMotorSims[i].getAngularPositionRad();
            inputs.driveVelocitiesRadPerSec[i] = driveMotorSims[i].getAngularVelocityRadPerSec();
            inputs.driveAppliedVolts[i] = driveMotorSims[i].getInputVoltage();
            inputs.driveCurrentAmps[i] = Math.abs(driveMotorSims[i].getCurrentDrawAmps());

            inputs.steerPositionsRad[i] = steerMotorSims[i].getAngularPositionRad();
            inputs.steerVelocitiesRadPerSec[i] = steerMotorSims[i].getAngularVelocityRadPerSec();
            inputs.steerAppliedVolts[i] = steerMotorSims[i].getInputVoltage();
            inputs.steerCurrentAmps[i] = Math.abs(steerMotorSims[i].getCurrentDrawAmps());
        }

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
     * Call this periodically (e.g., every 5ms in the robot sim loop).
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

        // Module positions relative to center: FL, FR, BL, BR
        double halfTrack = SimulationConstants.TRACK_WIDTH_METERS / 2.0;
        double halfBase = SimulationConstants.WHEEL_BASE_METERS / 2.0;
        double[][] moduleOffsets = {
            {halfBase, halfTrack},   // FL
            {halfBase, -halfTrack},  // FR
            {-halfBase, halfTrack},  // BL
            {-halfBase, -halfTrack}  // BR
        };

        // Calculate and apply physics for each module
        for (int i = 0; i < MODULE_COUNT; i++) {
            // Calculate module velocity from robot motion
            double moduleVx = robotVxMps - requestedOmegaRadPS * moduleOffsets[i][1];
            double moduleVy = robotVyMps + requestedOmegaRadPS * moduleOffsets[i][0];
            double moduleSpeed = Math.sqrt(moduleVx * moduleVx + moduleVy * moduleVy);

            // Calculate target steer angle
            if (moduleSpeed > 0.001) {
                targetSteerAnglesRad[i] = Math.atan2(moduleVy, moduleVx);
            }

            // Steer motor control - simple proportional voltage based on angle error
            double currentSteerAngle = steerMotorSims[i].getAngularPositionRad();
            double steerError = MathUtil.angleModulus(targetSteerAnglesRad[i] - currentSteerAngle);
            double steerVoltage = MathUtil.clamp(steerError * 12.0, -12.0, 12.0);
            steerMotorSims[i].setInputVoltage(steerVoltage);
            steerMotorSims[i].update(dtSeconds);

            // Drive motor control - voltage based on desired wheel speed
            double wheelRadPerSec = moduleSpeed / SimulationConstants.WHEEL_RADIUS_METERS;
            // Simple feedforward voltage (V = kV * velocity)
            double driveVoltage = (wheelRadPerSec / (SimulationConstants.MAX_SPEED_MPS /
                SimulationConstants.WHEEL_RADIUS_METERS)) * 12.0;
            driveVoltage = MathUtil.clamp(driveVoltage, -12.0, 12.0);
            driveMotorSims[i].setInputVoltage(driveVoltage);
            driveMotorSims[i].update(dtSeconds);
        }

        // Calculate actual robot motion from module velocities
        double avgDriveVelRadPerSec = 0;
        for (int i = 0; i < MODULE_COUNT; i++) {
            avgDriveVelRadPerSec += driveMotorSims[i].getAngularVelocityRadPerSec();
        }
        avgDriveVelRadPerSec /= MODULE_COUNT;

        // Convert motor velocity to wheel velocity
        double wheelVelocityMps = (avgDriveVelRadPerSec / SimulationConstants.DRIVE_GEAR_RATIO)
            * SimulationConstants.WHEEL_RADIUS_METERS;

        // Use requested omega directly for rotation (simplified)
        gyroYawRateDegPS = Math.toDegrees(requestedOmegaRadPS);
        gyroYawDegrees += gyroYawRateDegPS * dtSeconds;

        // Normalize gyro yaw to [-180, 180)
        while (gyroYawDegrees >= 180.0) gyroYawDegrees -= 360.0;
        while (gyroYawDegrees < -180.0) gyroYawDegrees += 360.0;

        // Update odometry using actual wheel speeds
        double fieldAngleRad = Math.toRadians(gyroYawDegrees);
        double fieldCos = Math.cos(fieldAngleRad);
        double fieldSin = Math.sin(fieldAngleRad);

        // Estimate robot velocity from average module speed and heading
        double actualRobotVx = wheelVelocityMps * Math.cos(targetSteerAnglesRad[0]);
        double actualRobotVy = wheelVelocityMps * Math.sin(targetSteerAnglesRad[0]);

        odometryX += (actualRobotVx * fieldCos - actualRobotVy * fieldSin) * dtSeconds;
        odometryY += (actualRobotVx * fieldSin + actualRobotVy * fieldCos) * dtSeconds;
        odometryRotationRad = Math.toRadians(gyroYawDegrees);
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

    /**
     * Gets the total current draw from all simulated motors.
     *
     * @return Total current draw in amps
     */
    public double getTotalCurrentDrawAmps() {
        double total = 0.0;
        for (int i = 0; i < MODULE_COUNT; i++) {
            total += Math.abs(driveMotorSims[i].getCurrentDrawAmps());
            total += Math.abs(steerMotorSims[i].getCurrentDrawAmps());
        }
        return total;
    }
}
