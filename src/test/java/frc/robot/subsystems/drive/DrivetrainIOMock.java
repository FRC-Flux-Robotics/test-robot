package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Mock implementation of DrivetrainIO for unit testing.
 * Provides configurable state and call tracking for verification.
 */
public class DrivetrainIOMock implements DrivetrainIO {

    private static final int MODULE_COUNT = 4;

    // Configurable inputs that will be returned by updateInputs
    private double gyroYawDegrees = 0.0;
    private double gyroYawRateDegPS = 0.0;
    private double gyroPitchDegrees = 0.0;
    private double gyroRollDegrees = 0.0;
    private boolean gyroConnected = true;

    private double[] drivePositionsRad = new double[MODULE_COUNT];
    private double[] driveVelocitiesRadPerSec = new double[MODULE_COUNT];
    private double[] driveAppliedVolts = new double[MODULE_COUNT];
    private double[] driveCurrentAmps = new double[MODULE_COUNT];

    private double[] steerPositionsRad = new double[MODULE_COUNT];
    private double[] steerVelocitiesRadPerSec = new double[MODULE_COUNT];
    private double[] steerAppliedVolts = new double[MODULE_COUNT];
    private double[] steerCurrentAmps = new double[MODULE_COUNT];

    private double odometryX = 0.0;
    private double odometryY = 0.0;
    private double odometryRotationRad = 0.0;

    // Call tracking
    private final List<DriveCall> driveCalls = new ArrayList<>();
    private int stopCallCount = 0;
    private int updateInputsCallCount = 0;
    private Pose2d lastResetPose = null;
    private Rotation2d lastOperatorPerspective = null;

    /**
     * Record of a drive command call.
     */
    public static class DriveCall {
        public final double vxMetersPerSec;
        public final double vyMetersPerSec;
        public final double omegaRadPerSec;
        public final boolean fieldCentric;

        public DriveCall(double vx, double vy, double omega, boolean fieldCentric) {
            this.vxMetersPerSec = vx;
            this.vyMetersPerSec = vy;
            this.omegaRadPerSec = omega;
            this.fieldCentric = fieldCentric;
        }
    }

    // ========== Configuration Methods (Fluent API) ==========

    /**
     * Sets the gyro yaw reading.
     */
    public DrivetrainIOMock withGyroYaw(double degrees) {
        this.gyroYawDegrees = degrees;
        return this;
    }

    /**
     * Sets the gyro yaw rate reading.
     */
    public DrivetrainIOMock withGyroYawRate(double degPerSec) {
        this.gyroYawRateDegPS = degPerSec;
        return this;
    }

    /**
     * Sets the gyro pitch reading.
     */
    public DrivetrainIOMock withGyroPitch(double degrees) {
        this.gyroPitchDegrees = degrees;
        return this;
    }

    /**
     * Sets the gyro roll reading.
     */
    public DrivetrainIOMock withGyroRoll(double degrees) {
        this.gyroRollDegrees = degrees;
        return this;
    }

    /**
     * Sets whether the gyro is connected.
     */
    public DrivetrainIOMock withGyroConnected(boolean connected) {
        this.gyroConnected = connected;
        return this;
    }

    /**
     * Sets the odometry position.
     */
    public DrivetrainIOMock withOdometry(double x, double y, double rotationRad) {
        this.odometryX = x;
        this.odometryY = y;
        this.odometryRotationRad = rotationRad;
        return this;
    }

    /**
     * Sets the odometry from a Pose2d.
     */
    public DrivetrainIOMock withOdometry(Pose2d pose) {
        this.odometryX = pose.getX();
        this.odometryY = pose.getY();
        this.odometryRotationRad = pose.getRotation().getRadians();
        return this;
    }

    /**
     * Sets drive motor positions for all modules.
     */
    public DrivetrainIOMock withDrivePositions(double[] positions) {
        System.arraycopy(positions, 0, this.drivePositionsRad, 0,
            Math.min(positions.length, MODULE_COUNT));
        return this;
    }

    /**
     * Sets drive motor velocities for all modules.
     */
    public DrivetrainIOMock withDriveVelocities(double[] velocities) {
        System.arraycopy(velocities, 0, this.driveVelocitiesRadPerSec, 0,
            Math.min(velocities.length, MODULE_COUNT));
        return this;
    }

    /**
     * Sets drive motor currents for all modules.
     */
    public DrivetrainIOMock withDriveCurrents(double[] currents) {
        System.arraycopy(currents, 0, this.driveCurrentAmps, 0,
            Math.min(currents.length, MODULE_COUNT));
        return this;
    }

    /**
     * Sets steer motor positions for all modules.
     */
    public DrivetrainIOMock withSteerPositions(double[] positions) {
        System.arraycopy(positions, 0, this.steerPositionsRad, 0,
            Math.min(positions.length, MODULE_COUNT));
        return this;
    }

    /**
     * Sets steer motor velocities for all modules.
     */
    public DrivetrainIOMock withSteerVelocities(double[] velocities) {
        System.arraycopy(velocities, 0, this.steerVelocitiesRadPerSec, 0,
            Math.min(velocities.length, MODULE_COUNT));
        return this;
    }

    /**
     * Sets steer motor currents for all modules.
     */
    public DrivetrainIOMock withSteerCurrents(double[] currents) {
        System.arraycopy(currents, 0, this.steerCurrentAmps, 0,
            Math.min(currents.length, MODULE_COUNT));
        return this;
    }

    // ========== DrivetrainIO Implementation ==========

    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        updateInputsCallCount++;

        inputs.gyroYawDegrees = gyroYawDegrees;
        inputs.gyroYawRateDegPS = gyroYawRateDegPS;
        inputs.gyroPitchDegrees = gyroPitchDegrees;
        inputs.gyroRollDegrees = gyroRollDegrees;
        inputs.gyroConnected = gyroConnected;

        System.arraycopy(drivePositionsRad, 0, inputs.drivePositionsRad, 0, MODULE_COUNT);
        System.arraycopy(driveVelocitiesRadPerSec, 0, inputs.driveVelocitiesRadPerSec, 0, MODULE_COUNT);
        System.arraycopy(driveAppliedVolts, 0, inputs.driveAppliedVolts, 0, MODULE_COUNT);
        System.arraycopy(driveCurrentAmps, 0, inputs.driveCurrentAmps, 0, MODULE_COUNT);

        System.arraycopy(steerPositionsRad, 0, inputs.steerPositionsRad, 0, MODULE_COUNT);
        System.arraycopy(steerVelocitiesRadPerSec, 0, inputs.steerVelocitiesRadPerSec, 0, MODULE_COUNT);
        System.arraycopy(steerAppliedVolts, 0, inputs.steerAppliedVolts, 0, MODULE_COUNT);
        System.arraycopy(steerCurrentAmps, 0, inputs.steerCurrentAmps, 0, MODULE_COUNT);

        inputs.odometryX = odometryX;
        inputs.odometryY = odometryY;
        inputs.odometryRotationRad = odometryRotationRad;
    }

    @Override
    public void driveFieldCentric(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {
        driveCalls.add(new DriveCall(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec, true));
    }

    @Override
    public void driveRobotCentric(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {
        driveCalls.add(new DriveCall(vxMetersPerSec, vyMetersPerSec, omegaRadPerSec, false));
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        lastResetPose = pose;
        odometryX = pose.getX();
        odometryY = pose.getY();
        odometryRotationRad = pose.getRotation().getRadians();
        gyroYawDegrees = pose.getRotation().getDegrees();
    }

    @Override
    public void stop() {
        stopCallCount++;
    }

    @Override
    public void setOperatorPerspective(Rotation2d rotation) {
        lastOperatorPerspective = rotation;
    }

    // ========== Verification Methods ==========

    /**
     * Returns true if driveFieldCentric was called at least once.
     */
    public boolean wasDriveFieldCentricCalled() {
        return driveCalls.stream().anyMatch(c -> c.fieldCentric);
    }

    /**
     * Returns true if driveRobotCentric was called at least once.
     */
    public boolean wasDriveRobotCentricCalled() {
        return driveCalls.stream().anyMatch(c -> !c.fieldCentric);
    }

    /**
     * Returns all drive calls in order.
     */
    public List<DriveCall> getDriveCalls() {
        return new ArrayList<>(driveCalls);
    }

    /**
     * Returns only field-centric drive calls.
     */
    public List<DriveCall> getFieldCentricCalls() {
        return driveCalls.stream().filter(c -> c.fieldCentric).toList();
    }

    /**
     * Returns only robot-centric drive calls.
     */
    public List<DriveCall> getRobotCentricCalls() {
        return driveCalls.stream().filter(c -> !c.fieldCentric).toList();
    }

    /**
     * Returns the last drive call, or null if none.
     */
    public DriveCall getLastDriveCall() {
        return driveCalls.isEmpty() ? null : driveCalls.get(driveCalls.size() - 1);
    }

    /**
     * Returns the number of times stop() was called.
     */
    public int getStopCallCount() {
        return stopCallCount;
    }

    /**
     * Returns the number of times updateInputs() was called.
     */
    public int getUpdateInputsCallCount() {
        return updateInputsCallCount;
    }

    /**
     * Returns the last pose passed to resetOdometry(), or null if never called.
     */
    public Pose2d getLastResetPose() {
        return lastResetPose;
    }

    /**
     * Returns the last rotation passed to setOperatorPerspective(), or null if never called.
     */
    public Rotation2d getLastOperatorPerspective() {
        return lastOperatorPerspective;
    }

    /**
     * Returns the current configured odometry as a Pose2d.
     */
    public Pose2d getCurrentOdometry() {
        return new Pose2d(odometryX, odometryY, new Rotation2d(odometryRotationRad));
    }

    /**
     * Resets all call tracking state.
     */
    public void reset() {
        driveCalls.clear();
        stopCallCount = 0;
        updateInputsCallCount = 0;
        lastResetPose = null;
        lastOperatorPerspective = null;
    }

    /**
     * Resets all state including configured inputs.
     */
    public void resetAll() {
        reset();
        gyroYawDegrees = 0.0;
        gyroYawRateDegPS = 0.0;
        gyroPitchDegrees = 0.0;
        gyroRollDegrees = 0.0;
        gyroConnected = true;
        drivePositionsRad = new double[MODULE_COUNT];
        driveVelocitiesRadPerSec = new double[MODULE_COUNT];
        driveAppliedVolts = new double[MODULE_COUNT];
        driveCurrentAmps = new double[MODULE_COUNT];
        steerPositionsRad = new double[MODULE_COUNT];
        steerVelocitiesRadPerSec = new double[MODULE_COUNT];
        steerAppliedVolts = new double[MODULE_COUNT];
        steerCurrentAmps = new double[MODULE_COUNT];
        odometryX = 0.0;
        odometryY = 0.0;
        odometryRotationRad = 0.0;
    }
}
