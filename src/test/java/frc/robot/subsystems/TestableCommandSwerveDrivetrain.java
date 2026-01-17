package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DrivetrainIO;
import frc.robot.subsystems.drive.DrivetrainIO.DrivetrainIOInputs;

/**
 * Test-only class that mirrors CommandSwerveDrivetrain's periodic behavior
 * without requiring Phoenix 6 hardware dependencies.
 *
 * <p>This class extracts the testable logic from CommandSwerveDrivetrain.periodic()
 * into a hardware-independent implementation. Use this for unit testing the
 * drivetrain's periodic update logic.
 *
 * <p>The class does NOT extend CommandSwerveDrivetrain because that would require
 * calling the super constructor which creates Phoenix 6 hardware.
 */
public class TestableCommandSwerveDrivetrain {

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d RED_ALLIANCE_PERSPECTIVE = Rotation2d.k180deg;

    private final DrivetrainIO io;
    private final DrivetrainIOInputs inputs;

    private Pose2d currentPose = new Pose2d();
    private Rotation2d gyroRotation = Rotation2d.kZero;
    private boolean hasAppliedOperatorPerspective = false;
    private Rotation2d operatorPerspective = Rotation2d.kZero;

    /**
     * Creates a testable drivetrain wrapper with the specified IO.
     *
     * @param io The DrivetrainIO implementation (typically a mock)
     */
    public TestableCommandSwerveDrivetrain(DrivetrainIO io) {
        this.io = io;
        this.inputs = new DrivetrainIOInputs();
    }

    /**
     * Runs the periodic update logic, mirroring CommandSwerveDrivetrain.periodic().
     *
     * <p>This method:
     * <ul>
     *   <li>Applies operator perspective based on alliance color when disabled</li>
     *   <li>Updates IO inputs from the hardware abstraction layer</li>
     *   <li>Logs inputs to AdvantageKit</li>
     *   <li>Updates odometry from gyro readings</li>
     *   <li>Publishes position data to SmartDashboard</li>
     * </ul>
     */
    public void periodic() {
        // Apply operator perspective when disabled or not yet applied
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RED_ALLIANCE_PERSPECTIVE : BLUE_ALLIANCE_PERSPECTIVE);
                hasAppliedOperatorPerspective = true;
            });
        }

        // Update IO inputs
        io.updateInputs(inputs);

        // Note: AdvantageKit logging (Logger.processInputs) is skipped in test wrapper
        // because it requires DrivetrainIOInputsAutoLogged which is generated at compile time
        // and not easily accessible from test code. The real CommandSwerveDrivetrain
        // handles this logging.

        // Update pose from odometry inputs
        currentPose = new Pose2d(inputs.odometryX, inputs.odometryY, new Rotation2d(inputs.odometryRotationRad));

        // Update gyro rotation
        gyroRotation = Rotation2d.fromDegrees(inputs.gyroYawDegrees);

        // Publish to SmartDashboard
        SmartDashboard.putNumber("Position_X", currentPose.getX());
        SmartDashboard.putNumber("Position_Y", currentPose.getY());
        SmartDashboard.putNumber("Rotation_Grad", currentPose.getRotation().getDegrees());
    }

    /**
     * Sets the operator perspective forward direction.
     *
     * @param rotation The forward direction from the operator's perspective
     */
    public void setOperatorPerspectiveForward(Rotation2d rotation) {
        this.operatorPerspective = rotation;
        io.setOperatorPerspective(rotation);
    }

    /**
     * Returns the current robot position.
     *
     * @return The current pose
     */
    public Pose2d getPosition() {
        return currentPose;
    }

    /**
     * Returns the current gyro rotation.
     *
     * @return The gyro rotation
     */
    public Rotation2d getGyroRotation() {
        return gyroRotation;
    }

    /**
     * Returns whether the operator perspective has been applied.
     *
     * @return true if operator perspective has been applied at least once
     */
    public boolean hasAppliedOperatorPerspective() {
        return hasAppliedOperatorPerspective;
    }

    /**
     * Returns the current operator perspective.
     *
     * @return The operator perspective rotation
     */
    public Rotation2d getOperatorPerspective() {
        return operatorPerspective;
    }

    /**
     * Returns the latest IO inputs.
     *
     * @return The DrivetrainIOInputs
     */
    public DrivetrainIOInputs getInputs() {
        return inputs;
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        io.resetOdometry(pose);
        currentPose = pose;
        gyroRotation = pose.getRotation();
    }

    /**
     * Resets the hasAppliedOperatorPerspective flag.
     * Useful for testing the operator perspective application logic.
     */
    public void resetOperatorPerspective() {
        hasAppliedOperatorPerspective = false;
        operatorPerspective = Rotation2d.kZero;
    }
}
