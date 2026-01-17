package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.TestFixtures;
import frc.robot.subsystems.drive.DrivetrainIO;
import frc.robot.subsystems.drive.DrivetrainIOMock;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for TestableCommandSwerveDrivetrain.
 * Tests the periodic() logic that would run in CommandSwerveDrivetrain.
 */
class TestableCommandSwerveDrivetrainTest {

    private DrivetrainIOMock mockIO;
    private TestableCommandSwerveDrivetrain drivetrain;

    @BeforeAll
    static void initHAL() {
        TestFixtures.initializeHAL();
    }

    @BeforeEach
    void setUp() {
        mockIO = new DrivetrainIOMock();
        drivetrain = new TestableCommandSwerveDrivetrain(mockIO);
    }

    // ========== Initial State Tests ==========

    @Test
    void initialState_positionIsOrigin() {
        Pose2d pose = drivetrain.getPosition();

        assertEquals(0.0, pose.getX(), 0.001);
        assertEquals(0.0, pose.getY(), 0.001);
        assertEquals(0.0, pose.getRotation().getDegrees(), 0.001);
    }

    @Test
    void initialState_gyroRotationIsZero() {
        assertEquals(0.0, drivetrain.getGyroRotation().getDegrees(), 0.001);
    }

    @Test
    void initialState_operatorPerspectiveNotApplied() {
        assertFalse(drivetrain.hasAppliedOperatorPerspective());
    }

    // ========== Periodic Update Tests ==========

    @Test
    void periodic_updatesIOInputs() {
        assertEquals(0, mockIO.getUpdateInputsCallCount());

        drivetrain.periodic();

        assertEquals(1, mockIO.getUpdateInputsCallCount());
    }

    @Test
    void periodic_updatesPosition() {
        mockIO.withOdometry(3.0, 4.0, Math.PI / 4);

        drivetrain.periodic();

        Pose2d pose = drivetrain.getPosition();
        assertEquals(3.0, pose.getX(), 0.001);
        assertEquals(4.0, pose.getY(), 0.001);
        assertEquals(45.0, pose.getRotation().getDegrees(), 0.001);
    }

    @Test
    void periodic_updatesGyroRotation() {
        mockIO.withGyroYaw(90.0);

        drivetrain.periodic();

        assertEquals(90.0, drivetrain.getGyroRotation().getDegrees(), 0.001);
    }

    @Test
    void periodic_publishesToSmartDashboard() {
        mockIO.withOdometry(1.5, 2.5, Math.PI / 6);

        drivetrain.periodic();

        // SmartDashboard values should be updated
        // Note: In tests, SmartDashboard may not persist values, but we verify the method runs
        // The actual SmartDashboard behavior depends on HAL initialization
        Pose2d pose = drivetrain.getPosition();
        assertEquals(1.5, pose.getX(), 0.001);
        assertEquals(2.5, pose.getY(), 0.001);
    }

    @Test
    void periodic_multipleCallsUpdateCorrectly() {
        // First update
        mockIO.withOdometry(1.0, 1.0, 0.0);
        drivetrain.periodic();
        assertEquals(1.0, drivetrain.getPosition().getX(), 0.001);

        // Second update with new position
        mockIO.withOdometry(2.0, 2.0, Math.PI / 2);
        drivetrain.periodic();
        assertEquals(2.0, drivetrain.getPosition().getX(), 0.001);
        assertEquals(2.0, drivetrain.getPosition().getY(), 0.001);
        assertEquals(90.0, drivetrain.getPosition().getRotation().getDegrees(), 0.001);

        assertEquals(2, mockIO.getUpdateInputsCallCount());
    }

    // ========== Odometry Reset Tests ==========

    @Test
    void resetOdometry_updatesPose() {
        Pose2d newPose = new Pose2d(5.0, 6.0, Rotation2d.fromDegrees(45.0));

        drivetrain.resetOdometry(newPose);

        Pose2d pose = drivetrain.getPosition();
        assertEquals(5.0, pose.getX(), 0.001);
        assertEquals(6.0, pose.getY(), 0.001);
        assertEquals(45.0, pose.getRotation().getDegrees(), 0.001);
    }

    @Test
    void resetOdometry_updatesGyroRotation() {
        Pose2d newPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0));

        drivetrain.resetOdometry(newPose);

        assertEquals(180.0, drivetrain.getGyroRotation().getDegrees(), 0.001);
    }

    @Test
    void resetOdometry_callsIOResetOdometry() {
        Pose2d newPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));

        drivetrain.resetOdometry(newPose);

        assertEquals(newPose, mockIO.getLastResetPose());
    }

    // ========== Operator Perspective Tests ==========

    @Test
    void setOperatorPerspectiveForward_updatesPerspective() {
        Rotation2d perspective = Rotation2d.fromDegrees(180.0);

        drivetrain.setOperatorPerspectiveForward(perspective);

        assertEquals(180.0, drivetrain.getOperatorPerspective().getDegrees(), 0.001);
    }

    @Test
    void setOperatorPerspectiveForward_callsIO() {
        Rotation2d perspective = Rotation2d.fromDegrees(90.0);

        drivetrain.setOperatorPerspectiveForward(perspective);

        assertEquals(perspective, mockIO.getLastOperatorPerspective());
    }

    @Test
    void resetOperatorPerspective_resetsFlag() {
        drivetrain.setOperatorPerspectiveForward(Rotation2d.k180deg);
        // Simulate that perspective was applied (would happen in periodic with alliance)
        drivetrain.periodic(); // This may or may not set the flag depending on DriverStation state

        drivetrain.resetOperatorPerspective();

        assertFalse(drivetrain.hasAppliedOperatorPerspective());
        assertEquals(0.0, drivetrain.getOperatorPerspective().getDegrees(), 0.001);
    }

    // ========== Input Access Tests ==========

    @Test
    void getInputs_returnsCurrentInputs() {
        mockIO.withGyroYaw(45.0).withGyroPitch(5.0).withGyroRoll(-3.0).withOdometry(1.0, 2.0, 0.5);

        drivetrain.periodic();
        DrivetrainIO.DrivetrainIOInputs inputs = drivetrain.getInputs();

        assertEquals(45.0, inputs.gyroYawDegrees, 0.001);
        assertEquals(5.0, inputs.gyroPitchDegrees, 0.001);
        assertEquals(-3.0, inputs.gyroRollDegrees, 0.001);
        assertEquals(1.0, inputs.odometryX, 0.001);
        assertEquals(2.0, inputs.odometryY, 0.001);
    }

    @Test
    void getInputs_includesModuleData() {
        double[] driveVelocities = {1.0, 2.0, 3.0, 4.0};
        double[] steerPositions = {0.1, 0.2, 0.3, 0.4};
        mockIO.withDriveVelocities(driveVelocities).withSteerPositions(steerPositions);

        drivetrain.periodic();
        DrivetrainIO.DrivetrainIOInputs inputs = drivetrain.getInputs();

        assertEquals(1.0, inputs.driveVelocitiesRadPerSec[0], 0.001);
        assertEquals(4.0, inputs.driveVelocitiesRadPerSec[3], 0.001);
        assertEquals(0.1, inputs.steerPositionsRad[0], 0.001);
        assertEquals(0.4, inputs.steerPositionsRad[3], 0.001);
    }

    // ========== Gyro Connection Tests ==========

    @Test
    void getInputs_gyroConnectedByDefault() {
        drivetrain.periodic();

        assertTrue(drivetrain.getInputs().gyroConnected);
    }

    @Test
    void getInputs_detectsGyroDisconnected() {
        mockIO.withGyroConnected(false);

        drivetrain.periodic();

        assertFalse(drivetrain.getInputs().gyroConnected);
    }
}
