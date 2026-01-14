package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.TestFixtures;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for DrivetrainIOMock.
 * Verifies call tracking and state configuration work correctly.
 */
class DrivetrainIOMockTest {

    private DrivetrainIOMock mock;
    private DrivetrainIO.DrivetrainIOInputs inputs;

    @BeforeAll
    static void initHAL() {
        TestFixtures.initializeHAL();
    }

    @BeforeEach
    void setUp() {
        mock = new DrivetrainIOMock();
        inputs = new DrivetrainIO.DrivetrainIOInputs();
    }

    // ========== Configuration Tests ==========

    @Test
    void defaultState_hasZeroValues() {
        mock.updateInputs(inputs);

        assertEquals(0.0, inputs.gyroYawDegrees, 0.001);
        assertEquals(0.0, inputs.gyroYawRateDegPS, 0.001);
        assertTrue(inputs.gyroConnected);
        assertEquals(0.0, inputs.odometryX, 0.001);
    }

    @Test
    void withGyroYaw_setsYaw() {
        mock.withGyroYaw(45.0);
        mock.updateInputs(inputs);

        assertEquals(45.0, inputs.gyroYawDegrees, 0.001);
    }

    @Test
    void withGyroYawRate_setsYawRate() {
        mock.withGyroYawRate(90.0);
        mock.updateInputs(inputs);

        assertEquals(90.0, inputs.gyroYawRateDegPS, 0.001);
    }

    @Test
    void withGyroPitch_setsPitch() {
        mock.withGyroPitch(15.0);
        mock.updateInputs(inputs);

        assertEquals(15.0, inputs.gyroPitchDegrees, 0.001);
    }

    @Test
    void withGyroRoll_setsRoll() {
        mock.withGyroRoll(-10.0);
        mock.updateInputs(inputs);

        assertEquals(-10.0, inputs.gyroRollDegrees, 0.001);
    }

    @Test
    void withGyroConnected_setsConnectionStatus() {
        mock.withGyroConnected(false);
        mock.updateInputs(inputs);

        assertFalse(inputs.gyroConnected);
    }

    @Test
    void withOdometry_componentsForm_setsOdometry() {
        mock.withOdometry(1.0, 2.0, Math.PI / 2);
        mock.updateInputs(inputs);

        assertEquals(1.0, inputs.odometryX, 0.001);
        assertEquals(2.0, inputs.odometryY, 0.001);
        assertEquals(Math.PI / 2, inputs.odometryRotationRad, 0.001);
    }

    @Test
    void withOdometry_poseForm_setsOdometry() {
        Pose2d pose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(180.0));
        mock.withOdometry(pose);
        mock.updateInputs(inputs);

        assertEquals(3.0, inputs.odometryX, 0.001);
        assertEquals(4.0, inputs.odometryY, 0.001);
        assertEquals(Math.PI, inputs.odometryRotationRad, 0.001);
    }

    @Test
    void withDrivePositions_setsPositions() {
        double[] positions = {1.0, 2.0, 3.0, 4.0};
        mock.withDrivePositions(positions);
        mock.updateInputs(inputs);

        assertEquals(1.0, inputs.drivePositionsRad[0], 0.001);
        assertEquals(2.0, inputs.drivePositionsRad[1], 0.001);
        assertEquals(3.0, inputs.drivePositionsRad[2], 0.001);
        assertEquals(4.0, inputs.drivePositionsRad[3], 0.001);
    }

    @Test
    void withDriveVelocities_setsVelocities() {
        double[] velocities = {10.0, 20.0, 30.0, 40.0};
        mock.withDriveVelocities(velocities);
        mock.updateInputs(inputs);

        assertEquals(10.0, inputs.driveVelocitiesRadPerSec[0], 0.001);
        assertEquals(40.0, inputs.driveVelocitiesRadPerSec[3], 0.001);
    }

    @Test
    void withDriveCurrents_setsCurrents() {
        double[] currents = {5.0, 10.0, 15.0, 20.0};
        mock.withDriveCurrents(currents);
        mock.updateInputs(inputs);

        assertEquals(5.0, inputs.driveCurrentAmps[0], 0.001);
        assertEquals(20.0, inputs.driveCurrentAmps[3], 0.001);
    }

    @Test
    void withSteerPositions_setsPositions() {
        double[] positions = {0.1, 0.2, 0.3, 0.4};
        mock.withSteerPositions(positions);
        mock.updateInputs(inputs);

        assertEquals(0.1, inputs.steerPositionsRad[0], 0.001);
        assertEquals(0.4, inputs.steerPositionsRad[3], 0.001);
    }

    @Test
    void withSteerVelocities_setsVelocities() {
        double[] velocities = {1.0, 2.0, 3.0, 4.0};
        mock.withSteerVelocities(velocities);
        mock.updateInputs(inputs);

        assertEquals(1.0, inputs.steerVelocitiesRadPerSec[0], 0.001);
        assertEquals(4.0, inputs.steerVelocitiesRadPerSec[3], 0.001);
    }

    @Test
    void withSteerCurrents_setsCurrents() {
        double[] currents = {2.0, 4.0, 6.0, 8.0};
        mock.withSteerCurrents(currents);
        mock.updateInputs(inputs);

        assertEquals(2.0, inputs.steerCurrentAmps[0], 0.001);
        assertEquals(8.0, inputs.steerCurrentAmps[3], 0.001);
    }

    @Test
    void fluentChaining_returnsThisForChaining() {
        DrivetrainIOMock result = mock.withGyroYaw(10.0).withGyroYawRate(5.0).withOdometry(1.0, 2.0, 0.0);

        assertSame(mock, result);
    }

    // ========== Call Tracking Tests ==========

    @Test
    void driveFieldCentric_tracksCalls() {
        mock.driveFieldCentric(1.0, 0.5, 0.1);

        assertTrue(mock.wasDriveFieldCentricCalled());
        assertEquals(1, mock.getDriveCalls().size());
    }

    @Test
    void driveFieldCentric_recordsParameters() {
        mock.driveFieldCentric(1.5, 2.5, 0.3);

        DrivetrainIOMock.DriveCall call = mock.getLastDriveCall();
        assertNotNull(call);
        assertEquals(1.5, call.vxMetersPerSec, 0.001);
        assertEquals(2.5, call.vyMetersPerSec, 0.001);
        assertEquals(0.3, call.omegaRadPerSec, 0.001);
        assertTrue(call.fieldCentric);
    }

    @Test
    void driveRobotCentric_tracksCalls() {
        mock.driveRobotCentric(0.5, 0.5, 0.0);

        assertTrue(mock.wasDriveRobotCentricCalled());
        assertFalse(mock.wasDriveFieldCentricCalled());
    }

    @Test
    void driveRobotCentric_recordsAsNotFieldCentric() {
        mock.driveRobotCentric(1.0, 0.0, 0.0);

        DrivetrainIOMock.DriveCall call = mock.getLastDriveCall();
        assertFalse(call.fieldCentric);
    }

    @Test
    void getFieldCentricCalls_filtersCorrectly() {
        mock.driveFieldCentric(1.0, 0.0, 0.0);
        mock.driveRobotCentric(0.5, 0.0, 0.0);
        mock.driveFieldCentric(0.0, 1.0, 0.0);

        List<DrivetrainIOMock.DriveCall> fieldCentricCalls = mock.getFieldCentricCalls();
        assertEquals(2, fieldCentricCalls.size());
        assertTrue(fieldCentricCalls.stream().allMatch(c -> c.fieldCentric));
    }

    @Test
    void getRobotCentricCalls_filtersCorrectly() {
        mock.driveFieldCentric(1.0, 0.0, 0.0);
        mock.driveRobotCentric(0.5, 0.0, 0.0);
        mock.driveRobotCentric(0.0, 0.5, 0.0);

        List<DrivetrainIOMock.DriveCall> robotCentricCalls = mock.getRobotCentricCalls();
        assertEquals(2, robotCentricCalls.size());
        assertTrue(robotCentricCalls.stream().noneMatch(c -> c.fieldCentric));
    }

    @Test
    void stop_incrementsCounter() {
        assertEquals(0, mock.getStopCallCount());

        mock.stop();
        assertEquals(1, mock.getStopCallCount());

        mock.stop();
        mock.stop();
        assertEquals(3, mock.getStopCallCount());
    }

    @Test
    void updateInputs_incrementsCounter() {
        assertEquals(0, mock.getUpdateInputsCallCount());

        mock.updateInputs(inputs);
        assertEquals(1, mock.getUpdateInputsCallCount());

        mock.updateInputs(inputs);
        mock.updateInputs(inputs);
        assertEquals(3, mock.getUpdateInputsCallCount());
    }

    @Test
    void resetOdometry_updatesOdometryAndTracksCall() {
        Pose2d pose = new Pose2d(5.0, 6.0, Rotation2d.fromDegrees(90.0));
        mock.resetOdometry(pose);

        assertEquals(pose, mock.getLastResetPose());

        mock.updateInputs(inputs);
        assertEquals(5.0, inputs.odometryX, 0.001);
        assertEquals(6.0, inputs.odometryY, 0.001);
        assertEquals(90.0, inputs.gyroYawDegrees, 0.001);
    }

    @Test
    void setOperatorPerspective_tracksCall() {
        Rotation2d perspective = Rotation2d.fromDegrees(180.0);
        mock.setOperatorPerspective(perspective);

        assertEquals(perspective, mock.getLastOperatorPerspective());
    }

    @Test
    void getCurrentOdometry_returnsPose() {
        mock.withOdometry(1.0, 2.0, Math.PI / 4);

        Pose2d pose = mock.getCurrentOdometry();
        assertEquals(1.0, pose.getX(), 0.001);
        assertEquals(2.0, pose.getY(), 0.001);
        assertEquals(Math.PI / 4, pose.getRotation().getRadians(), 0.001);
    }

    // ========== Reset Tests ==========

    @Test
    void reset_clearsCallTracking() {
        mock.driveFieldCentric(1.0, 0.0, 0.0);
        mock.stop();
        mock.updateInputs(inputs);
        mock.resetOdometry(new Pose2d(1, 1, Rotation2d.kZero));
        mock.setOperatorPerspective(Rotation2d.k180deg);

        mock.reset();

        assertTrue(mock.getDriveCalls().isEmpty());
        assertEquals(0, mock.getStopCallCount());
        assertEquals(0, mock.getUpdateInputsCallCount());
        assertNull(mock.getLastResetPose());
        assertNull(mock.getLastOperatorPerspective());
    }

    @Test
    void reset_preservesConfiguredInputs() {
        mock.withGyroYaw(45.0).withOdometry(1.0, 2.0, 0.0);
        mock.driveFieldCentric(1.0, 0.0, 0.0);

        mock.reset();
        mock.updateInputs(inputs);

        // Configured inputs should still be there
        assertEquals(45.0, inputs.gyroYawDegrees, 0.001);
        assertEquals(1.0, inputs.odometryX, 0.001);
    }

    @Test
    void resetAll_clearsEverything() {
        mock.withGyroYaw(45.0).withOdometry(1.0, 2.0, 0.0);
        mock.driveFieldCentric(1.0, 0.0, 0.0);
        mock.stop();

        mock.resetAll();
        mock.updateInputs(inputs);

        // Call tracking should be cleared
        assertTrue(mock.getDriveCalls().isEmpty());
        assertEquals(0, mock.getStopCallCount());

        // Configured inputs should be reset
        assertEquals(0.0, inputs.gyroYawDegrees, 0.001);
        assertEquals(0.0, inputs.odometryX, 0.001);
    }

    @Test
    void getLastDriveCall_returnsNull_whenNoCalls() {
        assertNull(mock.getLastDriveCall());
    }

    @Test
    void getLastDriveCall_returnsLastCall() {
        mock.driveFieldCentric(1.0, 0.0, 0.0);
        mock.driveFieldCentric(2.0, 0.0, 0.0);
        mock.driveFieldCentric(3.0, 0.0, 0.0);

        DrivetrainIOMock.DriveCall lastCall = mock.getLastDriveCall();
        assertEquals(3.0, lastCall.vxMetersPerSec, 0.001);
    }
}
