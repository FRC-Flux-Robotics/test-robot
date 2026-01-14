package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DrivetrainIO;
import frc.robot.subsystems.drive.DrivetrainIOMock;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for SubsystemMockBuilder.
 * Verifies the builder creates correctly configured mock subsystems.
 */
class SubsystemMockBuilderTest {

    private SubsystemMockBuilder builder;

    @BeforeAll
    static void initHAL() {
        TestFixtures.initializeHAL();
    }

    @BeforeEach
    void setUp() {
        builder = new SubsystemMockBuilder();
    }

    @Test
    void buildDrivetrainIO_defaultValues_hasZeroState() {
        DrivetrainIOMock mock = builder.buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        assertEquals(0.0, inputs.gyroYawDegrees, 0.001);
        assertEquals(0.0, inputs.odometryX, 0.001);
        assertEquals(0.0, inputs.odometryY, 0.001);
        assertTrue(inputs.gyroConnected);
    }

    @Test
    void buildDrivetrainIO_atPosition_setsOdometry() {
        Pose2d pose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(90.0));
        DrivetrainIOMock mock = builder.atPosition(pose).buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        assertEquals(3.0, inputs.odometryX, 0.001);
        assertEquals(4.0, inputs.odometryY, 0.001);
        assertEquals(Math.toRadians(90.0), inputs.odometryRotationRad, 0.001);
    }

    @Test
    void buildDrivetrainIO_atPositionComponents_setsOdometry() {
        DrivetrainIOMock mock = builder.atPosition(1.5, 2.5, 45.0).buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        assertEquals(1.5, inputs.odometryX, 0.001);
        assertEquals(2.5, inputs.odometryY, 0.001);
        assertEquals(Math.toRadians(45.0), inputs.odometryRotationRad, 0.001);
    }

    @Test
    void buildDrivetrainIO_withGyro_setsGyroState() {
        DrivetrainIOMock mock = builder.withGyro(45.0, true).buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        assertEquals(45.0, inputs.gyroYawDegrees, 0.001);
        assertTrue(inputs.gyroConnected);
    }

    @Test
    void buildDrivetrainIO_withGyroDisconnected_setsDisconnected() {
        DrivetrainIOMock mock = builder.withGyro(0.0, false).buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        assertFalse(inputs.gyroConnected);
    }

    @Test
    void buildDrivetrainIO_withGyroYawRate_setsYawRate() {
        DrivetrainIOMock mock = builder.withGyroYawRate(30.0).buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        assertEquals(30.0, inputs.gyroYawRateDegPS, 0.001);
    }

    @Test
    void buildDrivetrainIO_withGyroPitch_setsPitch() {
        DrivetrainIOMock mock = builder.withGyroPitch(15.0).buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        assertEquals(15.0, inputs.gyroPitchDegrees, 0.001);
    }

    @Test
    void buildDrivetrainIO_withGyroRoll_setsRoll() {
        DrivetrainIOMock mock = builder.withGyroRoll(-10.0).buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        assertEquals(-10.0, inputs.gyroRollDegrees, 0.001);
    }

    @Test
    void buildDrivetrainIO_withDrivePositionsArray_setsPositions() {
        double[] positions = {1.0, 2.0, 3.0, 4.0};
        DrivetrainIOMock mock = builder.withDrivePositions(positions).buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        assertEquals(1.0, inputs.drivePositionsRad[0], 0.001);
        assertEquals(2.0, inputs.drivePositionsRad[1], 0.001);
        assertEquals(3.0, inputs.drivePositionsRad[2], 0.001);
        assertEquals(4.0, inputs.drivePositionsRad[3], 0.001);
    }

    @Test
    void buildDrivetrainIO_withDrivePositionsUniform_setsAllPositions() {
        DrivetrainIOMock mock = builder.withDrivePositions(5.0).buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        for (int i = 0; i < 4; i++) {
            assertEquals(5.0, inputs.drivePositionsRad[i], 0.001);
        }
    }

    @Test
    void buildDrivetrainIO_withDriveVelocitiesArray_setsVelocities() {
        double[] velocities = {10.0, 20.0, 30.0, 40.0};
        DrivetrainIOMock mock = builder.withDriveVelocities(velocities).buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        assertEquals(10.0, inputs.driveVelocitiesRadPerSec[0], 0.001);
        assertEquals(40.0, inputs.driveVelocitiesRadPerSec[3], 0.001);
    }

    @Test
    void buildDrivetrainIO_withDriveVelocitiesUniform_setsAllVelocities() {
        DrivetrainIOMock mock = builder.withDriveVelocities(15.0).buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        for (int i = 0; i < 4; i++) {
            assertEquals(15.0, inputs.driveVelocitiesRadPerSec[i], 0.001);
        }
    }

    @Test
    void buildDrivetrainIO_withDriveCurrentsArray_setsCurrents() {
        double[] currents = {5.0, 10.0, 15.0, 20.0};
        DrivetrainIOMock mock = builder.withDriveCurrents(currents).buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        assertEquals(5.0, inputs.driveCurrentAmps[0], 0.001);
        assertEquals(20.0, inputs.driveCurrentAmps[3], 0.001);
    }

    @Test
    void buildDrivetrainIO_withDriveCurrentsUniform_setsAllCurrents() {
        DrivetrainIOMock mock = builder.withDriveCurrents(25.0).buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        for (int i = 0; i < 4; i++) {
            assertEquals(25.0, inputs.driveCurrentAmps[i], 0.001);
        }
    }

    @Test
    void buildDrivetrainIO_withSteerPositions_setsPositions() {
        double[] positions = {0.1, 0.2, 0.3, 0.4};
        DrivetrainIOMock mock = builder.withSteerPositions(positions).buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        assertEquals(0.1, inputs.steerPositionsRad[0], 0.001);
        assertEquals(0.4, inputs.steerPositionsRad[3], 0.001);
    }

    @Test
    void buildDrivetrainIO_withSteerPositionsUniform_setsAllPositions() {
        DrivetrainIOMock mock = builder.withSteerPositions(0.5).buildDrivetrainIO();
        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        for (int i = 0; i < 4; i++) {
            assertEquals(0.5, inputs.steerPositionsRad[i], 0.001);
        }
    }

    @Test
    void buildDrivetrainIO_chainedConfiguration_allValuesSet() {
        DrivetrainIOMock mock = builder.atPosition(1.0, 2.0, 90.0)
                .withGyro(45.0, true)
                .withGyroYawRate(10.0)
                .withDriveCurrents(30.0)
                .withDriveVelocities(100.0)
                .buildDrivetrainIO();

        DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
        mock.updateInputs(inputs);

        assertEquals(1.0, inputs.odometryX, 0.001);
        assertEquals(2.0, inputs.odometryY, 0.001);
        assertEquals(45.0, inputs.gyroYawDegrees, 0.001);
        assertEquals(10.0, inputs.gyroYawRateDegPS, 0.001);
        assertEquals(30.0, inputs.driveCurrentAmps[0], 0.001);
        assertEquals(100.0, inputs.driveVelocitiesRadPerSec[0], 0.001);
    }

    @Test
    void buildMockDrivetrain_defaultPosition_returnsOrigin() {
        CommandSwerveDrivetrain mock = builder.buildMockDrivetrain();

        Pose2d position = mock.getPosition();
        assertEquals(0.0, position.getX(), 0.001);
        assertEquals(0.0, position.getY(), 0.001);
        assertEquals(0.0, position.getRotation().getDegrees(), 0.001);
    }

    @Test
    void buildMockDrivetrain_atPosition_returnsConfiguredPosition() {
        Pose2d expectedPose = new Pose2d(5.0, 6.0, Rotation2d.fromDegrees(180.0));
        CommandSwerveDrivetrain mock = builder.atPosition(expectedPose).buildMockDrivetrain();

        Pose2d position = mock.getPosition();
        assertEquals(5.0, position.getX(), 0.001);
        assertEquals(6.0, position.getY(), 0.001);
        assertEquals(180.0, position.getRotation().getDegrees(), 0.001);
    }

    @Test
    void builder_isReusable_eachBuildReturnsNewInstance() {
        builder.atPosition(1.0, 1.0, 0.0);

        DrivetrainIOMock mock1 = builder.buildDrivetrainIO();
        DrivetrainIOMock mock2 = builder.buildDrivetrainIO();

        assertNotSame(mock1, mock2);
    }
}
