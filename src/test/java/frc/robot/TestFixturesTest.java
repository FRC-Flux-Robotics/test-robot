package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/**
 * Unit tests for TestFixtures.
 * Verifies test helper methods work correctly.
 */
class TestFixturesTest extends DrivetrainTestBase {

    @Test
    void createTestRobotConfig_returnsValidConfig() {
        RobotConfig config = TestFixtures.createTestRobotConfig();

        assertNotNull(config);
        assertNotNull(config.driveCANBus);
        assertNotNull(config.systemCANBus);

        // Verify all four modules are present
        assertNotNull(config.frontLeft);
        assertNotNull(config.frontRight);
        assertNotNull(config.backLeft);
        assertNotNull(config.backRight);
    }

    @Test
    void createTestRobotConfig_hasUniqueCAN_IDs() {
        RobotConfig config = TestFixtures.createTestRobotConfig();

        // Collect all CAN IDs
        int[] allDriveIds = {
            config.frontLeft.driveMotorId,
            config.frontRight.driveMotorId,
            config.backLeft.driveMotorId,
            config.backRight.driveMotorId
        };

        int[] allSteerIds = {
            config.frontLeft.steerMotorId,
            config.frontRight.steerMotorId,
            config.backLeft.steerMotorId,
            config.backRight.steerMotorId
        };

        int[] allEncoderIds = {
            config.frontLeft.encoderId,
            config.frontRight.encoderId,
            config.backLeft.encoderId,
            config.backRight.encoderId
        };

        // Verify uniqueness within each category
        assertEquals(4, java.util.Arrays.stream(allDriveIds).distinct().count(), "Drive motor IDs should be unique");
        assertEquals(4, java.util.Arrays.stream(allSteerIds).distinct().count(), "Steer motor IDs should be unique");
        assertEquals(4, java.util.Arrays.stream(allEncoderIds).distinct().count(), "Encoder IDs should be unique");
    }

    @Test
    void createTestSwerveModuleConfig_setsAllFields() {
        SwerveModuleConfig config = TestFixtures.createTestSwerveModuleConfig(
                5,
                6,
                15, // driveId, steerId, encoderId
                0.125, // offset
                12.0,
                -12.0 // xPos, yPos
                );

        assertEquals(5, config.driveMotorId);
        assertEquals(6, config.steerMotorId);
        assertEquals(15, config.encoderId);
        assertEquals(0.125, config.encoderOffset.in(Rotations), 0.0001);
        assertEquals(12.0, config.xPos.in(Inches), 0.0001);
        assertEquals(-12.0, config.yPos.in(Inches), 0.0001);

        // Default inversions should be false
        assertFalse(config.invertSide);
        assertFalse(config.steerMotorInverted);
        assertFalse(config.encoderInverted);
    }

    @Test
    void createTestSwerveModuleConfig_withInversions_setsInversionFlags() {
        SwerveModuleConfig config = TestFixtures.createTestSwerveModuleConfig(
                1, 2, 10, 0.0, 10.0, 10.0, true, true, true // All inverted
                );

        assertTrue(config.invertSide);
        assertTrue(config.steerMotorInverted);
        assertTrue(config.encoderInverted);
    }

    @Test
    void drivetrainTestBase_providesModuleAccessors() {
        // Verify the base class provides working accessors
        assertNotNull(getFrontLeftModule());
        assertNotNull(getFrontRightModule());
        assertNotNull(getBackLeftModule());
        assertNotNull(getBackRightModule());

        // Verify they return different modules (based on position)
        assertNotEquals(
                getFrontLeftModule().yPos.in(Inches),
                getFrontRightModule().yPos.in(Inches),
                "Front left and front right should have different Y positions");
    }
}
