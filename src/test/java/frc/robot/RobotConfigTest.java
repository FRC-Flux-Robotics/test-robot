package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/**
 * Unit tests for RobotConfig.
 * Verifies multi-robot configuration structures.
 */
class RobotConfigTest {

    @Test
    void constructor_setsAllFields() {
        SwerveModuleConfig fl = new SwerveModuleConfig(1, 2, 10, 0.0, 10.0, 10.0, false, false, false);
        SwerveModuleConfig fr = new SwerveModuleConfig(3, 4, 11, 0.0, 10.0, -10.0, true, false, false);
        SwerveModuleConfig bl = new SwerveModuleConfig(5, 6, 12, 0.0, -10.0, 10.0, false, false, false);
        SwerveModuleConfig br = new SwerveModuleConfig(7, 8, 13, 0.0, -10.0, -10.0, true, false, false);

        RobotConfig config = new RobotConfig("TestBus", "rio", 20, fl, fr, bl, br);

        assertEquals("TestBus", config.driveCANBus);
        assertEquals("rio", config.systemCANBus);
        assertEquals(20, config.pigeonId);
        assertSame(fl, config.frontLeft);
        assertSame(fr, config.frontRight);
        assertSame(bl, config.backLeft);
        assertSame(br, config.backRight);
    }

    @Test
    void testRobot_hasValidConfiguration() {
        RobotConfig test = RobotConfig.TestRobot;

        assertNotNull(test);
        assertEquals("Drivetrain", test.driveCANBus);
        assertEquals(20, test.pigeonId);

        // Verify all four modules are present
        assertNotNull(test.frontLeft);
        assertNotNull(test.frontRight);
        assertNotNull(test.backLeft);
        assertNotNull(test.backRight);

        // Verify CAN IDs are unique across modules
        int[] encoderIds = {
            test.frontLeft.encoderId,
            test.frontRight.encoderId,
            test.backLeft.encoderId,
            test.backRight.encoderId
        };
        assertEquals(4, java.util.Arrays.stream(encoderIds).distinct().count(),
            "Encoder IDs should be unique");
    }

    @Test
    void practiceRobot_hasValidConfiguration() {
        RobotConfig practice = RobotConfig.PracticeRobot;

        assertNotNull(practice);
        assertEquals("CANdace", practice.driveCANBus);
        assertEquals(24, practice.pigeonId);

        // Verify all four modules are present
        assertNotNull(practice.frontLeft);
        assertNotNull(practice.frontRight);
        assertNotNull(practice.backLeft);
        assertNotNull(practice.backRight);

        // Verify CAN IDs are unique across modules
        int[] driveIds = {
            practice.frontLeft.driveMotorId,
            practice.frontRight.driveMotorId,
            practice.backLeft.driveMotorId,
            practice.backRight.driveMotorId
        };
        assertEquals(4, java.util.Arrays.stream(driveIds).distinct().count(),
            "Drive motor IDs should be unique");
    }

    @Test
    void invertSideConstants_areOpposite() {
        // Left side should not be inverted, right side should be
        assertFalse(RobotConfig.InvertLeftSide);
        assertTrue(RobotConfig.InvertRightSide);
    }
}
