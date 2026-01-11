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
    void coralRobot_hasValidConfiguration() {
        RobotConfig coral = RobotConfig.CoralRobot;

        assertNotNull(coral);
        assertEquals("CANdace", coral.driveCANBus);
        assertEquals(24, coral.pigeonId);

        // Verify all four modules are present
        assertNotNull(coral.frontLeft);
        assertNotNull(coral.frontRight);
        assertNotNull(coral.backLeft);
        assertNotNull(coral.backRight);

        // Verify CAN IDs are unique across modules
        int[] driveIds = {
            coral.frontLeft.driveMotorId,
            coral.frontRight.driveMotorId,
            coral.backLeft.driveMotorId,
            coral.backRight.driveMotorId
        };
        assertEquals(4, java.util.Arrays.stream(driveIds).distinct().count(),
            "Drive motor IDs should be unique");
    }

    @Test
    void algaeRobot_hasValidConfiguration() {
        RobotConfig algae = RobotConfig.AlgaeRobot;

        assertNotNull(algae);
        assertEquals("Drivetrain", algae.driveCANBus);
        assertEquals(20, algae.pigeonId);

        // Verify all four modules are present
        assertNotNull(algae.frontLeft);
        assertNotNull(algae.frontRight);
        assertNotNull(algae.backLeft);
        assertNotNull(algae.backRight);

        // Verify CAN IDs are unique across modules
        int[] encoderIds = {
            algae.frontLeft.encoderId,
            algae.frontRight.encoderId,
            algae.backLeft.encoderId,
            algae.backRight.encoderId
        };
        assertEquals(4, java.util.Arrays.stream(encoderIds).distinct().count(),
            "Encoder IDs should be unique");
    }

    @Test
    void invertSideConstants_areOpposite() {
        // Left side should not be inverted, right side should be
        assertFalse(RobotConfig.InvertLeftSide);
        assertTrue(RobotConfig.InvertRightSide);
    }
}
