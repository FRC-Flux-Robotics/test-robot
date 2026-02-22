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
        RobotConfig test = RobotConfig.CoralRobot;

        assertNotNull(test);
        assertEquals("CANdace", test.driveCANBus);
        assertEquals(24, test.pigeonId);

        // Verify all four modules are present
        assertNotNull(test.frontLeft);
        assertNotNull(test.frontRight);
        assertNotNull(test.backLeft);
        assertNotNull(test.backRight);

        // Verify CAN IDs are unique across modules
        int[] encoderIds = {
            test.frontLeft.encoderId, test.frontRight.encoderId, test.backLeft.encoderId, test.backRight.encoderId
        };
        assertEquals(4, java.util.Arrays.stream(encoderIds).distinct().count(), "Encoder IDs should be unique");
    }

    @Test
    void invertSideConstants_areOpposite() {
        // Left side should not be inverted, right side should be
        assertFalse(RobotConfig.InvertLeftSide);
        assertTrue(RobotConfig.InvertRightSide);
    }

    // Validation tests for CAN bus names

    @Test
    void constructor_rejectsNullDriveCANBus() {
        SwerveModuleConfig module = createValidModule();
        IllegalArgumentException ex = assertThrows(
                IllegalArgumentException.class, () -> new RobotConfig(null, "rio", 20, module, module, module, module));
        assertTrue(ex.getMessage().contains("driveCANBus"));
    }

    @Test
    void constructor_rejectsEmptyDriveCANBus() {
        SwerveModuleConfig module = createValidModule();
        IllegalArgumentException ex = assertThrows(
                IllegalArgumentException.class, () -> new RobotConfig("", "rio", 20, module, module, module, module));
        assertTrue(ex.getMessage().contains("driveCANBus"));
    }

    @Test
    void constructor_rejectsNullSystemCANBus() {
        SwerveModuleConfig module = createValidModule();
        IllegalArgumentException ex = assertThrows(
                IllegalArgumentException.class,
                () -> new RobotConfig("Drivetrain", null, 20, module, module, module, module));
        assertTrue(ex.getMessage().contains("systemCANBus"));
    }

    @Test
    void constructor_rejectsEmptySystemCANBus() {
        SwerveModuleConfig module = createValidModule();
        IllegalArgumentException ex = assertThrows(
                IllegalArgumentException.class,
                () -> new RobotConfig("Drivetrain", "", 20, module, module, module, module));
        assertTrue(ex.getMessage().contains("systemCANBus"));
    }

    // Validation tests for Pigeon ID

    @Test
    void constructor_rejectsNegativePigeonId() {
        SwerveModuleConfig module = createValidModule();
        IllegalArgumentException ex = assertThrows(
                IllegalArgumentException.class,
                () -> new RobotConfig("Drivetrain", "rio", -1, module, module, module, module));
        assertTrue(ex.getMessage().contains("pigeonId"));
    }

    @Test
    void constructor_rejectsTooHighPigeonId() {
        SwerveModuleConfig module = createValidModule();
        IllegalArgumentException ex = assertThrows(
                IllegalArgumentException.class,
                () -> new RobotConfig("Drivetrain", "rio", 63, module, module, module, module));
        assertTrue(ex.getMessage().contains("pigeonId"));
    }

    @Test
    void constructor_acceptsBoundaryPigeonId() {
        SwerveModuleConfig module = createValidModule();
        // 0 and 62 should be accepted
        RobotConfig minId = new RobotConfig("Drivetrain", "rio", 0, module, module, module, module);
        assertEquals(0, minId.pigeonId);

        RobotConfig maxId = new RobotConfig("Drivetrain", "rio", 62, module, module, module, module);
        assertEquals(62, maxId.pigeonId);
    }

    // Validation tests for module configs

    @Test
    void constructor_rejectsNullFrontLeft() {
        SwerveModuleConfig module = createValidModule();
        IllegalArgumentException ex = assertThrows(
                IllegalArgumentException.class,
                () -> new RobotConfig("Drivetrain", "rio", 20, null, module, module, module));
        assertTrue(ex.getMessage().contains("frontLeft"));
    }

    @Test
    void constructor_rejectsNullFrontRight() {
        SwerveModuleConfig module = createValidModule();
        IllegalArgumentException ex = assertThrows(
                IllegalArgumentException.class,
                () -> new RobotConfig("Drivetrain", "rio", 20, module, null, module, module));
        assertTrue(ex.getMessage().contains("frontRight"));
    }

    @Test
    void constructor_rejectsNullBackLeft() {
        SwerveModuleConfig module = createValidModule();
        IllegalArgumentException ex = assertThrows(
                IllegalArgumentException.class,
                () -> new RobotConfig("Drivetrain", "rio", 20, module, module, null, module));
        assertTrue(ex.getMessage().contains("backLeft"));
    }

    @Test
    void constructor_rejectsNullBackRight() {
        SwerveModuleConfig module = createValidModule();
        IllegalArgumentException ex = assertThrows(
                IllegalArgumentException.class,
                () -> new RobotConfig("Drivetrain", "rio", 20, module, module, module, null));
        assertTrue(ex.getMessage().contains("backRight"));
    }

    private SwerveModuleConfig createValidModule() {
        return new SwerveModuleConfig(1, 2, 10, 0.0, 10.0, 10.0, false, false, false);
    }
}
