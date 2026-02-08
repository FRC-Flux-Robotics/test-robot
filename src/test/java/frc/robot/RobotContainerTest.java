package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for RobotContainer configuration.
 *
 * <p>Note: RobotContainer creates real Phoenix 6 hardware in its constructor,
 * making full instantiation testing impractical without simulation.
 * These tests validate configuration logic and constants.
 */
class RobotContainerTest {

    @BeforeAll
    static void initHAL() {
        TestFixtures.initializeHAL();
    }

    // ========== Configuration Validation Tests ==========

    @Test
    void coralRobotConfig_isValid() {
        RobotConfig config = RobotConfig.CoralRobot;

        assertNotNull(config, "CoralRobot config should exist");
        assertEquals("Drivetrain", config.driveCANBus, "CoralRobot should use Drivetrain CAN bus");
        assertEquals("rio", config.systemCANBus, "CoralRobot should use rio system CAN bus");
        assertEquals(20, config.pigeonId, "CoralRobot Pigeon ID should be 20");
    }

    @Test
    void practiceRobotConfig_isValid() {
        RobotConfig config = RobotConfig.PracticeRobot;

        assertNotNull(config, "PracticeRobot config should exist");
        assertEquals("CANdace", config.driveCANBus, "PracticeRobot should use CANdace CAN bus");
        assertEquals("rio", config.systemCANBus, "PracticeRobot should use rio system CAN bus");
        assertEquals(24, config.pigeonId, "PracticeRobot Pigeon ID should be 24");
    }

    @Test
    void coralRobotConfig_hasFourModules() {
        RobotConfig config = RobotConfig.CoralRobot;

        assertNotNull(config.frontLeft, "CoralRobot should have front left module");
        assertNotNull(config.frontRight, "CoralRobot should have front right module");
        assertNotNull(config.backLeft, "CoralRobot should have back left module");
        assertNotNull(config.backRight, "CoralRobot should have back right module");
    }

    @Test
    void practiceRobotConfig_hasFourModules() {
        RobotConfig config = RobotConfig.PracticeRobot;

        assertNotNull(config.frontLeft, "PracticeRobot should have front left module");
        assertNotNull(config.frontRight, "PracticeRobot should have front right module");
        assertNotNull(config.backLeft, "PracticeRobot should have back left module");
        assertNotNull(config.backRight, "PracticeRobot should have back right module");
    }

    @Test
    void coralRobotConfig_modulePositionsAreSymmetric() {
        RobotConfig config = RobotConfig.CoralRobot;

        // Front modules should have positive X
        assertTrue(config.frontLeft.xPos.in(Inches) > 0, "Front left X should be positive");
        assertTrue(config.frontRight.xPos.in(Inches) > 0, "Front right X should be positive");

        // Back modules should have negative X
        assertTrue(config.backLeft.xPos.in(Inches) < 0, "Back left X should be negative");
        assertTrue(config.backRight.xPos.in(Inches) < 0, "Back right X should be negative");

        // Left modules should have positive Y
        assertTrue(config.frontLeft.yPos.in(Inches) > 0, "Front left Y should be positive");
        assertTrue(config.backLeft.yPos.in(Inches) > 0, "Back left Y should be positive");

        // Right modules should have negative Y
        assertTrue(config.frontRight.yPos.in(Inches) < 0, "Front right Y should be negative");
        assertTrue(config.backRight.yPos.in(Inches) < 0, "Back right Y should be negative");
    }

    // ========== DrivetrainConstants Builder Tests ==========

    @Test
    void drivetrainConstants_canBeBuiltFromCoralConfig() {
        RobotConfig config = RobotConfig.CoralRobot;

        SwerveDrivetrainConstants constants = new SwerveDrivetrainConstants()
                .withCANBusName(config.driveCANBus)
                .withPigeon2Id(config.pigeonId);

        assertNotNull(constants, "SwerveDrivetrainConstants should be created");
        assertEquals("Drivetrain", constants.CANBusName, "CAN bus name should match config");
        assertEquals(20, constants.Pigeon2Id, "Pigeon ID should match config");
    }

    @Test
    void drivetrainConstants_canBeBuiltFromPracticeConfig() {
        RobotConfig config = RobotConfig.PracticeRobot;

        SwerveDrivetrainConstants constants = new SwerveDrivetrainConstants()
                .withCANBusName(config.driveCANBus)
                .withPigeon2Id(config.pigeonId);

        assertNotNull(constants, "SwerveDrivetrainConstants should be created");
        assertEquals("CANdace", constants.CANBusName, "CAN bus name should match config");
        assertEquals(24, constants.Pigeon2Id, "Pigeon ID should match config");
    }

    @Test
    void drivetrainConstants_canBeBuiltFromTestConfig() {
        RobotConfig config = TestFixtures.createTestRobotConfig();

        SwerveDrivetrainConstants constants = new SwerveDrivetrainConstants()
                .withCANBusName(config.driveCANBus)
                .withPigeon2Id(config.pigeonId);

        assertNotNull(constants, "SwerveDrivetrainConstants should be created from test config");
        assertEquals("TestBus", constants.CANBusName, "CAN bus name should match test config");
        assertEquals(1, constants.Pigeon2Id, "Pigeon ID should match test config");
    }

    // ========== Motor Inversion Configuration Tests ==========

    @Test
    void invertLeftSide_isFalse() {
        assertFalse(RobotConfig.InvertLeftSide, "Left side should not be inverted");
    }

    @Test
    void invertRightSide_isTrue() {
        assertTrue(RobotConfig.InvertRightSide, "Right side should be inverted");
    }

    @Test
    void coralRobot_leftModulesUseLeftInversion() {
        RobotConfig config = RobotConfig.CoralRobot;

        assertEquals(RobotConfig.InvertLeftSide, config.frontLeft.invertSide, "Front left should use left inversion");
        assertEquals(RobotConfig.InvertLeftSide, config.backLeft.invertSide, "Back left should use left inversion");
    }

    @Test
    void coralRobot_rightModulesUseRightInversion() {
        RobotConfig config = RobotConfig.CoralRobot;

        assertEquals(
                RobotConfig.InvertRightSide, config.frontRight.invertSide, "Front right should use right inversion");
        assertEquals(RobotConfig.InvertRightSide, config.backRight.invertSide, "Back right should use right inversion");
    }
}
