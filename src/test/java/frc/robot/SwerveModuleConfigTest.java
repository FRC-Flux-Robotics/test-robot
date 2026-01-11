package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/**
 * Unit tests for SwerveModuleConfig.
 * Verifies configuration data structure and unit conversions.
 */
class SwerveModuleConfigTest {

    @Test
    void constructor_setsAllFieldsCorrectly() {
        SwerveModuleConfig config = new SwerveModuleConfig(
            1, 2, 3,           // driveMotorId, steerMotorId, encoderId
            0.25,              // encoderOffsetAngle (rotations)
            10.0, -10.0,       // xPos, yPos (inches)
            true, false, true  // invertSide, steerMotorInverted, encoderInverted
        );

        assertEquals(1, config.driveMotorId);
        assertEquals(2, config.steerMotorId);
        assertEquals(3, config.encoderId);
        assertTrue(config.invertSide);
        assertFalse(config.steerMotorInverted);
        assertTrue(config.encoderInverted);
    }

    @Test
    void encoderOffset_convertsToRotations() {
        double offsetRotations = 0.5;
        SwerveModuleConfig config = new SwerveModuleConfig(
            1, 2, 3,
            offsetRotations,
            0.0, 0.0,
            false, false, false
        );

        assertEquals(offsetRotations, config.encoderOffset.in(Rotations), 0.0001);
    }

    @Test
    void position_convertsToInches() {
        double xInches = 11.5;
        double yInches = -11.5;
        SwerveModuleConfig config = new SwerveModuleConfig(
            1, 2, 3,
            0.0,
            xInches, yInches,
            false, false, false
        );

        assertEquals(xInches, config.xPos.in(Inches), 0.0001);
        assertEquals(yInches, config.yPos.in(Inches), 0.0001);
    }
}
