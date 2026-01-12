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

    @Test
    void encoderOffset_handlesNegativeValues() {
        double negativeOffset = -0.371826171875;
        SwerveModuleConfig config = new SwerveModuleConfig(
            1, 2, 3,
            negativeOffset,
            0.0, 0.0,
            false, false, false
        );

        assertEquals(negativeOffset, config.encoderOffset.in(Rotations), 0.0001);
    }

    @Test
    void constructor_handlesZeroValues() {
        SwerveModuleConfig config = new SwerveModuleConfig(
            0, 0, 0,
            0.0,
            0.0, 0.0,
            false, false, false
        );

        assertEquals(0, config.driveMotorId);
        assertEquals(0, config.steerMotorId);
        assertEquals(0, config.encoderId);
        assertEquals(0.0, config.encoderOffset.in(Rotations), 0.0001);
        assertEquals(0.0, config.xPos.in(Inches), 0.0001);
        assertEquals(0.0, config.yPos.in(Inches), 0.0001);
    }

    @Test
    void inversionFlags_allCombinations() {
        // All false
        SwerveModuleConfig allFalse = new SwerveModuleConfig(
            1, 2, 3, 0.0, 0.0, 0.0,
            false, false, false
        );
        assertFalse(allFalse.invertSide);
        assertFalse(allFalse.steerMotorInverted);
        assertFalse(allFalse.encoderInverted);

        // All true
        SwerveModuleConfig allTrue = new SwerveModuleConfig(
            1, 2, 3, 0.0, 0.0, 0.0,
            true, true, true
        );
        assertTrue(allTrue.invertSide);
        assertTrue(allTrue.steerMotorInverted);
        assertTrue(allTrue.encoderInverted);

        // Mixed - typical right side module
        SwerveModuleConfig rightSide = new SwerveModuleConfig(
            1, 2, 3, 0.0, 0.0, 0.0,
            true, false, false
        );
        assertTrue(rightSide.invertSide);
        assertFalse(rightSide.steerMotorInverted);
        assertFalse(rightSide.encoderInverted);
    }

    @Test
    void position_handlesNegativeCoordinates() {
        // Back-right module typically has negative X and Y
        SwerveModuleConfig backRight = new SwerveModuleConfig(
            1, 2, 3,
            0.0,
            -11.5, -11.5,
            false, false, false
        );

        assertEquals(-11.5, backRight.xPos.in(Inches), 0.0001);
        assertEquals(-11.5, backRight.yPos.in(Inches), 0.0001);
    }

    // Validation tests for CAN IDs

    @Test
    void constructor_rejectsNegativeDriveMotorId() {
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class, () ->
            new SwerveModuleConfig(-1, 2, 3, 0.0, 0.0, 0.0, false, false, false)
        );
        assertTrue(ex.getMessage().contains("driveMotorId"));
    }

    @Test
    void constructor_rejectsTooHighDriveMotorId() {
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class, () ->
            new SwerveModuleConfig(63, 2, 3, 0.0, 0.0, 0.0, false, false, false)
        );
        assertTrue(ex.getMessage().contains("driveMotorId"));
    }

    @Test
    void constructor_rejectsNegativeSteerMotorId() {
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class, () ->
            new SwerveModuleConfig(1, -1, 3, 0.0, 0.0, 0.0, false, false, false)
        );
        assertTrue(ex.getMessage().contains("steerMotorId"));
    }

    @Test
    void constructor_rejectsTooHighSteerMotorId() {
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class, () ->
            new SwerveModuleConfig(1, 63, 3, 0.0, 0.0, 0.0, false, false, false)
        );
        assertTrue(ex.getMessage().contains("steerMotorId"));
    }

    @Test
    void constructor_rejectsNegativeEncoderId() {
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class, () ->
            new SwerveModuleConfig(1, 2, -1, 0.0, 0.0, 0.0, false, false, false)
        );
        assertTrue(ex.getMessage().contains("encoderId"));
    }

    @Test
    void constructor_rejectsTooHighEncoderId() {
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class, () ->
            new SwerveModuleConfig(1, 2, 63, 0.0, 0.0, 0.0, false, false, false)
        );
        assertTrue(ex.getMessage().contains("encoderId"));
    }

    // Validation tests for encoder offset

    @Test
    void constructor_rejectsEncoderOffsetTooLow() {
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class, () ->
            new SwerveModuleConfig(1, 2, 3, -1.1, 0.0, 0.0, false, false, false)
        );
        assertTrue(ex.getMessage().contains("encoderOffsetAngle"));
    }

    @Test
    void constructor_rejectsEncoderOffsetTooHigh() {
        IllegalArgumentException ex = assertThrows(IllegalArgumentException.class, () ->
            new SwerveModuleConfig(1, 2, 3, 1.1, 0.0, 0.0, false, false, false)
        );
        assertTrue(ex.getMessage().contains("encoderOffsetAngle"));
    }

    @Test
    void constructor_acceptsBoundaryEncoderOffset() {
        // -1.0 and 1.0 should be accepted (boundary values)
        SwerveModuleConfig minOffset = new SwerveModuleConfig(
            1, 2, 3, -1.0, 0.0, 0.0, false, false, false);
        assertEquals(-1.0, minOffset.encoderOffset.in(Rotations), 0.0001);

        SwerveModuleConfig maxOffset = new SwerveModuleConfig(
            1, 2, 3, 1.0, 0.0, 0.0, false, false, false);
        assertEquals(1.0, maxOffset.encoderOffset.in(Rotations), 0.0001);
    }

    @Test
    void constructor_acceptsBoundaryCanIds() {
        // 0 and 62 should be accepted (boundary values)
        SwerveModuleConfig minIds = new SwerveModuleConfig(
            0, 0, 0, 0.0, 0.0, 0.0, false, false, false);
        assertEquals(0, minIds.driveMotorId);

        SwerveModuleConfig maxIds = new SwerveModuleConfig(
            62, 62, 62, 0.0, 0.0, 0.0, false, false, false);
        assertEquals(62, maxIds.driveMotorId);
    }
}
