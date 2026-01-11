package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.jupiter.api.Assertions.*;

import java.util.HashSet;
import java.util.Set;

import org.junit.jupiter.api.Test;

import frc.robot.generated.TunerConstants;

/**
 * Unit tests for TunerConstants (Phoenix 6 generated swerve configuration).
 * Validates module constants, CAN ID uniqueness, and configuration values.
 */
class TunerConstantsTest {

    @Test
    void allModuleConstants_areNotNull() {
        assertNotNull(TunerConstants.FrontLeft, "FrontLeft module should not be null");
        assertNotNull(TunerConstants.FrontRight, "FrontRight module should not be null");
        assertNotNull(TunerConstants.BackLeft, "BackLeft module should not be null");
        assertNotNull(TunerConstants.BackRight, "BackRight module should not be null");
    }

    @Test
    void driveMotorIds_areUnique() {
        Set<Integer> driveIds = new HashSet<>();
        driveIds.add(TunerConstants.FrontLeft.DriveMotorId);
        driveIds.add(TunerConstants.FrontRight.DriveMotorId);
        driveIds.add(TunerConstants.BackLeft.DriveMotorId);
        driveIds.add(TunerConstants.BackRight.DriveMotorId);

        assertEquals(4, driveIds.size(), "All 4 drive motor IDs should be unique");
    }

    @Test
    void steerMotorIds_areUnique() {
        Set<Integer> steerIds = new HashSet<>();
        steerIds.add(TunerConstants.FrontLeft.SteerMotorId);
        steerIds.add(TunerConstants.FrontRight.SteerMotorId);
        steerIds.add(TunerConstants.BackLeft.SteerMotorId);
        steerIds.add(TunerConstants.BackRight.SteerMotorId);

        assertEquals(4, steerIds.size(), "All 4 steer motor IDs should be unique");
    }

    @Test
    void encoderIds_areUnique() {
        Set<Integer> encoderIds = new HashSet<>();
        encoderIds.add(TunerConstants.FrontLeft.EncoderId);
        encoderIds.add(TunerConstants.FrontRight.EncoderId);
        encoderIds.add(TunerConstants.BackLeft.EncoderId);
        encoderIds.add(TunerConstants.BackRight.EncoderId);

        assertEquals(4, encoderIds.size(), "All 4 encoder IDs should be unique");
    }

    @Test
    void speedAt12Volts_isPositiveAndReasonable() {
        double speedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        assertTrue(speedMps > 0, "Speed should be positive");
        assertTrue(speedMps < 10, "Speed should be less than 10 m/s (reasonable max for FRC)");
        assertTrue(speedMps > 1, "Speed should be greater than 1 m/s (reasonable min)");
    }

    @Test
    void inversionConstants_leftAndRightAreOpposite() {
        assertFalse(TunerConstants.kInvertLeftSide, "Left side should not be inverted");
        assertTrue(TunerConstants.kInvertRightSide, "Right side should be inverted");
    }

    @Test
    void modulePositions_formSymmetricRectangle() {
        // Get module positions (in meters from TunerConstants)
        double flX = TunerConstants.FrontLeft.LocationX;
        double flY = TunerConstants.FrontLeft.LocationY;
        double frX = TunerConstants.FrontRight.LocationX;
        double frY = TunerConstants.FrontRight.LocationY;
        double blX = TunerConstants.BackLeft.LocationX;
        double blY = TunerConstants.BackLeft.LocationY;
        double brX = TunerConstants.BackRight.LocationX;
        double brY = TunerConstants.BackRight.LocationY;

        // Front modules should have same X (positive)
        assertEquals(flX, frX, 0.001, "Front modules should have same X position");
        assertTrue(flX > 0, "Front modules should have positive X");

        // Back modules should have same X (negative)
        assertEquals(blX, brX, 0.001, "Back modules should have same X position");
        assertTrue(blX < 0, "Back modules should have negative X");

        // Left modules should have same Y (positive)
        assertEquals(flY, blY, 0.001, "Left modules should have same Y position");
        assertTrue(flY > 0, "Left modules should have positive Y");

        // Right modules should have same Y (negative)
        assertEquals(frY, brY, 0.001, "Right modules should have same Y position");
        assertTrue(frY < 0, "Right modules should have negative Y");

        // Rectangle should be symmetric around center
        assertEquals(Math.abs(flX), Math.abs(blX), 0.001, "Rectangle should be symmetric in X");
        assertEquals(Math.abs(flY), Math.abs(frY), 0.001, "Rectangle should be symmetric in Y");
    }

    @Test
    void constantCreator_isNotNull() {
        assertNotNull(TunerConstants.ConstantCreator, "ConstantCreator factory should not be null");
    }
}
