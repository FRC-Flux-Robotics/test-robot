package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/**
 * Unit tests for Constants class.
 * Validates that configuration values are within expected ranges.
 */
class ConstantsTest {

    @Test
    void driveConstants_autoModeSpeed_isNegative() {
        // Auto mode drives backward to driver station
        assertTrue(Constants.DriveConstants.AutoModeSpeed < 0,
            "AutoModeSpeed should be negative for backward movement");
    }

    @Test
    void driveConstants_autoModeDriveTime_isPositive() {
        assertTrue(Constants.DriveConstants.AutoModeDriveTime > 0,
            "AutoModeDriveTime should be positive");
    }

    @Test
    void operatorConstants_controllerPorts_areValid() {
        assertTrue(Constants.OperatorConstants.DriverControllerPort >= 0,
            "Driver controller port should be non-negative");
        assertTrue(Constants.OperatorConstants.OperatorControllerPort >= 0,
            "Operator controller port should be non-negative");
    }

    @Test
    void operatorConstants_speedLimits_areWithinValidRange() {
        // Speed limits should be between 0 and 1
        double speedLimitX = Constants.OperatorConstants.SpeedLimitX;
        double speedLimitRot = Constants.OperatorConstants.SpeedLimitRot;

        assertTrue(speedLimitX > 0 && speedLimitX <= 1.0,
            "SpeedLimitX should be between 0 and 1, was: " + speedLimitX);
        assertTrue(speedLimitRot > 0 && speedLimitRot <= 1.0,
            "SpeedLimitRot should be between 0 and 1, was: " + speedLimitRot);
    }

    @Test
    void operatorConstants_triggerThreshold_isValid() {
        double threshold = Constants.OperatorConstants.TriggerThreshold;
        assertTrue(threshold >= 0 && threshold < 1.0,
            "TriggerThreshold should be between 0 and 1");
    }
}
