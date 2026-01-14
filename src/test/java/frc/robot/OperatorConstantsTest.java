package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants.OperatorConstants;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for OperatorConstants.
 * Validates that joystick input processing constants are within valid ranges.
 */
class OperatorConstantsTest {

    @Test
    void deadzoneValues_position_areInValidRange() {
        // xStart should be small positive (deadzone threshold)
        assertTrue(OperatorConstants.xStartPos >= 0.0, "xStartPos should be non-negative");
        assertTrue(OperatorConstants.xStartPos < 0.5, "xStartPos should be less than 0.5 (reasonable deadzone)");

        // xMiddle should be between xStart and 1.0
        assertTrue(
                OperatorConstants.xMiddlePos > OperatorConstants.xStartPos,
                "xMiddlePos should be greater than xStartPos");
        assertTrue(OperatorConstants.xMiddlePos <= 1.0, "xMiddlePos should be at most 1.0");
    }

    @Test
    void deadzoneValues_rotation_areInValidRange() {
        // Same constraints apply to rotation parameters
        assertTrue(OperatorConstants.xStartRot >= 0.0, "xStartRot should be non-negative");
        assertTrue(OperatorConstants.xStartRot < 0.5, "xStartRot should be less than 0.5");

        assertTrue(
                OperatorConstants.xMiddleRot > OperatorConstants.xStartRot,
                "xMiddleRot should be greater than xStartRot");
        assertTrue(OperatorConstants.xMiddleRot <= 1.0, "xMiddleRot should be at most 1.0");
    }

    @Test
    void sensitivityValues_position_areInValidRange() {
        // yStart should be small positive
        assertTrue(OperatorConstants.yStartPos >= 0.0, "yStartPos should be non-negative");
        assertTrue(OperatorConstants.yStartPos <= 1.0, "yStartPos should be at most 1.0");

        // yMiddle should be between yStart and yMax
        assertTrue(OperatorConstants.yMiddlePos >= OperatorConstants.yStartPos, "yMiddlePos should be >= yStartPos");
        assertTrue(OperatorConstants.yMiddlePos <= OperatorConstants.yMaxPos, "yMiddlePos should be <= yMaxPos");

        // yMax should be reasonable (not exceed 1.0 for safety)
        assertTrue(OperatorConstants.yMaxPos > 0.0, "yMaxPos should be positive");
        assertTrue(OperatorConstants.yMaxPos <= 1.0, "yMaxPos should not exceed 1.0 (full speed)");
    }

    @Test
    void sensitivityValues_rotation_areInValidRange() {
        assertTrue(OperatorConstants.yStartRot >= 0.0, "yStartRot should be non-negative");

        assertTrue(OperatorConstants.yMiddleRot >= OperatorConstants.yStartRot, "yMiddleRot should be >= yStartRot");
        assertTrue(OperatorConstants.yMiddleRot <= OperatorConstants.yMaxRot, "yMiddleRot should be <= yMaxRot");

        assertTrue(OperatorConstants.yMaxRot > 0.0, "yMaxRot should be positive");
        assertTrue(OperatorConstants.yMaxRot <= 1.5, "yMaxRot should be reasonable (<=1.5)");
    }

    @Test
    void controllerPorts_areValid() {
        // FRC driver station supports ports 0-5
        assertTrue(OperatorConstants.DriverControllerPort >= 0, "DriverControllerPort should be >= 0");
        assertTrue(OperatorConstants.DriverControllerPort <= 5, "DriverControllerPort should be <= 5");

        assertTrue(OperatorConstants.OperatorControllerPort >= 0, "OperatorControllerPort should be >= 0");
        assertTrue(OperatorConstants.OperatorControllerPort <= 5, "OperatorControllerPort should be <= 5");

        // Ports should be different if using two controllers
        if (OperatorConstants.UseTwoControllers) {
            assertNotEquals(
                    OperatorConstants.DriverControllerPort,
                    OperatorConstants.OperatorControllerPort,
                    "Driver and Operator ports should be different when using two controllers");
        }
    }

    @Test
    void triggerThreshold_isValid() {
        // Trigger threshold should be between 0 and 1
        assertTrue(OperatorConstants.TriggerThreshold >= 0.0, "TriggerThreshold should be non-negative");
        assertTrue(OperatorConstants.TriggerThreshold <= 1.0, "TriggerThreshold should be at most 1.0");

        // Should have some threshold to prevent accidental activation
        assertTrue(
                OperatorConstants.TriggerThreshold >= 0.05,
                "TriggerThreshold should be at least 0.05 to prevent accidental triggers");
    }

    @Test
    void sensitivityCoefficients_areValid() {
        // LinCoef should be positive and reasonable
        assertTrue(OperatorConstants.LinCoef > 0.0, "LinCoef should be positive");
        assertTrue(OperatorConstants.LinCoef <= 1.0, "LinCoef should be at most 1.0");

        // CuspX should be between 0 and 1
        assertTrue(OperatorConstants.CuspX >= 0.0, "CuspX should be non-negative");
        assertTrue(OperatorConstants.CuspX <= 1.0, "CuspX should be at most 1.0");

        // MinLimit should be reasonable
        assertTrue(OperatorConstants.MinLimit >= 0.0, "MinLimit should be non-negative");
        assertTrue(OperatorConstants.MinLimit <= 1.0, "MinLimit should be at most 1.0");
    }

    @Test
    void rotationCoefficients_areValid() {
        assertTrue(OperatorConstants.RotLinCoef > 0.0, "RotLinCoef should be positive");
        assertTrue(OperatorConstants.RotLinCoef <= 1.0, "RotLinCoef should be at most 1.0");

        assertTrue(OperatorConstants.RotCuspX >= 0.0, "RotCuspX should be non-negative");
        assertTrue(OperatorConstants.RotCuspX <= 1.0, "RotCuspX should be at most 1.0");
    }
}
