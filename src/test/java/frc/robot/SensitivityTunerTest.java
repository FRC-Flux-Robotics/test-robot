package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for SensitivityTuner.
 * Verifies SmartDashboard-integrated sensitivity curve tuning.
 */
class SensitivityTunerTest {

    @BeforeAll
    static void setUpClass() {
        TestFixtures.initializeHAL();
    }

    @Test
    void transfer_appliesSensitivityCurve() {
        SensitivityTuner tuner = new SensitivityTuner("Test_",
            0.1, 0.5, 0.1, 0.5, 1.0);

        // Below deadzone should return 0
        assertEquals(0.0, tuner.transfer(0.05), 0.0001);

        // At xStart should return yStart
        assertEquals(0.1, tuner.transfer(0.1), 0.0001);

        // At full input should return yMax
        assertEquals(1.0, tuner.transfer(1.0), 0.0001);
    }

    @Test
    void transfer_handlesNegativeInput() {
        SensitivityTuner tuner = new SensitivityTuner("Neg_",
            0.1, 0.5, 0.1, 0.5, 1.0);

        // Negative below deadzone
        assertEquals(0.0, tuner.transfer(-0.05), 0.0001);

        // Negative at full should return -yMax
        assertEquals(-1.0, tuner.transfer(-1.0), 0.0001);
    }

    @Test
    void constructor_publishesToSmartDashboard() {
        // Create with unique prefix to avoid conflicts
        SensitivityTuner tuner = new SensitivityTuner("Unique_",
            0.02, 0.6, 0.1, 0.4, 0.8);

        // Just verify it doesn't throw - SmartDashboard is initialized
        double result = tuner.transfer(0.5);
        assertTrue(result > 0.0, "Should return positive value for positive input");
    }

    @Test
    void multipleInstances_withDifferentPrefixes_coexist() {
        SensitivityTuner tuner1 = new SensitivityTuner("A_",
            0.1, 0.5, 0.1, 0.5, 1.0);
        SensitivityTuner tuner2 = new SensitivityTuner("B_",
            0.2, 0.6, 0.2, 0.4, 0.8);

        // Each should behave according to its own parameters
        double result1 = tuner1.transfer(0.15);
        double result2 = tuner2.transfer(0.15);

        // tuner1 should output (0.15 is above its 0.1 deadzone)
        assertTrue(result1 > 0.0);
        // tuner2 should output 0 (0.15 is below its 0.2 deadzone)
        assertEquals(0.0, result2, 0.0001);
    }
}
