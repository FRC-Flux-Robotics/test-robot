package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for Sensitivity.
 * Verifies joystick sensitivity with threshold and cusp behavior.
 */
class SensitivityTest {

    private Sensitivity sensitivity;

    // Standard test configuration:
    // threshold=0.1, cuspX=0.5, linCoef=0.3, limit=0.8
    @BeforeEach
    void setUp() {
        sensitivity = new Sensitivity(0.1, 0.5, 0.3, 0.8);
    }

    @Test
    void transfer_belowThreshold_returnsZero() {
        // Input below threshold should return 0
        assertEquals(0.0, sensitivity.transfer(0.05), 0.0001);
        assertEquals(0.0, sensitivity.transfer(0.0), 0.0001);
        assertEquals(0.0, sensitivity.transfer(0.09), 0.0001);
    }

    @Test
    void transfer_atThreshold_returnsLinear() {
        // At exactly threshold, should use linear coefficient
        // For input = 0.1 with linCoef = 0.3: output = 0.1 * 0.3 = 0.03
        double result = sensitivity.transfer(0.1);
        assertEquals(0.03, result, 0.0001);
    }

    @Test
    void transfer_belowCusp_usesLinearCoefficient() {
        // Below cuspX, output = input * linCoef
        // For input = 0.4 with linCoef = 0.3: output = 0.4 * 0.3 = 0.12
        double result = sensitivity.transfer(0.4);
        assertEquals(0.12, result, 0.0001);
    }

    @Test
    void transfer_aboveCusp_usesQuadraticCurve() {
        // Above cuspX, uses quadratic formula: a*x^2 + b*x + c
        // The exact values depend on the coefficients calculated in set()
        double resultAtCusp = sensitivity.transfer(0.5);
        double resultAboveCusp = sensitivity.transfer(0.7);

        // Result should be between linear output at cusp and limit
        assertTrue(resultAboveCusp > resultAtCusp,
            "Output above cusp should be greater than at cusp");
        assertTrue(resultAboveCusp <= 0.8,
            "Output should not exceed limit");
    }

    @Test
    void transfer_exceedsLimit_clampsToLimit() {
        // Output should never exceed the limit
        double result = sensitivity.transfer(1.0);
        assertTrue(result <= 0.8,
            "Output should be clamped to limit of 0.8, was: " + result);
    }

    @Test
    void transfer_negativeInput_returnsNegativeOutput() {
        // Negative inputs should produce negative outputs
        double positiveResult = sensitivity.transfer(0.4);
        double negativeResult = sensitivity.transfer(-0.4);

        assertEquals(-positiveResult, negativeResult, 0.0001);
    }

    @Test
    void transfer_negativeThreshold_shiftsDeadzone() {
        // Negative threshold creates different behavior - shifts the input
        Sensitivity negThreshold = new Sensitivity(-0.1, 0.5, 0.3, 0.8);

        // With negative threshold, small inputs get shifted
        double result = negThreshold.transfer(0.05);
        // 0.05 - (-0.1) = 0.15, which is > 0, so not zero
        assertTrue(result > 0, "Negative threshold should shift deadzone");
    }

    @Test
    void minLimit_valueAboveMin_returnsValue() {
        // Values above minValue should pass through unchanged
        assertEquals(0.5, Sensitivity.minLimit(0.5, 0.3), 0.0001);
        assertEquals(-0.5, Sensitivity.minLimit(-0.5, 0.3), 0.0001);
    }

    @Test
    void minLimit_valueBelowMin_returnsMin() {
        // Non-zero values below minValue should be boosted to minValue
        assertEquals(0.3, Sensitivity.minLimit(0.1, 0.3), 0.0001);
        assertEquals(-0.3, Sensitivity.minLimit(-0.1, 0.3), 0.0001);
    }

    @Test
    void minLimit_zeroValue_returnsZero() {
        // Zero should remain zero (not boosted to min)
        assertEquals(0.0, Sensitivity.minLimit(0.0, 0.3), 0.0001);
    }

    @Test
    void set_updatesAllParameters() {
        // After calling set with new parameters, behavior should change
        sensitivity.set(0.2, 0.6, 0.4, 1.0);

        // Below new threshold
        assertEquals(0.0, sensitivity.transfer(0.15), 0.0001);

        // At new threshold with new linCoef
        assertEquals(0.08, sensitivity.transfer(0.2), 0.0001); // 0.2 * 0.4 = 0.08
    }
}
