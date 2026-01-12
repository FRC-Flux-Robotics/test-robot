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

    // Edge case tests for joystick input processing

    @Test
    void transfer_withZeroThreshold_noDeadband() {
        // threshold=0 means no deadzone
        Sensitivity noDeadzone = new Sensitivity(0.0, 0.5, 0.3, 0.8);

        // Very small input should produce output
        double result = noDeadzone.transfer(0.01);
        assertEquals(0.003, result, 0.0001); // 0.01 * 0.3 = 0.003
    }

    @Test
    void transfer_withZeroCusp_allQuadratic() {
        // cuspX=0 means all inputs above threshold use quadratic
        Sensitivity allQuadratic = new Sensitivity(0.1, 0.0, 0.3, 0.8);

        // Any input above threshold should use quadratic formula
        double result = allQuadratic.transfer(0.5);
        // Result should be positive and within limit
        assertTrue(result > 0, "Output should be positive");
        assertTrue(result <= 0.8, "Output should be within limit");
    }

    @Test
    void transfer_withFullCusp_allLinear() {
        // cuspX=1.0 means all inputs use linear coefficient
        Sensitivity allLinear = new Sensitivity(0.1, 1.0, 0.5, 1.0);

        // All inputs should use linear scaling
        assertEquals(0.25, allLinear.transfer(0.5), 0.0001); // 0.5 * 0.5 = 0.25
        assertEquals(0.4, allLinear.transfer(0.8), 0.0001);  // 0.8 * 0.5 = 0.4
    }

    @Test
    void transfer_withZeroLimit_clampsToZero() {
        // limit=0 means all output is clamped to zero
        Sensitivity zeroLimit = new Sensitivity(0.1, 0.5, 0.3, 0.0);

        assertEquals(0.0, zeroLimit.transfer(0.5), 0.0001);
        assertEquals(0.0, zeroLimit.transfer(1.0), 0.0001);
    }

    @Test
    void transfer_exactlyAtBoundaries_handlesCorrectly() {
        // Test behavior exactly at boundary points
        // Exactly at threshold
        double atThreshold = sensitivity.transfer(0.1);
        assertEquals(0.03, atThreshold, 0.0001); // 0.1 * 0.3 = 0.03

        // Exactly at cusp
        double atCusp = sensitivity.transfer(0.5);
        assertEquals(0.15, atCusp, 0.0001); // 0.5 * 0.3 = 0.15
    }
}
