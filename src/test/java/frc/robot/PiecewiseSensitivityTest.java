package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for PiecewiseSensitivity.
 * Verifies joystick input curve calculations.
 */
class PiecewiseSensitivityTest {

    private PiecewiseSensitivity sensitivity;

    // Standard test configuration:
    // xStart=0.1, xMiddle=0.5, yStart=0.1, yMiddle=0.5, yMax=1.0
    @BeforeEach
    void setUp() {
        sensitivity = new PiecewiseSensitivity(0.1, 0.5, 0.1, 0.5, 1.0);
    }

    @Test
    void transfer_belowStart_returnsZero() {
        // Input below deadzone threshold should return 0
        assertEquals(0.0, sensitivity.transfer(0.05), 0.0001);
        assertEquals(0.0, sensitivity.transfer(0.0), 0.0001);
        assertEquals(0.0, sensitivity.transfer(0.09), 0.0001);
    }

    @Test
    void transfer_atStart_returnsStartY() {
        // At exactly xStart, should return yStart
        assertEquals(0.1, sensitivity.transfer(0.1), 0.0001);
    }

    @Test
    void transfer_atMiddle_returnsMiddleY() {
        // At exactly xMiddle, should return yMiddle
        assertEquals(0.5, sensitivity.transfer(0.5), 0.0001);
    }

    @Test
    void transfer_atOne_returnsMax() {
        // At input of 1.0, should return yMax
        assertEquals(1.0, sensitivity.transfer(1.0), 0.0001);
    }

    @Test
    void transfer_aboveOne_returnsMax() {
        // Input above 1.0 should clamp to yMax
        assertEquals(1.0, sensitivity.transfer(1.5), 0.0001);
        assertEquals(1.0, sensitivity.transfer(2.0), 0.0001);
    }

    @Test
    void transfer_negativeInput_returnsNegativeOutput() {
        // Negative inputs should produce negative outputs with same magnitude
        double positiveResult = sensitivity.transfer(0.3);
        double negativeResult = sensitivity.transfer(-0.3);

        assertEquals(-positiveResult, negativeResult, 0.0001);
    }

    @Test
    void transfer_negativeBelowStart_returnsZero() {
        // Negative input below deadzone should also return 0
        assertEquals(0.0, sensitivity.transfer(-0.05), 0.0001);
    }

    @Test
    void transfer_inFirstSegment_interpolatesCorrectly() {
        // Between xStart and xMiddle, should linearly interpolate
        // At x=0.3 (midpoint between 0.1 and 0.5), should be at y=0.3 (midpoint between 0.1 and 0.5)
        assertEquals(0.3, sensitivity.transfer(0.3), 0.0001);
    }

    @Test
    void transfer_inSecondSegment_interpolatesCorrectly() {
        // Between xMiddle and 1.0, should linearly interpolate
        // At x=0.75 (midpoint between 0.5 and 1.0), should be at y=0.75 (midpoint between 0.5 and 1.0)
        assertEquals(0.75, sensitivity.transfer(0.75), 0.0001);
    }

    @Test
    void set_updatesAllParameters() {
        // After calling set, behavior should change
        sensitivity.set(0.2, 0.6, 0.2, 0.4, 0.8);

        // Below new threshold
        assertEquals(0.0, sensitivity.transfer(0.1), 0.0001);

        // At new max
        assertEquals(0.8, sensitivity.transfer(1.0), 0.0001);
    }
}
