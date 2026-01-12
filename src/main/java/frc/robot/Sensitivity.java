package frc.robot;

/**
 * Hybrid linear-quadratic sensitivity curve for joystick input processing.
 *
 * <p>Applies a piecewise transfer function that combines linear and quadratic regions:
 * <ul>
 *   <li>A deadzone where inputs below {@code threshold} return zero</li>
 *   <li>A linear region from {@code threshold} to {@code cuspX} using {@code linCoef}</li>
 *   <li>A quadratic region above {@code cuspX} for smooth acceleration to full output</li>
 *   <li>Output clamping at {@code limit} to prevent exceeding maximum values</li>
 * </ul>
 *
 * <p>The quadratic region provides a smooth curve that connects the linear region
 * to full output, allowing fine control at low speeds while still reaching maximum
 * output naturally.
 *
 * <p>Negative inputs produce negative outputs with the same magnitude transformation.
 *
 * @see PiecewiseSensitivity
 */
class Sensitivity {
    double threshold;
    double linCoef;
    double cuspX;
    double limit;
    double a, b, c;

    /**
     * Creates a new sensitivity curve with the specified parameters.
     *
     * @param th threshold value below which input returns zero (deadzone), typically 0.05-0.15
     * @param cx cusp X value where the curve transitions from linear to quadratic, typically 0.3-0.7
     * @param lc linear coefficient applied in the first region, typically 0.2-0.5
     * @param li limit value that clamps maximum output, typically 0.8-1.0
     */
    public Sensitivity(double th, double cx, double lc, double li) {
        set(th, cx, lc, li);
    }

    /**
     * Updates the sensitivity curve parameters and recalculates internal coefficients.
     *
     * <p>Can be used to dynamically adjust sensitivity during runtime, for example
     * via SmartDashboard tuning.
     *
     * <p>The quadratic coefficients (a, b, c) are computed to ensure a smooth
     * transition from the linear region at {@code cuspX} to full output at input = 1.0.
     *
     * @param th threshold value below which input returns zero (deadzone)
     * @param cx cusp X value where the curve transitions from linear to quadratic
     * @param lc linear coefficient applied in the first region
     * @param li limit value that clamps maximum output
     */
    public void set(double th, double cx, double lc, double li) {
        threshold = th;
        cuspX = cx;
        linCoef = lc;
        limit = li;

        double x0 = cuspX;
        double g = (2.0 - x0) * x0;
        double denom = 1.0 - g;
        a = (1.0 - linCoef) / denom;
        c = a * x0 * x0;
        b = (linCoef + (linCoef * x0 - 2.0) * x0) / denom;
    }

    /**
     * Applies the sensitivity curve to transform a joystick input value.
     *
     * <p>The transformation follows these rules:
     * <ul>
     *   <li>Inputs with absolute value below {@code threshold} return 0 (deadzone)</li>
     *   <li>Inputs between {@code threshold} and {@code cuspX} are scaled by {@code linCoef}</li>
     *   <li>Inputs above {@code cuspX} follow the quadratic curve (a*x² + b*x + c)</li>
     *   <li>Output is clamped to {@code limit}</li>
     * </ul>
     *
     * <p>If {@code threshold} is negative, the input is shifted by its absolute value
     * before applying the deadzone check.
     *
     * <p>The sign of the input is preserved in the output.
     *
     * @param x the input value, typically from a joystick axis in range [-1.0, 1.0]
     * @return the transformed output value, with sign preserved
     */
    double transfer(double x) {
        double xabs = Math.abs(x);
        if (threshold < 0) {
            xabs -= threshold;
            if (xabs <= 0)
              return 0;
        }
        else if (xabs < threshold)
          return 0;

        if (xabs <= cuspX) {
            xabs *= linCoef;
        }
        else {
            xabs = a * xabs * xabs + b * xabs + c;
        }
        if (xabs > limit)
            xabs = limit;
        return x >= 0 ? xabs : -xabs;
    }

    /**
     * Ensures a non-zero value meets a minimum magnitude threshold.
     *
     * <p>If the absolute value of the input is non-zero but below the minimum,
     * it is boosted to the minimum value while preserving the sign. Zero values
     * pass through unchanged.
     *
     * <p>This is useful for motor outputs where very small values may not overcome
     * friction but zero should remain zero (no movement commanded).
     *
     * @param value the input value to check
     * @param minValue the minimum absolute value threshold
     * @return the original value if zero or above minimum, otherwise minValue with original sign
     */
    public static double minLimit(double value, double minValue) {
      double abs = Math.abs(value);
      double sign = value > 0 ? 1 : -1;
      return abs > 0 && abs < minValue ? sign * minValue : value;
    }
}