package frc.robot;

/**
 *
 */
class Sensitivity {
    double threshold;
    double linCoef;
    double cuspX;
    double limit;
    double a, b, c;

    public Sensitivity(double th, double cx, double lc, double li) {
        set(th, cx, lc, li);
    }

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

    public static double minLimit(double value, double minValue) {
      double abs = Math.abs(value);
      double sign = value > 0 ? 1 : -1;
      return abs > 0 && abs < minValue ? sign * minValue : value;
    }
}