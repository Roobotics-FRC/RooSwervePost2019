package frc.team4373.robot.input.filters;

/**
 * Piecewise linear function for more granular joystick control.
 * -------------------------------------------------------------
 * Note that this piecewise function is pre-defined for a single case.
 * As such, it should MOST LIKELY NOT be used in anything new.
 */
public class FineGrainedPiecewiseFilter extends DoubleTypeFilter {

    @Override
    public Double applyFilter(Double input) {
        return (Math.abs(input) <= 0.8) ? 0.5 * input : ((3 * input) - (Math.signum(input) * 2));
    }
}
