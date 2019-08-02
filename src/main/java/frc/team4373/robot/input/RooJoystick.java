package frc.team4373.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import frc.team4373.robot.input.filters.DoubleTypeFilter;

/**
 * This class extends the WPILib Joystick class to add deadzone and filter functionality.
 */
public class RooJoystick<F extends DoubleTypeFilter> extends Joystick {
    /**
     * A representation of an axis of the joystick.
     */
    public enum Axis {
        X, Y, Z, TWIST, THROTTLE;

        /**
         * Creates an Axis from an AxisType.
         * @param axis the AxisType to copy.
         * @return a new Axis with the same 'mental image' as the AxisType.
         */
        public static Axis from(AxisType axis) {
            switch (axis) {
                case kX:
                    return X;
                case kY:
                    return Y;
                case kZ:
                    return Z;
                case kTwist:
                    return TWIST;
                case kThrottle:
                    return THROTTLE;
                default:
                    return null;
            }
        }
    }

    private static final double DEADZONE = 0.09;
    private F filter = null;

    public RooJoystick(int port, F filter) {
        super(port);
        this.filter = filter;
    }

    /**
     * Filter a value using the joystick's filter.
     * @param val the value to filter.
     * @return the filtered value.
     */
    private double filter(double val) {
        // We don't know the return type because of type erasure...
        return applyDeadzone(this.filter.applyFilter(val));
    }

    /**
     * Ignores input if it is within the deadzone (if it is negligible).
     * @param input the input value to be checked.
     * @return the input value if it is large enough, or 0 if it was negligible.
     */
    private double applyDeadzone(double input) {
        return Math.abs(input) <= DEADZONE ? 0 : input;
    }

    public double rooGetX() {
        return this.filter(this.getX());
    }

    public double rooGetY() {
        return this.filter(this.getY());
    }

    public double rooGetZ() {
        return this.filter(this.getZ());
    }

    public double rooGetTwist() {
        return this.filter(this.getTwist());
    }

    public double rooGetThrottle() {
        return this.filter(this.getThrottle());
    }

    /**
     * Returns z axis with a custom filter.
     * @return the filtered z-axis.
     */
    public double rooGetZFiltered() {
        double singleFilterZ = this.rooGetZ();
        double filtered = Math.signum(singleFilterZ) * Math.sqrt(Math.abs(singleFilterZ)) / 3;
        if (Math.abs(filtered) < DEADZONE) filtered = 0;
        return filtered;
    }

    /**
     * Returns z-axis with a less sensitive filter, especially at low power
     * @return The filtered z-axis.
     */
    public double newRooGetZFiltered() {
        return Math.signum(this.getZ()) * this.getZ() * this.rooGetZFiltered();
    }

    /**
     * Returns the filtered value of a joystick axis.
     *
     * @param axis the axis from which to read.
     * @return the filtered value of the axis.
     */
    public double getAxis(Axis axis) {
        switch (axis) {
            case X:
                return this.rooGetX();
            case Y:
                return this.rooGetY();
            case Z:
                return this.rooGetZ();
            case TWIST:
                return this.rooGetTwist();
            case THROTTLE:
                return this.rooGetThrottle();
            default:
                return 0d;
        }
    }

    /**
     * Returns the filtered value of a joystick axis.
     * If joystick has single, well-defined axes, {@link #getAxis(Axis axis)} is preferable.
     *
     * @param axis the axis to read from.
     * @return the filtered value of the axis.
     */
    public double getAxis(int axis) {
        return this.filter(this.getRawAxis(axis));
    }

    /**
     * Calculates a normalized (0–360) angle from the y-axis to the joystick's filtered position.
     * This is analogous to a polar theta-value.
     * @return the normalized angle from the y-axis to the joystick location, in degrees.
     */
    public double getAngle() {
        // Compute the angle relative to the y-axis (90°)
        double rawAngle = 90 - Math.toDegrees(Math.atan2(rooGetY(), rooGetX()));
        // Normalize the angle so that it is positive
        return ((rawAngle % 360) + 360) % 360;
    }

    /**
     * Returns the absolute (positive) distance from the origin by which the joystick has been
     * displaced. This is analogous to a polar r-value.
     * @return the distance the joystick has been moved from the origin.
     */
    public double getDistance() {
        return Math.sqrt(Math.pow(rooGetX(), 2) + Math.pow(rooGetY(), 2));
    }
}
