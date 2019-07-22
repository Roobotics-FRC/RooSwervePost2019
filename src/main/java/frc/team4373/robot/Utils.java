package frc.team4373.robot;

public final class Utils {
    private Utils() {}

    /**
     * Takes an angle in degrees and reduces it to its smallest positive equivalent from 0 to 360.
     * @param angle the angle to normalize, in degrees.
     * @return the angle represented as a positive angle on the range [0, 360].
     */
    public static double normalizeAngle(double angle) {
        return ((angle % 360) + 360) % 360;
    }

    /**
     * Calculates a normalized angle from the y-axis to a specified point.
     * @param x the x-coordinate of the point.
     * @param y the y-coordinate of the point.
     * @return the normalized angle from the y-axis to the point, in degrees.
     */
    public static double calculateYOffset(double x, double y) {
        return Utils.normalizeAngle(90 - Math.toDegrees(Math.atan2(y, x)));
    }
}
