package frc.team4373.robot;

public final class Utils {
    private Utils() {}

    /**
     * Takes an angle in degrees and reduces it to its smallest positive equivalent from 0 to 360.
     * @param angle the angle to normalize, in degrees.
     * @return the angle represented as a positive angle on the range [0, 360].
     */
    public static double normalizeAngle(double angle) {
        return leastResidue(angle, 360);
    }

    /**
     * Calculates a normalized angle from the y-axis to a specified point.
     * @param x the x-coordinate of the point.
     * @param y the y-coordinate of the point.
     * @return the normalized angle from the y-axis to the point, in degrees.
     */
    public static double calculateYOffset(double x, double y) {
        return Utils.normalizeAngle(90 - Math.toDegrees(Math.atan2(y, x)));

        // // Compute the angle relative to the y-axis (90°)
        // double rawAngle = 90 - Math.toDegrees(Math.atan2(rooGetY(), rooGetX()));
        // // Normalize the angle so that it is positive
        // return Utils.normalizeAngle(rawAngle);
    }

    /**
     * Computes the least residue (i.e., > 0) of a number n modulo the given modulus.
     *
     * @throws IllegalArgumentException if modulus <= 0.
     *
     * @param n the number whose least residue to compute.
     * @param modulus the modulus in which to compute the least residue.
     * @return the least residue.
     */
    public static double leastResidue(double n, double modulus) {
        if (modulus <= 0) {
            throw new IllegalArgumentException("Modulus cannot be less than or equal to zero.");
        }
        return ((n % modulus) + modulus) % modulus;
    }
}
