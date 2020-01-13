package frc.team4373.robot.input;

import frc.team4373.robot.RobotMap;

public class WheelVector {
    public static final WheelVector ZERO = new WheelVector(0, 0);

    public final double speed;
    public final double angle;

    /**
     * Constructs a new WheelVector.
     * @param speed the speed (i.e., the magnitude) of the vector in the range [-1, 1].
     * @param angle the angle (i.e., the direction) of the vector in degrees.
     */
    public WheelVector(double speed, double angle) {
        this.speed = speed;
        this.angle = angle;
    }

    public static class VectorSet {
        public static final VectorSet ZERO = new VectorSet(
                WheelVector.ZERO,
                WheelVector.ZERO,
                WheelVector.ZERO,
                WheelVector.ZERO);

        public WheelVector right1;
        public WheelVector right2;
        public WheelVector left1;
        public WheelVector left2;

        /**
         * Constructs a new WheelVector VectorSet.
         * @param right1 the vector for the front right wheel.
         * @param right2 the vector for the rear right wheel.
         * @param left1 the vector for the front left wheel.
         * @param left2 the vector for the rear left wheel.
         */
        public VectorSet(WheelVector right1, WheelVector right2,
                         WheelVector left1, WheelVector left2) {
            this.right1 = right1;
            this.right2 = right2;
            this.left1 = left1;
            this.left2 = left2;
        }

        public String toString() {
            return "R1: " + right1.toString() + ", R2: " + right2.toString()
                    + ", L1: " + left1.toString() + ", L2: " + left2.toString();
        }

        /**
         * Returns whether the given vector set is equal to another.
         * @param set the vector set against which to compare.
         * @return true if equal, false if not.
         */
        public boolean equals(WheelVector.VectorSet set) {
            return this.right1.equals(set.right1)
                    && this.right2.equals(set.right2)
                    && this.left1.equals(set.left1)
                    && this.left2.equals(set.left2);
        }
    }

    @Override
    public String toString() {
        return "(speed: " + this.speed + ", angle: " + this.angle + ")";
    }

    public boolean equals(WheelVector vector) {
        return this.speed - vector.speed < RobotMap.FP_EQUALITY_THRESHOLD
                && this.angle - vector.angle < RobotMap.FP_EQUALITY_THRESHOLD;
    }
}
