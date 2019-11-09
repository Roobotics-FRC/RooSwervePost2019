package frc.team4373.robot.input;

public class WheelVector {
    public final double speed;
    public final double angle;

    public WheelVector(double speed, double angle) {
        this.speed = speed;
        this.angle = angle;
    }

    public static class VectorSet {
        public WheelVector right1;
        public WheelVector right2;
        public WheelVector left1;
        public WheelVector left2;

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
        return this.speed == vector.speed && this.angle == vector.angle;
    }
}
