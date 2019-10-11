package frc.team4373.robot.input;

public class WheelVector {
    public final double speed;
    public final double angle;

    public WheelVector(double speed, double angle) {
        this.speed = speed;
        this.angle = angle;
    }

    @Override
    public String toString() {
        return "(speed: " + this.speed + ", angle: " + this.angle + ")";
    }
}
