package frc.team4373.robot.input;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.Utils;

public class SwerveInputTransform {
    private static final double RADIUS = Math.sqrt(Math.pow(RobotMap.ROBOT_WHEELBASE, 2)
            + Math.pow(RobotMap.ROBOT_TRACKWIDTH, 2));
    private static final double LR = RobotMap.ROBOT_WHEELBASE / RADIUS;
    private static final double WR = (RobotMap.ROBOT_TRACKWIDTH / RADIUS);

    /**
     * Produces swerve velocity vectors using the given inputs.
     *
     * <p>See https://www.chiefdelphi.com/t/107383
     * @param rotation The rotation of the joystick (CW is positive)
     * @param x The x coordinate of the joystick (right is positive)
     * @param y The y coordinate of the joystick (forward is positive)
     * @param imuAngle The angle of the gyro (IMU)
     */
    public static WheelVector.VectorSet process(double rotation, double x, double y,
                                                double imuAngle) {
        double angle = Math.toRadians(imuAngle);

        final double temp = y * Math.cos(angle) + x * Math.sin(angle);
        x = -y * Math.sin(angle) + x * Math.cos(angle);
        y = temp;

        final double A = x - rotation * LR;
        final double B = x + rotation * LR;
        final double C = y - rotation * WR;
        final double D = y + rotation * WR;

        double[] speeds = new double[4];
        double[] angles = new double[4];

        speeds[0] = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2)); //front right
        speeds[1] = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2)); // front left
        speeds[2] = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2)); // rear left
        speeds[3] = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2)); // rear right

        angles[0] = Math.toDegrees(Math.atan2(B,C));
        angles[1] = Math.toDegrees(Math.atan2(B,D));
        angles[2] = Math.toDegrees(Math.atan2(A,D));
        angles[3] = Math.toDegrees(Math.atan2(A,C));

        final double highestWheelSpeed = Math.max(Math.max(speeds[0], speeds[1]),
                Math.max(speeds[2], speeds[3]));
        if (highestWheelSpeed > 1) {
            for (int i = 0; i < 4; ++i) {
                speeds[i] /= highestWheelSpeed;
            }
        }

        WheelVector right1 = new WheelVector(speeds[0], angles[0]);
        WheelVector left1 = new WheelVector(speeds[1], angles[1]);
        WheelVector left2 = new WheelVector(speeds[2], angles[2]);
        WheelVector right2 = new WheelVector(speeds[3], angles[3]);

        return new WheelVector.VectorSet(right1, right2, left1, left2);
    }

    /**
     * Processes x-y input from the joystick to produce translational-only movement vectors.
     * @param x the x-coordinate of the joystick's position (right-positive).
     * @param y the y-coordinate of the joystick's position (forward-positive).
     * @return a {@link WheelVector.VectorSet} of translational movement vectors.
     */
    public static WheelVector.VectorSet processTranslation(double x, double y) {
        double angle = Utils.calculateYOffset(x, y);
        double magnitude = Math.sqrt(x * x + y * y);
        WheelVector vec = new WheelVector(magnitude, angle);
        return new WheelVector.VectorSet(vec, vec, vec, vec);
    }
}
