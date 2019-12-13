package frc.team4373.robot.input;

import frc.team4373.robot.RobotMap;
import frc.team4373.robot.Utils;

public class SwerveInputTransform {
    private static final double RADIUS = Math.sqrt(Math.pow(RobotMap.ROBOT_WHEELBASE, 2)
            + Math.pow(RobotMap.ROBOT_TRACKWIDTH, 2));
    private static final double LR = RobotMap.ROBOT_WHEELBASE / RADIUS;
    private static final double WR = (RobotMap.ROBOT_TRACKWIDTH / RADIUS);

    private static double[] speeds = new double[RobotMap.WHEEL_COUNT];
    private static double[] angles = new double[RobotMap.WHEEL_COUNT];

    /**
     * Produces swerve velocity vectors relative to the field for the given input.
     *
     * <p>See https://www.chiefdelphi.com/t/107383
     * @param rotation the rotation of the joystick (CW is positive)
     * @param x the x coordinate of the joystick (right is positive)
     * @param y the y coordinate of the joystick (forward is positive)
     * @param imuAngle the current heading of the robot
     * @return a {@link WheelVector.VectorSet} of velocity vectors.
     */
    public static WheelVector.VectorSet processNorthUp(double rotation, double x, double y,
                                                       double imuAngle) {
        double angle = Math.toRadians(imuAngle);

        final double temp = y * Math.cos(angle) + x * Math.sin(angle);
        // TODO: why are the signs flipped here?
        x = -y * Math.sin(angle) + x * Math.cos(angle);
        y = temp;

        return processOwnShipUp(rotation, x, y);
    }

    /**
     * Produces swerve velocity vectors relative to the robot for the given inputs.
     *
     * @param rotation the rotation of the joystick (CW is positive)
     * @param x the x coordinate of the joystick (right is positive)
     * @param y the y coordinate of the joystick (forward is positive)
     * @return a {@link WheelVector.VectorSet} of velocity vectors.
     */
    public static WheelVector.VectorSet processOwnShipUp(double rotation, double x, double y) {
        final double A = x - rotation * LR;
        final double B = x + rotation * LR;
        final double C = y - rotation * WR;
        final double D = y + rotation * WR;

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
        if (magnitude > 1) magnitude = 1;
        WheelVector vec = new WheelVector(magnitude, angle);
        return new WheelVector.VectorSet(vec, vec, vec, vec);
    }

    /**
     * Processes rotational input from the joystick to produce rotational-only movement vectors.
     * @param rate the rate of rotation, [-1, 1].
     * @return a {@link WheelVector.VectorSet} of rotational movement vectors.
     */
    public static WheelVector.VectorSet processRotation(double rate) {
        double refAngle = Math.toDegrees(Math.atan2(RobotMap.ROBOT_TRACKWIDTH / 2,
                RobotMap.ROBOT_WHEELBASE / 2));
        double r1Angle = 90 + refAngle;
        double l1Angle = 90 - refAngle;
        double l2Angle = -(90 - refAngle);
        double r2Angle = -(90 + refAngle);
        WheelVector r1 = new WheelVector(rate, r1Angle);
        WheelVector r2 = new WheelVector(rate, r2Angle);
        WheelVector l1 = new WheelVector(rate, l1Angle);
        WheelVector l2 = new WheelVector(rate, l2Angle);
        return new WheelVector.VectorSet(r1, r2, l1, l2);
    }
}
