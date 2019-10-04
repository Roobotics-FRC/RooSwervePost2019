package frc.team4373.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.Utils;
import frc.team4373.robot.commands.teleop.DriveWithJoystick;
import frc.team4373.robot.commands.teleop.SwerveDriveWithJoystick;

/**
 * A programmatic representation of the robot's drivetrain.
 */
public class Drivetrain extends Subsystem {
    private static final double RADIUS = Math.sqrt(Math.pow(RobotMap.ROBOT_WHEELBASE, 2)
            + Math.pow(RobotMap.ROBOT_TRACKWIDTH, 2));
    private static final double LR = RobotMap.ROBOT_WHEELBASE / RADIUS;
    private static final double WR = (RobotMap.ROBOT_TRACKWIDTH / RADIUS);
    private static volatile Drivetrain instance;

    /**
     * The getter for the Drivetrain class.
     * @return the singleton Drivetrain object.
     */
    public static Drivetrain getInstance() {
        if (instance == null) {
            synchronized (Drivetrain.class) {
                if (instance == null) {
                    instance = new Drivetrain();
                }
            }
        }
        return instance;
    }

    public enum WheelID {
        RIGHT_1, RIGHT_2, LEFT_1, LEFT_2
    }

    private SwerveWheel right1;
    private SwerveWheel right2;
    private SwerveWheel left1;
    private SwerveWheel left2;
    private PigeonIMU pigeon;
    private double initialAngle;

    private Drivetrain() {
        this.right1 = new SwerveWheel(WheelID.RIGHT_1);
        this.right2 = new SwerveWheel(WheelID.RIGHT_2);
        this.left1 = new SwerveWheel(WheelID.LEFT_1);
        this.left2 = new SwerveWheel(WheelID.LEFT_2);

        this.pigeon = new PigeonIMU(RobotMap.PIGEON_PORT);
        this.initialAngle = getPigeonYaw();
    }

    /**
     * Returns the current angle relative to the starting position.
     */
    public double getAngle() {
        return Utils.normalizeAngle(getPigeonYaw() - initialAngle);
    }

    /**
     * Returns the Pigeon yaw value.
     * @return Pigeon yaw value.
     */
    private double getPigeonYaw() {
        double[] ypr = new double[3];
        this.pigeon.getYawPitchRoll(ypr);
        return ypr[0] * -1;
    }

    /**
     * Sets outputs of all motors of all wheels to zero.
     */
    public void zeroMotors() {
        this.right1.stop();
        this.right2.stop();
        this.left1.stop();
        this.left2.stop();
    }

    /**
     * Drives, in swerve mode, using the given inputs.
     *
     * <p>See https://www.chiefdelphi.com/t/107383
     * @param rotation The rotation of the joystick (CW is positive)
     * @param x The x coordinate of the joystick (right is positive)
     * @param y THe y coordinate of teh joystick (forward is positive)
     */
    public void drive(double rotation, double x, double y) {
        double angle = Math.toRadians(getAngle());

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

        right1.set(angles[0], speeds[0]);
        left1.set(angles[1], speeds[1]);
        left2.set(angles[2], speeds[2]);
        right2.set(angles[3], speeds[3]);
    }

    /**
     * Resets the encoders to within [-4095, 4095]
     * Call this method in `robotInit` to (almost) eliminate the possibility of
     *  accumulator rollover during one power cycle.
     */
    public void modularizeEncoders() {
        this.right1.modularizeAbsoluteEncoder();
        this.right2.modularizeAbsoluteEncoder();
        this.left1.modularizeAbsoluteEncoder();
        this.left2.modularizeAbsoluteEncoder();
    }

    /**
     * This function should <b>NEVER</b> <i>regularly</i> be called.
     * It should be called once per mechanical change, with all wheels facing forward.
     */
    private void resetEncoders() {
        this.right1.resetAbsoluteEncoder();
        this.right2.resetAbsoluteEncoder();
        this.left1.resetAbsoluteEncoder();
        this.left2.resetAbsoluteEncoder();
    }

    public SwerveWheel getSwerveWheel(WheelID wheel){
       switch (wheel){
           case RIGHT_1:
               return this.right1;
           case RIGHT_2:
               return this.right2;
           case LEFT_1:
               return this.left1;
           case LEFT_2:
               return this.left2;

       }
       return null;

    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveWithJoystick());
    }
}
