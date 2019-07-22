package frc.team4373.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.Utils;
import frc.team4373.robot.commands.teleop.SwerveDriveWithJoystick;

/**
 * A programmatic representation of the robot's drivetrain.
 */
public class Drivetrain extends Subsystem {
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

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new SwerveDriveWithJoystick());
    }
}
