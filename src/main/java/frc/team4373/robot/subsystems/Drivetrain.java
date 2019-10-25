package frc.team4373.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.Utils;
import frc.team4373.robot.commands.teleop.DriveWithJoystick;
import frc.team4373.robot.input.WheelVector;

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

    /**
     * Sets velocity vectors to the four SwerveWheels with PID setpoints for both speed and angle.
     * @param vectors the four vectors ordered right1, left1, left2, right2.
     */
    public void setWheelsPID(WheelVector[] vectors) {
        if (vectors.length == 4) {
            if (vectors[0] != null) this.right1.set(vectors[0].speed, vectors[0].angle);
            if (vectors[1] != null) this.left1.set(vectors[1].speed, vectors[1].angle);
            if (vectors[2] != null) this.left2.set(vectors[2].speed, vectors[2].angle);
            if (vectors[3] != null) this.right2.set(vectors[3].speed, vectors[3].angle);
        } else {
            DriverStation.reportError("Invalid array passed to setWheelsPID", false);
        }
    }

    /**
     * Sets vectors to the SwerveWheels with a PID setpoint for angle and % output for speed.
     * @param vectors the four vectors ordered right1, left1, left2, right2.
     */
    public void setWheelsPercOut(WheelVector[] vectors) {
        if (vectors.length == 4) {
            if (vectors[0] != null) {
                this.right1.setPercentOutput(vectors[0].speed, vectors[0].angle);
            }
            if (vectors[1] != null) {
                this.left1.setPercentOutput(vectors[1].speed, vectors[1].angle);
            }
            if (vectors[2] != null) {
                this.left2.setPercentOutput(vectors[2].speed, vectors[2].angle);
            }
            if (vectors[3] != null) {
                this.right2.setPercentOutput(vectors[3].speed, vectors[3].angle);
            }
        } else {
            DriverStation.reportError("Invalid array passed to setWheelsPercOut", false);
        }
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
    public void resetEncoder(WheelID wheelID) {
        getWheel(wheelID).resetAbsoluteEncoder();
    }

    public void setPID(WheelID wheelID, RobotMap.PID drivePID, RobotMap.PID rotatorPID) {
        getWheel(wheelID).setPID(drivePID, rotatorPID);
    }

    private SwerveWheel getWheel(WheelID wheelID) {
        switch (wheelID) {
            case RIGHT_1:
                return this.right1;
            case RIGHT_2:
                return this.right2;
            case LEFT_1:
                return this.left1;
            case LEFT_2:
                return this.left2;
            default:
                return getWheel(WheelID.RIGHT_1);
        }
    }

    /**
     * Logs encoder values to SmartDashboard.
     */
    public void logEncoders() {
        SmartDashboard.putString("R1", this.right1.encoderValues().toString());
        SmartDashboard.putString("R2", this.right2.encoderValues().toString());
        SmartDashboard.putString("L1", this.left1.encoderValues().toString());
        SmartDashboard.putString("L2", this.left2.encoderValues().toString());
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveWithJoystick());
    }
}
