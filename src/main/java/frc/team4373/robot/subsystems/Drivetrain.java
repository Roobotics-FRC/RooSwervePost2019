package frc.team4373.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.Utils;
import frc.team4373.robot.commands.teleop.SwerveDriveWithJoystick;
import frc.team4373.robot.input.WheelVector;
import frc.team4373.roologger.subsystems.LoggableDrivetrain;

/**
 * A programmatic representation of the robot's drivetrain.
 */
public class Drivetrain extends LoggableDrivetrain {
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

    @Override
    public void setLeft(double v) {
        setWheelsPID(new WheelVector.VectorSet(
                null,
                null,
                new WheelVector(1, 0),
                new WheelVector(1,0)
        ));
    }

    @Override
    public void setRight(double v) {
        setWheelsPID(new WheelVector.VectorSet(
                new WheelVector(1, 0),
                new WheelVector(1,0),
                null,
                null
        ));
    }

    @Override
    public double getLeftPercent() {
        return 0;
    }

    @Override
    public double getRightPercent() {
        return 0;
    }

    @Override
    public double getLeftVelocity() {
        return left1.encoderValues().speed; //FIXME: If `encoderValues` made sense, this WOULD NOT
        // work, but it doesn't make sense, so this does work.
    }

    @Override
    public double getRightVelocity() {
        return right1.encoderValues().speed;
    }

    @Override
    public double getLeftPosition() {
        return left1.getLinearPos();
    }

    @Override
    public double getRightPosition() {
        return right1.getLinearPos();
    }

    @Override
    public double getYaw() {
        return getPigeonYaw();
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
    public void setWheelsPID(WheelVector.VectorSet vectors) {
        if (vectors.right1 != null) this.right1.set(vectors.right1.speed, vectors.right1.angle);
        if (vectors.right2 != null) this.right2.set(vectors.right2.speed, vectors.right2.angle);
        if (vectors.left1 != null) this.left1.set(vectors.left1.speed, vectors.left1.angle);
        if (vectors.left2 != null) this.left2.set(vectors.left2.speed, vectors.left2.angle);
    }

    /**
     * Sets vectors to the SwerveWheels with a PID setpoint for angle and % output for speed.
     * @param vectors the four vectors ordered right1, left1, left2, right2.
     */
    public void setWheelsPercOut(WheelVector.VectorSet vectors) {
        if (vectors.right1 != null) {
            this.right1.setPercentOutput(vectors.right1.speed, vectors.right1.angle);
        }
        if (vectors.right2 != null) {
            this.right2.setPercentOutput(vectors.right2.speed, vectors.right2.angle);
        }
        if (vectors.left1 != null) {
            this.left1.setPercentOutput(vectors.left1.speed, vectors.left1.angle);
        }
        if (vectors.left2 != null) {
            this.left2.setPercentOutput(vectors.left2.speed, vectors.left2.angle);
        }
    }

    public double getWheelLinearPos(WheelID wheelID) {
        return getWheel(wheelID).getLinearPos();
    }

    /**
     * Resets the encoders to within [-4095, 4095]
     * Call this method in `robotInit` to (almost) eliminate the possibility of
     *  accumulator rollover during one power cycle.
     */
    public void modularizeEncoders() {
        this.right1.modularizeAbsoluteRotation();
        this.right2.modularizeAbsoluteRotation();
        this.left1.modularizeAbsoluteRotation();
        this.left2.modularizeAbsoluteRotation();
    }

    /**
     * This function should <b>NEVER</b> <i>regularly</i> be called.
     * It should be called once per mechanical change, with all wheels facing forward.
     */
    public void resetEncoder(WheelID wheelID) {
        getWheel(wheelID).resetAbsoluteRotation();
    }

    /**
     * Sets PID gains for the specified {@link SwerveWheel}.
     * @param wheelID the ID whose gains to modify.
     * @param drivePID the PID gains for the speed loop.
     * @param rotatorPID the PID gains for the rotational loop.
     */
    public void setPID(WheelID wheelID, RobotMap.PID drivePID, RobotMap.PID rotatorPID) {
        getWheel(wheelID).setPID(drivePID, rotatorPID);
    }

    /**
     * Returns the {@link SwerveWheel} at the specified position.
     * @param wheelID the ID of the wheel to fetch.
     * @return the indicated wheel object.
     */
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
     * Resets the pigeon's yaw to consider the current orientation field-forward.
     */
    public void resetPigeonYaw() {
        this.pigeon.setYaw(90);
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
        setDefaultCommand(new SwerveDriveWithJoystick());
    }
}
