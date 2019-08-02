package frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.team4373.robot.RobotMap;

/**
 * Represents a swerve wheel with two motors.
 */
public class SwerveWheel {
    private WPI_TalonSRX driveMotor;
    private WPI_TalonSRX rotatorMotor;
    private static final double HALF_REVOLUTION_TICKS = 180 * RobotMap.DEGREES_TO_ENCODER_TICKS;
    private static final double FULL_REVOLUTION_TICKS = 360 * RobotMap.DEGREES_TO_ENCODER_TICKS;

    /**
     * Constructs a new sweve wheel for the specified wheel.
     * @param wheelID the wheel to construct.
     */
    public SwerveWheel(Drivetrain.WheelID wheelID) {
        RobotMap.MotorConfig driveMotorConfig = RobotMap.getDriveMotorConfig(wheelID);
        RobotMap.MotorConfig rotatorMotorConfig = RobotMap.getRotatorMotorConfig(wheelID);

        this.driveMotor = new WPI_TalonSRX(driveMotorConfig.port);
        this.rotatorMotor = new WPI_TalonSRX(rotatorMotorConfig.port);

        this.driveMotor.setInverted(driveMotorConfig.inverted);
        this.rotatorMotor.setInverted(rotatorMotorConfig.inverted);

        this.driveMotor.setNeutralMode(driveMotorConfig.neutralMode);
        this.rotatorMotor.setNeutralMode(rotatorMotorConfig.neutralMode);

        this.driveMotor.setSensorPhase(driveMotorConfig.encoderPhase);
        this.rotatorMotor.setSensorPhase(rotatorMotorConfig.encoderPhase);

        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        this.rotatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    }

    /**
     * Sets the swerve wheel's two motors' PID loops.
     * @param heading the heading, in degrees, at which to angle the wheel.
     * @param speed the percent of maximum speed at which to drive.
     */
    public void set(double heading, double speed) {
        setSpeed(speed);
        setHeading(heading);
    }

    /**
     * Sets the speed to the given value.
     * @param speed The percent of maximum speed at which to drive.
     */
    private void setSpeed(double speed) {
        this.driveMotor.set(ControlMode.Velocity, speed * RobotMap.MAX_WHEEL_SPEED);
    }

    /**
     * Sets the heading to the given value.
     * @param heading The heading, in degrees, at which to angle the wheel.
     */
    private void setHeading(double heading) {
        double current = this.rotatorMotor.getSelectedSensorPosition() * RobotMap.DEGREES_TO_ENCODER_TICKS;
        double target = heading * RobotMap.DEGREES_TO_ENCODER_TICKS;
        double error = target - current;
        if (Math.abs(error) > HALF_REVOLUTION_TICKS) {
            error = -(Math.signum(error) * (FULL_REVOLUTION_TICKS - Math.abs(error)));
        }
        this.rotatorMotor.set(ControlMode.Position, current + error);
    }

    /**
     * Stops all motors.
     */
    public void stop() {
        this.driveMotor.set(ControlMode.PercentOutput, 0);
        this.rotatorMotor.set(ControlMode.PercentOutput, 0);
    }
}