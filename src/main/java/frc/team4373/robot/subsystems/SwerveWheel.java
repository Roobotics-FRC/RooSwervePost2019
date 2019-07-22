package frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.team4373.robot.RobotMap;

/**
 * A Javadoc template. TODO: Update SwerveWheel Javadoc.
 */
public class SwerveWheel {
    private WPI_TalonSRX driveMotor;
    private WPI_TalonSRX rotatorMotor;

    public SwerveWheel(Drivetrain.MotorID motorID) {
        RobotMap.MotorConfig driveMotorConfig = RobotMap.getDriveMotorConfig(motorID);
        RobotMap.MotorConfig rotatorMotorConfig = RobotMap.getRotatorMotorConfig(motorID);

        this.driveMotor = new WPI_TalonSRX(driveMotorConfig.port);
        this.rotatorMotor = new WPI_TalonSRX(rotatorMotorConfig.port);

        this.driveMotor.setInverted(driveMotorConfig.inverted);
        this.rotatorMotor.setInverted(rotatorMotorConfig.inverted);

        this.driveMotor.setNeutralMode(driveMotorConfig.neutralMode);
        this.rotatorMotor.setNeutralMode(rotatorMotorConfig.neutralMode);

        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        this.rotatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    }

    /**
     * Sets the swerve wheel's two motors' PID loops.
     * @param heading the heading, in degrees, at which to angle the wheel.
     * @param velocity the percent of maximum velocity at which to drive.
     */
    public void set(double heading, double velocity) {
        this.driveMotor.set(ControlMode.Velocity, velocity * RobotMap.MAX_WHEEL_VELOCITY);
        // TODO: set rotator motor
    }

    public double getAngle() {
        return 0;//FIXME: ...obviously
    }
}