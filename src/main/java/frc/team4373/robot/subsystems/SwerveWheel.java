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
     * @param heading THe heading, in degrees, at which to angle the wheel.
     */
    private void setHeading(double heading) {
        this.rotatorMotor.set(ControlMode.Position, heading); //This needs to be converted to encoder ticks.
        //FIXME: This isn't a real implementation...
    }

    public double getAngle() {
        return 0;//FIXME: ...obviously
    }
}