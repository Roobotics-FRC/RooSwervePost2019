package frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.Utils;

/**
 * Represents a swerve wheel with two motors.
 */
public class SwerveWheel {
    private WPI_TalonSRX driveMotor;
    private WPI_TalonSRX rotatorMotor;
    private static final double HALF_REVOLUTION_TICKS = 180 * RobotMap.DEGREES_TO_ENCODER_UNITS;
    private static final double FULL_REVOLUTION_TICKS = 360 * RobotMap.DEGREES_TO_ENCODER_UNITS;

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
        this.rotatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        this.rotatorMotor.configFeedbackNotContinuous(false, 1000);

        this.driveMotor.config_kF(RobotMap.PID_IDX, driveMotorConfig.gains.kF);
        this.driveMotor.config_kP(RobotMap.PID_IDX, driveMotorConfig.gains.kP);
        this.driveMotor.config_kI(RobotMap.PID_IDX, driveMotorConfig.gains.kI);
        this.driveMotor.config_kD(RobotMap.PID_IDX, driveMotorConfig.gains.kD);

        this.rotatorMotor.config_kF(RobotMap.PID_IDX, rotatorMotorConfig.gains.kF);
        this.rotatorMotor.config_kP(RobotMap.PID_IDX, rotatorMotorConfig.gains.kP);
        this.rotatorMotor.config_kI(RobotMap.PID_IDX, rotatorMotorConfig.gains.kI);
        this.rotatorMotor.config_kD(RobotMap.PID_IDX, rotatorMotorConfig.gains.kD);
    }

    /**
     * Sets the swerve wheel's two motors' PID loops.
     * @param heading the heading, in degrees, at which to angle the wheel.
     * @param speed the percent of maximum speed at which to drive.
     */
    public void set(double speed, double heading) {
        /*
        // minimize azimuth rotation, reversing drive if necessary
        isInverted = Math.abs(azimuthError) > 0.25 * TICKS;
        if (isInverted) {
            azimuthError -= Math.copySign(0.5 * TICKS, azimuthError);
            drive = -drive;
        }
        *///TODO: This would mean that instead of turning 170°, we turn -10 and flip the speed.
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
        double rawCurrent = this.rotatorMotor.getSelectedSensorPosition();
        double current = Utils.leastResidue(rawCurrent, 4096);
        double target = heading * RobotMap.DEGREES_TO_ENCODER_UNITS;
        double error = target - current;
        if (Math.abs(error) > HALF_REVOLUTION_TICKS) {
            error = -(Math.signum(error) * (FULL_REVOLUTION_TICKS - Math.abs(error)));
        }
        this.rotatorMotor.set(ControlMode.Position, rawCurrent + error);
    }

    public void setPercentOutput(double speed, double heading) {
        this.driveMotor.set(ControlMode.PercentOutput, speed);
        this.rotatorMotor.set(ControlMode.PercentOutput, heading);
    }

    /**
     * Stops all motors.
     */
    public void stop() {
        this.driveMotor.set(ControlMode.PercentOutput, 0);
        this.rotatorMotor.set(ControlMode.PercentOutput, 0);
    }

    public void modularizeAbsoluteEncoder() {
        this.rotatorMotor.setSelectedSensorPosition(
                (int) (this.rotatorMotor.getSelectedSensorPosition() % FULL_REVOLUTION_TICKS));
    }

    public void resetAbsoluteEncoder() {
        this.rotatorMotor.setSelectedSensorPosition(0);
    }
}