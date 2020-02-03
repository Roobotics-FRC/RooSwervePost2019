package frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.input.WheelVector;

/**
 * Represents a swerve wheel with two motors.
 */
public class SwerveWheel {
    private static final double HALF_REVOLUTION_TICKS = 180 * RobotMap.DEGREES_TO_ENCODER_UNITS;
    private static final double FULL_REVOLUTION_TICKS = 360 * RobotMap.DEGREES_TO_ENCODER_UNITS;

    private WPI_TalonSRX driveMotor;
    private WPI_TalonSRX rotatorMotor;

    private Drivetrain.WheelID wheelID;
    private boolean isInverted = false;

    /**
     * Constructs a new sweve wheel for the specified wheel.
     * @param wheelID the wheel to construct.
     */
    public SwerveWheel(Drivetrain.WheelID wheelID) {
        RobotMap.MotorConfig driveMotorConfig = RobotMap.getDriveMotorConfig(wheelID);
        RobotMap.MotorConfig rotatorMotorConfig = RobotMap.getRotatorMotorConfig(wheelID);
        this.wheelID = wheelID;

        this.driveMotor = new WPI_TalonSRX(driveMotorConfig.port);
        this.rotatorMotor = new WPI_TalonSRX(rotatorMotorConfig.port);

        this.driveMotor.configPeakCurrentLimit(RobotMap.TALON_MAX_AMPS,
                RobotMap.TALON_TIMEOUT_MS);
        this.driveMotor.enableCurrentLimit(true);

        this.driveMotor.setInverted(driveMotorConfig.inverted);
        this.rotatorMotor.setInverted(rotatorMotorConfig.inverted);

        this.driveMotor.setNeutralMode(driveMotorConfig.neutralMode);
        this.rotatorMotor.setNeutralMode(rotatorMotorConfig.neutralMode);

        this.driveMotor.setSensorPhase(driveMotorConfig.encoderPhase);
        this.rotatorMotor.setSensorPhase(rotatorMotorConfig.encoderPhase);

        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        this.rotatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        this.rotatorMotor.configFeedbackNotContinuous(false, RobotMap.TALON_TIMEOUT_MS);

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

        heading *= RobotMap.DEGREES_TO_ENCODER_UNITS;

        double currentRotation = rotatorMotor.getSelectedSensorPosition();
        double rotationError = Math.IEEEremainder(heading - currentRotation,
                RobotMap.ROTATOR_ENCODER_TICKS_PER_REV);

        SmartDashboard.putNumber("swerve/" + this.wheelID.name() + "/Pre-Inv Rot", rotationError);

        // minimize azimuth rotation, reversing drive if necessary
        isInverted = Math.abs(rotationError) > 0.25 * RobotMap.ROTATOR_ENCODER_TICKS_PER_REV;
        if (isInverted) {
            rotationError -= Math.copySign(0.5 * RobotMap.ROTATOR_ENCODER_TICKS_PER_REV,
                    rotationError);
            speed = -speed;
        }

        SmartDashboard.putNumber("swerve/" + this.wheelID.name() + "/Rot Offset", rotationError);
        SmartDashboard.putNumber("swerve/" + this.wheelID.name() + "/Rot Setpt",
                currentRotation + rotationError);
        SmartDashboard.putNumber("swerve/" + this.wheelID.name() + "/Speed",
                speed * RobotMap.MAX_WHEEL_SPEED);
        SmartDashboard.putNumber("swerve/" + this.wheelID.name() + "/Pos",
                this.getDriveMotorPosition());

        this.rotatorMotor.set(ControlMode.Position, currentRotation + rotationError);
        if (speed == 0) {
            this.driveMotor.set(ControlMode.PercentOutput, 0);
        } else {
            this.driveMotor.set(ControlMode.Velocity, speed * RobotMap.MAX_WHEEL_SPEED);
        }
    }

    /**
     * Sets drive and rotator PID gains.
     * @param drivePID a {@link RobotMap.PID} object containing new parameters for the drive PID,
     *                 or null to leave unchanged.
     * @param rotatorPID a {@link RobotMap.PID} object containing parameters for rotational PID,
     *                   or null to leave unchanged.
     */
    public void setPID(RobotMap.PID drivePID, RobotMap.PID rotatorPID) {
        if (drivePID != null) {
            setDrivePID(drivePID);
        }
        if (rotatorPID != null) {
            setRotatorPID(rotatorPID);
        }
    }

    private void setRotatorPID(RobotMap.PID pid) {
        this.rotatorMotor.config_kP(0, pid.kP);
        this.rotatorMotor.config_kI(0, pid.kI);
        this.rotatorMotor.config_kD(0, pid.kD);
        this.rotatorMotor.config_kF(0, pid.kF);
    }

    private void setDrivePID(RobotMap.PID pid) {
        this.driveMotor.config_kP(0, pid.kP);
        this.driveMotor.config_kI(0, pid.kI);
        this.driveMotor.config_kD(0, pid.kD);
        this.driveMotor.config_kF(0, pid.kF);
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

    public WheelVector encoderValues() {
        return new WheelVector(driveMotor.getSelectedSensorVelocity(),
                rotatorMotor.getSelectedSensorPosition());
    }

    /**
     * Gets the percent output of the drive motor.
     * @return the percent output, [-1, 1].
     */
    public double drivePercentOutput() {
        return this.driveMotor.getMotorOutputPercent();
    }

    /**
     * Gets the current position of the drive motor.
     * @return the current position in encoder units.
     */
    public double getDriveMotorPosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public void modularizeAbsoluteEncoder() {
        this.rotatorMotor.setSelectedSensorPosition(
                (int) (this.rotatorMotor.getSelectedSensorPosition() % FULL_REVOLUTION_TICKS));
    }

    public void resetAbsoluteEncoder() {
        this.rotatorMotor.setSelectedSensorPosition(0);
    }
}