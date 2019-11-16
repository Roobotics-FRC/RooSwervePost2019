package frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.Utils;
import frc.team4373.robot.input.WheelVector;

/**
 * Represents a swerve wheel with two motors.
 */
public class SwerveWheel {
    private WPI_TalonSRX driveMotor;
    private WPI_TalonSRX rotatorMotor;
    private double encoderOffset = 0;
    private static final double HALF_REVOLUTION_TICKS = 180 * RobotMap.DEGREES_TO_ENCODER_UNITS;
    private static final double FULL_REVOLUTION_TICKS = 360 * RobotMap.DEGREES_TO_ENCODER_UNITS;

    /**
     * Constructs a new swerve wheel for the specified wheel.
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
        double rawCurrent = this.getRotation() - this.encoderOffset;
        double current = Utils.leastResidue(rawCurrent, 4096);
        double target = heading * RobotMap.DEGREES_TO_ENCODER_UNITS;
        double error = target - current;
        if (Math.abs(error) > HALF_REVOLUTION_TICKS) {
            error = -(Math.signum(error) * (FULL_REVOLUTION_TICKS - Math.abs(error)));
        }
        this.rotatorMotor.set(ControlMode.Position, rawCurrent + error);
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

    /**
     * Sets PID gains to the rotator motor.
     * @param pid the PID gains for the rotator motor.
     */
    private void setRotatorPID(RobotMap.PID pid) {
        this.rotatorMotor.config_kP(0, pid.kP);
        this.rotatorMotor.config_kI(0, pid.kI);
        this.rotatorMotor.config_kD(0, pid.kD);
        this.rotatorMotor.config_kF(0, pid.kF);
    }

    /**
     * Sets PID gains to the drive motor.
     * @param pid the PID gains for the drive motor.
     */
    private void setDrivePID(RobotMap.PID pid) {
        this.driveMotor.config_kP(0, pid.kP);
        this.driveMotor.config_kI(0, pid.kI);
        this.driveMotor.config_kD(0, pid.kD);
        this.driveMotor.config_kF(0, pid.kF);
    }

    /**
     * Sets the percent output of the drive and rotator motors.
     * @param speed the percent output [-1, 1] of the drive motor.
     * @param heading the percent output [-1, 1] of the rotator motor.
     */
    public void setPercentOutput(double speed, double heading) {
        // TODO: safety check speeds
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

    /**
     * Gets the current rotation of the rotator motor in encoder units.
     * @return the current rotation in encoder units.
     */
    public double getRotation() {
        return this.rotatorMotor.getSelectedSensorPosition();
    }

    /**
     * Returns the current encoder values as a {@link WheelVector}.
     * <p>Note that the units of the vector are incorrect (encoder units rather than
     * percent/degrees).</p>
     *
     * @return a vector containing the current encoder values in encoder units.
     */
    public WheelVector encoderValues() {
        // TODO: this should not be a WheelVector because its units do not match those of the type
        return new WheelVector(driveMotor.getSelectedSensorVelocity(),
                rotatorMotor.getSelectedSensorPosition());
    }

    /**
     * Sets the encoder's current position to be equal to its least residue
     * mod the number of units in one revolution.
     */
    public void modularizeAbsoluteEncoder() {
        this.rotatorMotor.setSelectedSensorPosition(
                (int) (this.rotatorMotor.getSelectedSensorPosition() % FULL_REVOLUTION_TICKS));
    }

    /**
     * Resets the rotator encoder offset such that the specified heading is considered 0.
     * @param offset the raw encoder value to consider as forward (0).
     */
    public void setRotationOffset(double offset) {
        this.encoderOffset = offset;
    }
}