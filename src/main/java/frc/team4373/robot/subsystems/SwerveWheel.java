package frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.Utils;
import frc.team4373.robot.input.WheelVector;

import static com.ctre.phoenix.motorcontrol.ControlMode.MotionMagic;

/**
 * Represents a swerve wheel with two motors.
 */
public class SwerveWheel {
    private WPI_TalonSRX driveMotor;
    private WPI_TalonSRX rotatorMotor;
    private static final double HALF_REVOLUTION_TICKS = 180 * RobotMap.DEGREES_TO_ENCODER_UNITS;
    private static final double FULL_REVOLUTION_TICKS = 360 * RobotMap.DEGREES_TO_ENCODER_UNITS;
    private boolean isInverted = false;
    private double linearZeroPos = 0;

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
        this.rotatorMotor.configFeedbackNotContinuous(false, 1000);

        this.driveMotor.config_kF(RobotMap.PID_IDX, driveMotorConfig.gains.kF);
        this.driveMotor.config_kP(RobotMap.PID_IDX, driveMotorConfig.gains.kP);
        this.driveMotor.config_kI(RobotMap.PID_IDX, driveMotorConfig.gains.kI);
        this.driveMotor.config_kD(RobotMap.PID_IDX, driveMotorConfig.gains.kD);

        this.rotatorMotor.config_kF(RobotMap.PID_IDX, rotatorMotorConfig.gains.kF);
        this.rotatorMotor.config_kP(RobotMap.PID_IDX, rotatorMotorConfig.gains.kP);
        this.rotatorMotor.config_kI(RobotMap.PID_IDX, rotatorMotorConfig.gains.kI);
        this.rotatorMotor.config_kD(RobotMap.PID_IDX, rotatorMotorConfig.gains.kD);

        this.linearZeroPos = this.getRawLinearPos();
    }

    /**
     * Sets the swerve wheel's two motors' PID loops.
     * @param heading the heading, in degrees, at which to angle the wheel.
     * @param speed the percent of maximum speed at which to drive.
     */
    public void set(double speed, double heading) {
        if (speed == 0) {
            this.driveMotor.set(ControlMode.PercentOutput, 0);
            return;
        }

        heading *= RobotMap.DEGREES_TO_ENCODER_UNITS;

        double currentRotation = rotatorMotor.getSelectedSensorPosition();
        double rotationError = Math.IEEEremainder(heading - currentRotation,
                RobotMap.WHEEL_ENCODER_TICKS_PER_REV);

        // minimize azimuth rotation, reversing drive if necessary
        isInverted = Math.abs(rotationError) > 0.25 * RobotMap.WHEEL_ENCODER_TICKS_PER_REV;
        if (isInverted) {
            rotationError -= Math.copySign(0.5 * RobotMap.WHEEL_ENCODER_TICKS_PER_REV,
                    rotationError);
            speed = -speed;
        }

        this.rotatorMotor.set(ControlMode.Position, currentRotation + rotationError);
        this.driveMotor.set(ControlMode.Velocity, speed * RobotMap.MAX_WHEEL_SPEED);
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

    /**
     * Sets swerve wheel vectors via percent output.
     * @param speed the speed as a percent.
     * @param heading the rotation speed as a percent.
     */
    public void setPercentOutput(double speed, double heading) {
        // TODO: Safety check speeds
        this.driveMotor.set(ControlMode.PercentOutput, speed);
        this.rotatorMotor.set(ControlMode.PercentOutput, heading);
    }

    /**
     * Gets the current position of the drive encoder (i.e., distance traveled)
     * relative to the start position.
     * @return the distance traveled in encoder units.
     */
    public double getLinearPos() {
        return this.getRawLinearPos() - this.linearZeroPos;
    }

    /**
     * Gets the current, raw position of the drive encoder.
     * @return the position of the encoder in encoder units.
     */
    public double getRawLinearPos() {
        return this.driveMotor.getSelectedSensorPosition();
    }

    /**
     * Stops all motors.
     */
    public void stop() {
        this.driveMotor.set(ControlMode.PercentOutput, 0);
        this.rotatorMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * NOTE: this is not actually a WheelVector, as its units are incorrect.
     * @return a WheelVector of encoder values.
     */
    public WheelVector encoderValues() {
        return new WheelVector(driveMotor.getSelectedSensorVelocity(),
                rotatorMotor.getSelectedSensorPosition());
    }

    /**
     * Sets the encoder's current position to be equal to its least residue
     * mod the number of units in one revolution.
     */
    public void modularizeAbsoluteRotation() {
        this.rotatorMotor.setSelectedSensorPosition(
                (int) (this.rotatorMotor.getSelectedSensorPosition() % FULL_REVOLUTION_TICKS));
    }

    /**
     * Resets the rotator encoder offset such that the current heading is considered 0.
     */
    public void resetAbsoluteRotation() {
        this.rotatorMotor.setSelectedSensorPosition(0);
    }
}