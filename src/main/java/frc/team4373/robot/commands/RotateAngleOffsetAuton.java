package frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.Utils;
import frc.team4373.robot.input.*;
import frc.team4373.robot.subsystems.Drivetrain;


/**
 * Rotates the robot a specified number of degrees.
 */
public class RotateAngleOffsetAuton extends PIDCommand {
    private static final double THRESHOLD = 1;
    private static final RobotMap.PID pid = new RobotMap.PID(0, 0.1, 0, 0);

    private Drivetrain drivetrain;
    private double offset;
    private double targetAngle;

    /**
     * Constructs an offset rotator auton command.
     * @param offset the angle offset by which to rotate, in degrees.
     */
    public RotateAngleOffsetAuton(double offset) {
        super("RotateAngleAuton", pid.kP, pid.kI, pid.kD);
        requires(this.drivetrain = Drivetrain.getInstance());
        this.offset = offset;
    }

    @Override
    protected void initialize() {
        targetAngle = drivetrain.getPigeonYawRaw() + offset;
        this.setSetpoint(targetAngle);
        this.getPIDController().setOutputRange(-RobotMap.AUTON_TURN_SPEED, RobotMap.AUTON_TURN_SPEED);
    }

    @Override
    protected boolean isFinished() {
        SmartDashboard.putNumber("targetAngle", targetAngle);
        return Math.abs(targetAngle - drivetrain.getPigeonYawRaw()) < THRESHOLD;
    }

    @Override
    protected double returnPIDInput() {
        return drivetrain.getPigeonYawRaw();
    }

    @Override
    protected void usePIDOutput(double output) {
        SmartDashboard.putNumber("output", output);
        drivetrain.setWheelsPID(SwerveInputTransform.processOwnShipUp(output, 0, 0));
    }

    @Override
    protected void end() {
        drivetrain.setWheelsPercOut(WheelVector.VectorSet.ZERO);
    }

    @Override
    protected void interrupted() {
        end();
    }
}
