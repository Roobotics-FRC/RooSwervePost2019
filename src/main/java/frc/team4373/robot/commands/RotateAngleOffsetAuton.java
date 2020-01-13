package frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.Utils;
import frc.team4373.robot.input.*;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * Rotates the robot a specified number of degrees.
 */
public class RotateAngleOffsetAuton extends PIDCommand {
    private static final double THRESHOLD = 1;
    private static final RobotMap.PID pid = new RobotMap.PID(0, 0.001, 0, 0);

    private Drivetrain drivetrain;
    private double offset;
    private double targetAngle;

    public RotateAngleOffsetAuton(double offset) {
        super("RotateAngleAuton", pid.kP, pid.kI, pid.kD);
        requires(this.drivetrain = Drivetrain.getInstance());
        this.offset = offset;
    }

    @Override
    protected void initialize() {
        targetAngle = drivetrain.getPigeonYawRaw() + offset;
        this.setSetpoint(0);
    }

    @Override
    protected boolean isFinished() {
        System.out.println("targetAngle = " + targetAngle);
        System.out.println("drivetrain.getAngle() = " + drivetrain.getPigeonYawRaw());
        return Math.abs(targetAngle - drivetrain.getPigeonYawRaw()) < THRESHOLD;
    }

    @Override
    protected double returnPIDInput() {
        return targetAngle - drivetrain.getPigeonYawRaw();
    }

    @Override
    protected void usePIDOutput(double output) {
        System.out.println("output = " + output);
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
