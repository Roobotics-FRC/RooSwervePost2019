package frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.Utils;
import frc.team4373.robot.input.*;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * A Javadoc template. TODO: Update RotateAuton Javadoc.
 */
public class RotateAngleAuton extends PIDCommand {
    private static final double THRESHOLD = 1;
    private static final RobotMap.PID pid = new RobotMap.PID(0, 0.001, 0, 0);

    private Drivetrain drivetrain;
    private boolean firstTime = true;
    private double targetAngle;

    public RotateAngleAuton(double offset) {
        super("RotateAngleAuton", pid.kP, pid.kI, pid.kD);
        requires(this.drivetrain = Drivetrain.getInstance());
        targetAngle = offset;
    }

    @Override
    protected void initialize() {
        targetAngle += drivetrain.getAngle();
        targetAngle = Utils.normalizeAngle(targetAngle);
        firstTime = true;
    }

    @Override
    protected void execute() {
        if (firstTime) {
            targetAngle += drivetrain.getAngle();
            targetAngle = Utils.normalizeAngle(targetAngle);
            firstTime = false;
        }
    }

    @Override
    protected boolean isFinished() {
        System.out.println("targetAngle = " + targetAngle);
        System.out.println("drivetrain.getAngle() = " + drivetrain.getAngle());
        return (targetAngle - drivetrain.getAngle()) < THRESHOLD;
    }

    @Override
    protected double returnPIDInput() {
        return targetAngle - drivetrain.getAngle();
    }

    @Override
    protected void usePIDOutput(double output) {
        drivetrain.setWheelsPID(SwerveInputTransform.processRotation(output));
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
