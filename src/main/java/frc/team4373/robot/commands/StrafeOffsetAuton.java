package frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.input.*;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * Moves the robot a specified number of inches.
 */
public class StrafeOffsetAuton extends PIDCommand {
    private static final double MOTOR_OUTPUT_THRESHOLD = 0.0625;
    private static final RobotMap.PID pid = new RobotMap.PID(0, 0.01, 0, 0);

    private Drivetrain drivetrain;
    private double inchOffset;
    private double targetPosition;
    private boolean finished = false;

    /**
     * Constructs an offset strafing auton command.
     * @param inchOffset the offset in inches to rotate.
     */
    public StrafeOffsetAuton(double inchOffset) {
        super("StrafeAuton", pid.kP, pid.kI, pid.kD);
        requires(this.drivetrain = Drivetrain.getInstance());
        this.inchOffset = inchOffset;
    }

    @Override
    protected void initialize() {
        targetPosition = drivetrain.getAverageDriveMotorPosition()
                * RobotMap.ENCODER_UNITS_TO_INCHES + inchOffset;
        this.setSetpoint(targetPosition);
        this.getPIDController().setOutputRange(
                -RobotMap.AUTON_STRAFE_SPEED, RobotMap.AUTON_STRAFE_SPEED);
        this.finished = false;
    }

    @Override
    protected boolean isFinished() {
        SmartDashboard.putNumber("targetPosition", targetPosition);
        return this.finished;
    }

    @Override
    protected double returnPIDInput() {
        return drivetrain.getAverageDriveMotorPosition() * RobotMap.ENCODER_UNITS_TO_INCHES;
    }

    @Override
    protected void usePIDOutput(double output) {
        SmartDashboard.putNumber("output", output);
        if (Math.abs(output) <= MOTOR_OUTPUT_THRESHOLD) {
            this.finished = true;
            return;
        }
        double setPointAngle = 90;
        WheelVector.VectorSet vectorSetPoint = new WheelVector.VectorSet(
                new WheelVector(output, setPointAngle),
                new WheelVector(output, setPointAngle),
                new WheelVector(output, setPointAngle),
                new WheelVector(output, setPointAngle)
        );
        drivetrain.setWheelsPID(vectorSetPoint);
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
