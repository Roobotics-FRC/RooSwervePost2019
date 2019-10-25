package frc.team4373.robot.commands.util;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4373.robot.Robot;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.subsystems.Drivetrain;

public class SetWheelPIDCommand extends Command {
    private Drivetrain drivetrain;

    public SetWheelPIDCommand() {
        requires(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    protected void execute() {
        RobotMap.PID rotator = new RobotMap.PID(SmartDashboard.getNumber("R kF", 0),
                SmartDashboard.getNumber("R kP", 1),
                SmartDashboard.getNumber("R kI", 0),
                SmartDashboard.getNumber("R kD", 0));
        RobotMap.PID drive = new RobotMap.PID(SmartDashboard.getNumber("D kF", 0),
                SmartDashboard.getNumber("D kP", 0.25),
                SmartDashboard.getNumber("D kI", 0),
                SmartDashboard.getNumber("D kD", 0));
        Drivetrain.getInstance().setPID(Robot.wheelChooser.getSelected(), drive, rotator);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
