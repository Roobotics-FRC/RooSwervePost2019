package frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4373.robot.subsystems.Drivetrain;
import frc.team4373.robot.subsystems.LoggableDrivetrain;

/**
 * A Javadoc template. TODO: Update RunRunLogCOmmand Javadoc.
 */
public class RunRunLogCommand extends Command {
    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void execute() {
        Scheduler.getInstance().add(new RunLogCommand(
                Drivetrain.getInstance(),
                SmartDashboard.getNumber("log_duration", 0),
                SmartDashboard.getNumber("log_velocity", 0),
                SmartDashboard.getBoolean("log_left", false),
                SmartDashboard.getBoolean("log_right", false)));

        // Scheduler.getInstance().add(new RunLogCommand(
        //         new LoggableDrivetrain() {
        //             @Override
        //             public void setLeft(double v) {
        //
        //             }
        //
        //             @Override
        //             public void setRight(double v) {
        //
        //             }
        //
        //             @Override
        //             public double getLeftPercent() {
        //                 return 0;
        //             }
        //
        //             @Override
        //             public double getRightPercent() {
        //                 return 0;
        //             }
        //
        //             @Override
        //             public double getLeftVelocity() {
        //                 return 0;
        //             }
        //
        //             @Override
        //             public double getRightVelocity() {
        //                 return 0;
        //             }
        //
        //             @Override
        //             public double getLeftPosition() {
        //                 return 0;
        //             }
        //
        //             @Override
        //             public double getRightPosition() {
        //                 return 0;
        //             }
        //
        //             @Override
        //             public double getYaw() {
        //                 return 0;
        //             }
        //
        //             @Override
        //             protected void initDefaultCommand() {
        //
        //             }
        //         },
        //         SmartDashboard.getNumber("log_duration", 0),
        //         SmartDashboard.getNumber("log_velocity", 0),
        //         SmartDashboard.getBoolean("log_left", false),
        //         SmartDashboard.getBoolean("log_right", false)));
    }
}
