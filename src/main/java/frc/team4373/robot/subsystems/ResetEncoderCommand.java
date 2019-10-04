package frc.team4373.robot.subsystems;

import edu.wpi.first.wpilibj.command.Command;

public class ResetEncoderCommand extends Command {
    public ResetEncoderCommand() {
        requires(Drivetrain.getInstance());
        setTimeout(0.5);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Drivetrain.getInstance().resetEncoders();
        System.out.println("Encoders have been reset");
    }

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }
}