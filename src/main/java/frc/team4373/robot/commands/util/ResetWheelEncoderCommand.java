package frc.team4373.robot.commands.util;

import edu.wpi.first.wpilibj.command.Command;
import frc.team4373.robot.subsystems.Drivetrain;

public class ResetWheelEncoderCommand extends Command {
    private Drivetrain.WheelID wheelID;
    private Drivetrain drivetrain;

    public ResetWheelEncoderCommand(Drivetrain.WheelID wheelID) {
        requires(this.drivetrain = Drivetrain.getInstance());
        this.wheelID = wheelID;
    }

    @Override
    protected void execute() {
        this.drivetrain.resetEncoder(wheelID);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
