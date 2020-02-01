package frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.*;
import frc.team4373.robot.subsystems.*;

import java.util.Arrays;

/**
 * Removes any active commands from all subsystems.
 */
public class ClearSubsystemCommand extends Command {
    /**
     * Creates a new ClearSubsystemCommand to clear all current subsystem commands
     * and return them to default (i.e., teleop).
     */
    public ClearSubsystemCommand(Subsystem subsystem) {
        requires(subsystem);
        if (subsystem instanceof Drivetrain) {
            Scheduler.getInstance().removeAll();
        }
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}