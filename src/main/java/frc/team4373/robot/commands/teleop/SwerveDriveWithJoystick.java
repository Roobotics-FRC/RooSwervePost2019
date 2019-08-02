package frc.team4373.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.team4373.robot.Utils;
import frc.team4373.robot.input.OI;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * A command that allows control of the swerve drive using the joystick.
 */
public class SwerveDriveWithJoystick extends Command {
    private Drivetrain drivetrain;

    public SwerveDriveWithJoystick() {
        requires(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {

    }

    @Override
    protected void execute() {
        double x = OI.getInstance().getDriveJoystick().rooGetX();
        double y = OI.getInstance().getDriveJoystick().rooGetY();
        double z = OI.getInstance().getDriveJoystick().rooGetZFiltered();
        // double angle = Utils.calculateYOffset(x, y);
        double angle = OI.getInstance().getDriveJoystick().getAngle();
        //TODO: implement
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        this.drivetrain.zeroMotors();
    }

    @Override
    protected void interrupted() {
        this.end();
    }
}
