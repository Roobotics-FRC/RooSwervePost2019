package frc.team4373.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.team4373.robot.input.OI;
import frc.team4373.robot.input.WheelVector;
import frc.team4373.robot.subsystems.Drivetrain;

public class DriveWithJoystick extends Command {
    private Drivetrain drivetrain;

    public DriveWithJoystick() {
        requires(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    public void initialize() {
        this.drivetrain.zeroMotors();
    }

    @Override
    public void execute() {
        double twist = OI.getInstance().getDriveJoystick().rooGetZ();
        double forward = OI.getInstance().getDriveJoystick().rooGetY();
        WheelVector vec = new WheelVector(forward, twist);
        this.drivetrain.setWheelsPercOut(new WheelVector.VectorSet(vec, vec, vec, vec));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end() {
        this.drivetrain.zeroMotors();
    }

    @Override
    public void interrupted() {
        this.end();
    }
}



