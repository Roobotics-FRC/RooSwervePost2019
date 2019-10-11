package frc.team4373.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4373.robot.input.WheelVector;
import frc.team4373.robot.subsystems.Drivetrain;
import frc.team4373.robot.subsystems.SwerveWheel;

/**
 * A Javadoc template. TODO: Update DriveFromShuffleboard Javadoc.
 */
public class DriveFromShuffleboard extends Command {
    private Drivetrain drivetrain;

    public DriveFromShuffleboard() {
        requires(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    protected void initialize() {
        SmartDashboard.putNumber("L1 Vel", 0);
        SmartDashboard.putNumber("L2 Vel", 0);
        SmartDashboard.putNumber("R1 Vel", 0);
        SmartDashboard.putNumber("R2 Vel", 0);

        SmartDashboard.putNumber("L1 Ang", 0);
        SmartDashboard.putNumber("L2 Ang", 0);
        SmartDashboard.putNumber("R1 Ang", 0);
        SmartDashboard.putNumber("R2 Ang", 0);

        SmartDashboard.putBoolean("Closed Loop", false);
    }

    @Override
    protected void execute() {
        SmartDashboard.putBoolean("Driving From Shuffleboard", true);

        WheelVector vecr1 = new WheelVector(SmartDashboard.getNumber("R1 Vel", 0),
                SmartDashboard.getNumber("R1 Ang", 0));
        WheelVector vecl1 = new WheelVector(SmartDashboard.getNumber("L1 Vel", 0),
                SmartDashboard.getNumber("L1 Ang", 0));
        WheelVector vecl2 = new WheelVector(SmartDashboard.getNumber("L2 Vel", 0),
                SmartDashboard.getNumber("L2 Ang", 0));
        WheelVector vecr2 = new WheelVector(SmartDashboard.getNumber("R2 Vel", 0),
                SmartDashboard.getNumber("R2 Ang", 0));

        if (SmartDashboard.getBoolean("Closed Loop", false)) {
            this.drivetrain.setWheelsPercOut(new WheelVector[]{vecr1, vecl1, vecl2, vecr2});
        } else {
            this.drivetrain.setWheelsPID(new WheelVector[]{vecr1, vecl1, vecl2, vecr2});
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        SmartDashboard.putBoolean("Driving From Shuffleboard", false);
        this.drivetrain.zeroMotors();
    }

    @Override
    protected void interrupted() {
        this.end();
    }
}
