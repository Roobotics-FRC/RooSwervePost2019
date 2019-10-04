package frc.team4373.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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


        SwerveWheel l1 = drivetrain.getSwerveWheel(Drivetrain.WheelID.LEFT_1);
        SwerveWheel l2 = drivetrain.getSwerveWheel(Drivetrain.WheelID.LEFT_2);
        SwerveWheel r1 = drivetrain.getSwerveWheel(Drivetrain.WheelID.RIGHT_1);
        SwerveWheel r2 = drivetrain.getSwerveWheel(Drivetrain.WheelID.RIGHT_2);

        if (SmartDashboard.getBoolean("Closed Loop", false)) {
            l1.setPercentOutput(SmartDashboard.getNumber("L1 Vel", 0),
                    SmartDashboard.getNumber("L1 Ang", 0));
            l2.setPercentOutput(SmartDashboard.getNumber("L2 Vel", 0),
                    SmartDashboard.getNumber("L2 Ang", 0));
            r1.setPercentOutput(SmartDashboard.getNumber("R1 Vel", 0),
                    SmartDashboard.getNumber("R1 Ang", 0));
            r2.setPercentOutput(SmartDashboard.getNumber("R2 Vel", 0),
                    SmartDashboard.getNumber("R2 Ang", 0));
        } else {
            l1.set(SmartDashboard.getNumber("L1 Ang", 0),
                    SmartDashboard.getNumber("L1 Vel", 0));
            l2.set(SmartDashboard.getNumber("L2 Ang", 0),
                    SmartDashboard.getNumber("L2 Vel", 0));
            r1.set(SmartDashboard.getNumber("R1 Ang", 0),
                    SmartDashboard.getNumber("R1 Vel", 0));
            r2.set(SmartDashboard.getNumber("R2 Ang", 0),
                    SmartDashboard.getNumber("R2 Vel", 0));
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
