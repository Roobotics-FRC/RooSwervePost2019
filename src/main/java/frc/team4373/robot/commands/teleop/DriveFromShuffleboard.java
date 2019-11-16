package frc.team4373.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4373.robot.input.WheelVector;
import frc.team4373.robot.subsystems.Drivetrain;

import java.util.Arrays;

/**
 * Allows for control of the swerve drive using Shuffleboard.
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

        WheelVector.VectorSet vectors = new WheelVector.VectorSet(vecr1, vecr2, vecl1, vecl2);
        SmartDashboard.putString("vectors", vectors.toString());

        if (SmartDashboard.getBoolean("Closed Loop", false)) {
            this.drivetrain.setWheelsPID(vectors);
        } else {
            this.drivetrain.setWheelsPercOut(vectors);
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
