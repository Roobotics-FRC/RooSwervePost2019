package frc.team4373.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4373.robot.commands.RunRunLogCommand;
import frc.team4373.robot.commands.util.ResetWheelEncoderCommand;
import frc.team4373.robot.commands.util.SetWheelPIDCommand;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonCommand = null;
    public static SendableChooser<Drivetrain.WheelID> wheelChooser = new SendableChooser<>();

    /**
     * Constructor for the Robot class. Variable initialization occurs here;
     * WPILib-related setup should occur in {@link #robotInit}.
     */
    public Robot() {
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     *
     * <p>All SmartDashboard fields should be initially added here.</p>
     */
    @Override
    public void robotInit() {
        Drivetrain.getInstance().modularizeEncoders();

        for (Drivetrain.WheelID wheelID: Drivetrain.WheelID.values()) {
            SmartDashboard.putData("Reset " + wheelID.name() + " Enc",
                    new ResetWheelEncoderCommand(wheelID));
            wheelChooser.addOption(wheelID.name(), wheelID);
        }

        SmartDashboard.putData("Modified Wheel", wheelChooser);

        SmartDashboard.putNumber("D kP", 0.25);
        SmartDashboard.putNumber("D kI", 0);
        SmartDashboard.putNumber("D kD", 0);
        SmartDashboard.putNumber("D kF", 0);

        SmartDashboard.putNumber("R kP", 3.5);
        SmartDashboard.putNumber("R kI", 0);
        SmartDashboard.putNumber("R kD", 3);
        SmartDashboard.putNumber("R kF", 0);

        SmartDashboard.putData("Set Selected PID", new SetWheelPIDCommand());


        SmartDashboard.putNumber("log_duration", 0);
        SmartDashboard.putNumber("log_velocity", 0);
        SmartDashboard.putBoolean("log_left", false);
        SmartDashboard.putBoolean("log_right", false);

        SmartDashboard.putData("Run Log Command", new RunRunLogCommand());
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want run during disabled,
     * autonomous, teleoperated, and test.
     *
     * <p>This runs after the mode-specific periodic functions but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        Drivetrain.getInstance().logEncoders();
        // SmartDashboard.getNumber("L1 R kP", 0.25);
        // SmartDashboard.getNumber("L1 R kP", 0.25);
        // SmartDashboard.getNumber("L1 R kP", 0.25);
        // SmartDashboard.getNumber("L1 R kP", 0.25);
        // SmartDashboard.getNumber("L1 R kP", 0.25);
        // SmartDashboard.getNumber("L1 R kP", 0.25);
        // SmartDashboard.getNumber("L1 R kP", 0.25);
        // SmartDashboard.getNumber("L1 R kP", 0.25);
    }

    /**
     * This function is called once when Sandstorm mode starts.
     */
    @Override
    public void autonomousInit() {
    }

    /**
     * This function is called once when teleoperated mode starts.
     */
    @Override
    public void teleopInit() {
    }

    /**
     * This function is called periodically during Sandstorm mode.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * This function runs periodically in disabled mode.
     * It is used to verify that the selected auton configuration is legal.
     * <br>
     * <b>UNDER NO CIRCUMSTANCES SHOULD SUBSYSTEMS BE ACCESSED OR ENGAGED IN THIS METHOD.</b>
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * Constrains a percent output to [-1, 1].
     * @param output the percent output value to constrain.
     * @return the input percent output constrained to the safe range.
     */
    public static double constrainPercentOutput(double output) {
        if (output > 1) {
            return 1;
        }
        if (output < -1) {
            return -1;
        }
        return output;
    }
}
