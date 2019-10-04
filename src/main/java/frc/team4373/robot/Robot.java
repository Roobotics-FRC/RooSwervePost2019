package frc.team4373.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private WPI_TalonSRX steer;
    private WPI_TalonSRX drive;
    private WPI_TalonSRX steer2;
    private WPI_TalonSRX drive2;
    private WPI_TalonSRX steer3;
    private WPI_TalonSRX drive3;
    private WPI_TalonSRX steer4;
    private WPI_TalonSRX drive4;

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
        steer = new WPI_TalonSRX(12);
        steer.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        drive = new WPI_TalonSRX(11);
        drive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        steer2 = new WPI_TalonSRX(14);
        steer2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        drive2 = new WPI_TalonSRX(13);
        drive2.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        steer3 = new WPI_TalonSRX(16);
        steer3.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        drive3 = new WPI_TalonSRX(15);
        drive3.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        steer4 = new WPI_TalonSRX(18);
        steer4.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        drive4 = new WPI_TalonSRX(17);
        drive4.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
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
        SmartDashboard.putNumber("Steering 1 pos", steer.getSelectedSensorPosition());
        SmartDashboard.putNumber("Steering 1 vel", steer.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Driving 1 pos", drive.getSelectedSensorPosition());
        SmartDashboard.putNumber("Driving 1 vel", drive.getSelectedSensorVelocity());

        SmartDashboard.putNumber("Steering 2 pos", steer2.getSelectedSensorPosition());
        SmartDashboard.putNumber("Steering 2 vel", steer2.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Driving 2 pos", drive2.getSelectedSensorPosition());
        SmartDashboard.putNumber("Driving 2 vel", drive2.getSelectedSensorVelocity());

        SmartDashboard.putNumber("Steering 3 pos", steer3.getSelectedSensorPosition());
        SmartDashboard.putNumber("Steering 3 vel", steer3.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Driving 3 pos", drive3.getSelectedSensorPosition());
        SmartDashboard.putNumber("Driving 3 vel", drive3.getSelectedSensorVelocity());

        SmartDashboard.putNumber("Steering 4 pos", steer4.getSelectedSensorPosition());
        SmartDashboard.putNumber("Steering 4 vel", steer4.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Driving 4 pos", drive4.getSelectedSensorPosition());
        SmartDashboard.putNumber("Driving 4 vel", drive4.getSelectedSensorVelocity());
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
