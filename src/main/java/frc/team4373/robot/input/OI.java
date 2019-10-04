package frc.team4373.robot.input;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.commands.ClearSubsystemCommand;
import frc.team4373.robot.commands.teleop.*;
import frc.team4373.robot.input.filters.FineGrainedPiecewiseFilter;
import frc.team4373.robot.input.filters.XboxAxisFilter;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * OI provides access to operator interface devices.
 */
public class OI {
    private static volatile OI oi = null;
    private RooJoystick<FineGrainedPiecewiseFilter> driveJoystick;
    private RooJoystick<XboxAxisFilter> operatorJoystick;

    private JoystickButton swerveDriveButton;
    private JoystickButton shuffleboardDriveButton;
    private JoystickButton joystickDriveButton;
    private JoystickButton killAutonButton;

    private OI() {
        this.driveJoystick =
                new RooJoystick<>(RobotMap.DRIVE_JOYSTICK_PORT, new FineGrainedPiecewiseFilter());
        this.operatorJoystick =
                new RooJoystick<>(RobotMap.OPERATOR_JOYSTICK_PORT, new XboxAxisFilter());


        swerveDriveButton = new JoystickButton(driveJoystick,
                RobotMap.BUTTON_SWERVE_DRIVE_WITH_JOYSTICK);
        swerveDriveButton.whenPressed(new SwerveDriveWithJoystick());

        shuffleboardDriveButton = new JoystickButton(driveJoystick,
                RobotMap.BUTTON_DRIVE_FROM_SHUFFLEBOARD);
        shuffleboardDriveButton.whenPressed(new DriveFromShuffleboard());

        joystickDriveButton = new JoystickButton(driveJoystick,
                RobotMap.BUTTON_DRIVE_WITH_JOYSTICK);
        joystickDriveButton.whenPressed(new DriveWithJoystick());

        killAutonButton = new JoystickButton(driveJoystick,
                RobotMap.BUTTON_KILL_AUTONS);
        killAutonButton.whenPressed(new ClearSubsystemCommand(Drivetrain.getInstance()));
    }

    /**
     * The getter for the OI singleton.
     *
     * @return The static OI singleton object.
     */
    public static OI getInstance() {
        if (oi == null) {
            synchronized (OI.class) {
                if (oi == null) {
                    oi = new OI();
                }
            }
        }
        return oi;
    }

    /**
     * Gets the drive joystick controlling the robot.
     * @return The drive joystick controlling the robot.
     */
    public RooJoystick getDriveJoystick() {
        return this.driveJoystick;
    }

    /**
     * Gets the operator joystick controlling the robot.
     * @return The operator joystick controlling the robot.
     */
    public RooJoystick getOperatorJoystick() {
        return this.operatorJoystick;
    }
}
