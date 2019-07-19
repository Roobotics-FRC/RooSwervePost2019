package frc.team4373.robot.input;

import frc.team4373.robot.RobotMap;
import frc.team4373.robot.input.filters.FineGrainedPiecewiseFilter;
import frc.team4373.robot.input.filters.XboxAxisFilter;

/**
 * OI provides access to operator interface devices.
 */
public class OI {
    private static volatile OI oi = null;
    private RooJoystick<FineGrainedPiecewiseFilter> driveJoystick;
    private RooJoystick<XboxAxisFilter> operatorJoystick;

    private OI() {
        this.driveJoystick =
                new RooJoystick<>(RobotMap.DRIVE_JOYSTICK_PORT, new FineGrainedPiecewiseFilter());
        this.operatorJoystick =
                new RooJoystick<>(RobotMap.OPERATOR_JOYSTICK_PORT, new XboxAxisFilter());        
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
