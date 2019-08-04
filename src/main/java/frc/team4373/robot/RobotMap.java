package frc.team4373.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * Holds various mappings and constants.
 */

public final class RobotMap {
    private RobotMap() {}

    public static final double ROBOT_WHEELBASE = -1;
    public static final double ROBOT_TRACKWIDTH = -1;

    // OI devices
    public static final int DRIVE_JOYSTICK_PORT = 0;
    public static final int OPERATOR_JOYSTICK_PORT = 1;

    // Wheels et al.
    public static final double MAX_WHEEL_SPEED = 1; // TODO: calculate this
    public static final double WHEEL_ENCODER_TICKS = 4096;
    public static final int HEADING_PID_IDX = 0;
    public static final double DEGREES_TO_ENCODER_TICKS = -1; //FIXME: calculate this

    public static final class MotorConfig {
        public final int port;
        public final boolean inverted;
        public final NeutralMode neutralMode;
        public final boolean encoderPhase;

        /**
         * Constructs a new MotorConfig.
         * @param port the port to which the motor is attached.
         * @param inverted whether to invert motor output values.
         * @param neutralMode the motor's passive neutral mode.
         */
        public MotorConfig(int port, boolean inverted,
                           NeutralMode neutralMode, boolean encoderPhase) {
            this.port = port;
            this.inverted = inverted;
            this.neutralMode = neutralMode;
            this.encoderPhase = encoderPhase;
        }

    }

    // Motor CAN chain identifiers
    public static final int PIGEON_PORT = 19;

    /**
     * Gets the drive motor configuration for the specified wheel.
     * @param wheelID the wheel whose configuration to fetch.
     * @return the drive motor configuration for the specified wheel.
     */
    public static MotorConfig getDriveMotorConfig(Drivetrain.WheelID wheelID) {
        switch (wheelID) {
            case LEFT_1:
                return new MotorConfig(11, false, NeutralMode.Brake, false);
            case LEFT_2:
                return new MotorConfig(13, false, NeutralMode.Brake, false);
            case RIGHT_1:
                return new MotorConfig(15, false, NeutralMode.Brake, false);
            case RIGHT_2:
                return new MotorConfig(17, false, NeutralMode.Brake, false);
            default:
                return getDriveMotorConfig(Drivetrain.WheelID.LEFT_1);
        }
    }

    /**
     * Gets the rotator motor configuration for the specified wheel.
     * @param wheelID the wheel whose configuration to fetch.
     * @return the rotator motor configuration for the specified wheel.
     */
    public static MotorConfig getRotatorMotorConfig(Drivetrain.WheelID wheelID) {
        switch (wheelID) {
            case LEFT_1:
                return new MotorConfig(12, false, NeutralMode.Brake, false);
            case LEFT_2:
                return new MotorConfig(14, false, NeutralMode.Brake, false);
            case RIGHT_1:
                return new MotorConfig(16, false, NeutralMode.Brake, false);
            case RIGHT_2:
                return new MotorConfig(18, false, NeutralMode.Brake, false);
            default:
                return getRotatorMotorConfig(Drivetrain.WheelID.LEFT_1);
        }
    }

    // PID- and sensor-related constants
    public static final class PID {
        public final double kP;
        public final double kI;
        public final double kD;

        PID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }

}