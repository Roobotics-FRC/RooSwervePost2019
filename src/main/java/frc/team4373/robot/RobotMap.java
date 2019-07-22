package frc.team4373.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * Holds various mappings and constants.
 */

public class RobotMap {
    // Sensor phase configuration
    public static final boolean DRIVETRAIN_RIGHT_ENCODER_PHASE = false;
    public static final boolean DRIVETRAIN_LEFT_ENCODER_PHASE = false;

    // OI devices
    public static final int DRIVE_JOYSTICK_PORT = 0;
    public static final int OPERATOR_JOYSTICK_PORT = 1;

    // Wheels et al.
    public static final double MAX_WHEEL_VELOCITY = 1; // TODO: calculate this
    public static final double WHEEL_ENCODER_TICKS = 4096;

    public static class MotorConfig {
        public final int port;
        public final boolean inverted;
        public final NeutralMode neutralMode;

        public MotorConfig(int port, boolean inverted, NeutralMode neutralMode) {
            this.port = port;
            this.inverted = inverted;
            this.neutralMode = neutralMode;
        }

    }

    // Motor CAN chain identifiers
    public static final int PIGEON_PORT = 19;
    public static MotorConfig getDriveMotorConfig(Drivetrain.MotorID motorID) {
        switch (motorID) {
            case LEFT_1:
                return new MotorConfig(11, false, NeutralMode.Brake);
            case LEFT_2:
                return new MotorConfig(13, false, NeutralMode.Brake);
            case RIGHT_1:
                return new MotorConfig(15, false, NeutralMode.Brake);
            case RIGHT_2:
                return new MotorConfig(17, false, NeutralMode.Brake);
            default:
                return getDriveMotorConfig(Drivetrain.MotorID.LEFT_1);
        }
    }
    public static MotorConfig getRotatorMotorConfig(Drivetrain.MotorID motorID) {
        switch (motorID) {
            case LEFT_1:
                return new MotorConfig(12, false, NeutralMode.Brake);
            case LEFT_2:
                return new MotorConfig(14, false, NeutralMode.Brake);
            case RIGHT_1:
                return new MotorConfig(16, false, NeutralMode.Brake);
            case RIGHT_2:
                return new MotorConfig(18, false, NeutralMode.Brake);
            default:
                return getRotatorMotorConfig(Drivetrain.MotorID.LEFT_1);
        }
        // Equivalent to: `return getDriveMotor(motorID) + 1;`
    }

    // PID- and sensor-related constants
    public static class PID {
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