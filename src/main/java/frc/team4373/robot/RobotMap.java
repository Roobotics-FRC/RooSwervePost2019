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

    public static class MotorConfig {
        public final int port;
        public final NeutralMode neutralMode;
        public final boolean inverted;

        public MotorConfig(int port, NeutralMode neutralMode, boolean inverted) {
            this.port = port;
            this.neutralMode = neutralMode;
            this.inverted = inverted;
        }

    }

    // Motor CAN chain identifiers
    public static MotorConfig getDriveMotorConfig(Drivetrain.MotorID motorID) {
        switch (motorID) {
            case LEFT_1:
                return new MotorConfig(11, NeutralMode.Brake, false);
            case LEFT_2:
                return new MotorConfig(13, NeutralMode.Brake, false);
            case RIGHT_1:
                return new MotorConfig(15, NeutralMode.Brake, false);
            case RIGHT_2:
                return new MotorConfig(17, NeutralMode.Brake, false);
            default:
                return getDriveMotorConfig(Drivetrain.MotorID.LEFT_1);
        }
    }
    public static MotorConfig getRotatorMotorConfig(Drivetrain.MotorID motorID) {
        switch (motorID) {
            case LEFT_1:
                return new MotorConfig(12, NeutralMode.Brake, false);
            case LEFT_2:
                return new MotorConfig(14, NeutralMode.Brake, false);
            case RIGHT_1:
                return new MotorConfig(16, NeutralMode.Brake, false);
            case RIGHT_2:
                return new MotorConfig(18, NeutralMode.Brake, false);
            default:
                return getRotatorMotorConfig(Drivetrain.MotorID.LEFT_1);
        }
        // Equivalent to: `return getDriveMotor(motorID) + 1;`
    }

    // PID- and sensor-related constants
    public static class PID {
        public double kP;
        public double kI;
        public double kD;

        PID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }

}