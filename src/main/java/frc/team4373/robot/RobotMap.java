package frc.team4373.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.team4373.robot.subsystems.Drivetrain;

import java.util.Objects;

/**
 * Holds various mappings and constants.
 */

public final class RobotMap {
    private RobotMap() {}

    // Physical measurements
    // These are in inches, but units don't matter; only ratios are used
    public static final double ROBOT_TRACKWIDTH = 22;
    public static final double ROBOT_WHEELBASE = 27;
    public static final double WHEEL_DIAMETER = 6;

    // OI devices
    public static final int DRIVE_JOYSTICK_PORT = 0;
    public static final int OPERATOR_JOYSTICK_PORT = 1;

    // Buttons
    public static final int BUTTON_DRIVE_FROM_SHUFFLEBOARD = 8;
    public static final int BUTTON_DRIVE_WITH_JOYSTICK = 9;
    public static final int BUTTON_KILL_AUTONS = 10;
    public static final int BUTTON_RESET_ORIENTATION = 4;
    public static final int BUTTON_SWERVE_DRIVE_WITH_JOYSTICK = 7;

    // Wheels et al.
    public static final double MAX_WHEEL_SPEED = 8400; // TODO: calculate this
    public static final int PID_IDX = 0;
    public static final int WHEEL_COUNT = 4;
    public static final int WHEEL_ENCODER_TICKS_PER_REV = 4096;

    // Conversion factors
    public static final double DEGREES_TO_ENCODER_UNITS = WHEEL_ENCODER_TICKS_PER_REV / 360d;
    public static final double DEGREES_TO_PIGEON_UNITS = 8192d / 360d;
    public static final double ENCODER_UNITS_TO_IN = WHEEL_DIAMETER  * Math.PI
            / WHEEL_ENCODER_TICKS_PER_REV;

    // CAN chain identifiers
    public static final int PIGEON_PORT = 19;

    // Timing
    public static final int TALON_TIMEOUT_MS = 1000;
    public static final int SCHEDULER_EXEC_RATE = 50;

    // Programmatic resources
    public static final double FP_EQUALITY_THRESHOLD = 1e-5;

    // Motion profile measurements
    // TODO: calculate these
    public static final double MAX_VELOCITY = 1.6;
    public static final double MAX_ACCEL = 3.2;
    public static final double MAX_JERK = 60.0;

    // Miscellaneous PID gains
    public static final PID MOT_PROF_PID = new PID(0, 1, 0, 0);

    /**
     * Gets the drive motor configuration for the specified wheel.
     * @param wheelID the wheel whose configuration to fetch.
     * @return the drive motor configuration for the specified wheel.
     */
    public static MotorConfig getDriveMotorConfig(Drivetrain.WheelID wheelID) {
        switch (wheelID) {
            case LEFT_1:
                return new MotorConfig(11, true, NeutralMode.Brake, true,
                        new PID(0, 0.25, 0, 0));
            case LEFT_2:
                return new MotorConfig(13, true, NeutralMode.Brake, true,
                        new PID(0, 0.25, 0, 0));
            case RIGHT_1:
                return new MotorConfig(15, false, NeutralMode.Brake, true,
                        new PID(0, 0.25, 0, 0));
            case RIGHT_2:
                return new MotorConfig(17, false, NeutralMode.Brake, true,
                        new PID(0, 0.25, 0, 0));
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
                return new MotorConfig(12, true, NeutralMode.Brake, false,
                        new PID(0, 3.5, 0, 3));
            case LEFT_2:
                return new MotorConfig(14, true, NeutralMode.Brake, false,
                        new PID(0, 3.5, 0, 3));
            case RIGHT_1:
                return new MotorConfig(16, true, NeutralMode.Brake, false,
                        new PID(0, 3.5, 0, 3));
            case RIGHT_2:
                return new MotorConfig(18, true, NeutralMode.Brake, false,
                        new PID(0, 3.5, 0, 3));
            default:
                return getRotatorMotorConfig(Drivetrain.WheelID.LEFT_1);
        }
    }

    public static final class MotorConfig {
        public final int port;
        public final boolean inverted;
        public final NeutralMode neutralMode;
        public final boolean encoderPhase;
        public final PID gains;

        /**
         * Constructs a new MotorConfig.
         * @param port the port to which the motor is attached.
         * @param inverted whether to invert motor output values.
         * @param neutralMode the motor's passive neutral mode.
         */
        public MotorConfig(int port, boolean inverted,
                           NeutralMode neutralMode, boolean encoderPhase, PID gains) {
            this.port = port;
            this.inverted = inverted;
            this.neutralMode = neutralMode;
            this.encoderPhase = encoderPhase;
            this.gains = gains;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            MotorConfig that = (MotorConfig) o;
            return port == that.port
                    && inverted == that.inverted
                    && encoderPhase == that.encoderPhase
                    && neutralMode == that.neutralMode
                    && gains.equals(that.gains);
        }

        @Override
        public int hashCode() {
            return Objects.hash(port, inverted, neutralMode, encoderPhase, gains);
        }
    }

    // PID- and sensor-related constants
    public static final class PID {
        public final double kF;
        public final double kP;
        public final double kI;
        public final double kD;

        /**
         * Constructs a new PID parameters object.
         * @param kF feed-forward gain.
         * @param kP proportional gain.
         * @param kI integral gain.
         * @param kD derivative gain.
         */
        public PID(double kF, double kP, double kI, double kD) {
            this.kF = kF;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            PID pid = (PID) o;
            return Double.compare(pid.kF, kF) == 0
                    && Double.compare(pid.kP, kP) == 0
                    && Double.compare(pid.kI, kI) == 0
                    && Double.compare(pid.kD, kD) == 0;
        }

        @Override
        public int hashCode() {
            return Objects.hash(kF, kP, kI, kD);
        }
    }
}