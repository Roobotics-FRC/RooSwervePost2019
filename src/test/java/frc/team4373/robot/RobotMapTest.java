package frc.team4373.robot;

import frc.team4373.robot.subsystems.Drivetrain;
import org.junit.jupiter.api.Test;


import static org.junit.jupiter.api.Assertions.*;

/**
 * A Javadoc template. TODO: Update RobotMapTest Javadoc.
 */
class RobotMapTest {
    @Test
    void testMappings() {
        assertEquals(TestRobotMap.ROBOT_TRACKWIDTH, RobotMap.ROBOT_TRACKWIDTH);
        assertEquals(TestRobotMap.ROBOT_WHEELBASE, RobotMap.ROBOT_WHEELBASE);
        assertEquals(TestRobotMap.WHEEL_DIAMETER, RobotMap.WHEEL_DIAMETER);

        // OI devices
        assertEquals(TestRobotMap.DRIVE_JOYSTICK_PORT, RobotMap.DRIVE_JOYSTICK_PORT);
        assertEquals(TestRobotMap.OPERATOR_JOYSTICK_PORT, RobotMap.OPERATOR_JOYSTICK_PORT);

        // Buttons
        assertEquals(TestRobotMap.BUTTON_DRIVE_FROM_SHUFFLEBOARD,
                RobotMap.BUTTON_DRIVE_FROM_SHUFFLEBOARD);
        assertEquals(TestRobotMap.BUTTON_DRIVE_WITH_JOYSTICK, RobotMap.BUTTON_DRIVE_WITH_JOYSTICK);
        assertEquals(TestRobotMap.BUTTON_KILL_AUTONS, RobotMap.BUTTON_KILL_AUTONS);
        assertEquals(TestRobotMap.BUTTON_RESET_ORIENTATION, RobotMap.BUTTON_RESET_ORIENTATION);
        assertEquals(TestRobotMap.BUTTON_SWERVE_DRIVE_WITH_JOYSTICK,
                RobotMap.BUTTON_SWERVE_DRIVE_WITH_JOYSTICK);

        // Wheels et al.
        assertEquals(TestRobotMap.MAX_WHEEL_SPEED, RobotMap.MAX_WHEEL_SPEED);
        assertEquals(TestRobotMap.PID_IDX, RobotMap.PID_IDX);
        assertEquals(TestRobotMap.WHEEL_COUNT, RobotMap.WHEEL_COUNT);
        assertEquals(TestRobotMap.WHEEL_ENCODER_TICKS_PER_REV,
                RobotMap.WHEEL_ENCODER_TICKS_PER_REV);

        // Conversion factors
        assertEquals(TestRobotMap.DEGREES_TO_ENCODER_UNITS, RobotMap.DEGREES_TO_ENCODER_UNITS);
        assertEquals(TestRobotMap.DEGREES_TO_PIGEON_UNITS, RobotMap.DEGREES_TO_PIGEON_UNITS);
        assertEquals(TestRobotMap.ENCODER_UNITS_TO_IN, RobotMap.ENCODER_UNITS_TO_IN);

        // CAN chain identifiers
        assertEquals(TestRobotMap.PIGEON_PORT, RobotMap.PIGEON_PORT);

        // Timing
        assertEquals(TestRobotMap.TALON_TIMEOUT_MS, RobotMap.TALON_TIMEOUT_MS);
        assertEquals(TestRobotMap.SCHEDULER_EXEC_RATE, RobotMap.SCHEDULER_EXEC_RATE);

        // Programmatic resources
        assertEquals(TestRobotMap.FP_EQUALITY_THRESHOLD, RobotMap.FP_EQUALITY_THRESHOLD);

        // Motion profile measurements
        assertEquals(TestRobotMap.MAX_VELOCITY, RobotMap.MAX_VELOCITY);
        assertEquals(TestRobotMap.MAX_ACCEL, RobotMap.MAX_ACCEL);
        assertEquals(TestRobotMap.MAX_JERK, RobotMap.MAX_JERK);

        // Miscellaneous PID gains
        assertEquals(TestRobotMap.MOT_PROF_PID, RobotMap.MOT_PROF_PID);

        for (Drivetrain.WheelID w: Drivetrain.WheelID.values()) {
            assertEquals(TestRobotMap.getDriveMotorConfig(w), RobotMap.getDriveMotorConfig(w));
            assertEquals(TestRobotMap.getRotatorMotorConfig(w), RobotMap.getRotatorMotorConfig(w));
        }
    }
}