package frc.team4373.robot;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class RobotTest {

    @Test
    void constrainPercentOutput() {
        assertEquals(Robot.constrainPercentOutput(30), 1);
        assertEquals(Robot.constrainPercentOutput(1.1), 1);
        assertEquals(Robot.constrainPercentOutput(-30), -1);
        assertEquals(Robot.constrainPercentOutput(-1.1), -1);
        assertEquals(Robot.constrainPercentOutput(0), 0);
        assertEquals(Robot.constrainPercentOutput(0.2), 0.2);
        assertEquals(Robot.constrainPercentOutput(-0.2), -0.2);
        assertEquals(Robot.constrainPercentOutput(0.9), 0.9);
        assertEquals(Robot.constrainPercentOutput(-0.9), -0.9);
        assertEquals(Robot.constrainPercentOutput(100), 1) ;
        assertEquals(Robot.constrainPercentOutput(-100), -1) ;
        assertEquals(Robot.constrainPercentOutput(0.25), 0.25);
    }
}