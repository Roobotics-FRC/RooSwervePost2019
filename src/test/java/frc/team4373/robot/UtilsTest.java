package frc.team4373.robot;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class UtilsTest {

    @Test
    void normalizeAngle() {
        assertEquals(Utils.normalizeAngle(30), 30);
        assertEquals(Utils.normalizeAngle(-30), 330);
        assertEquals(Utils.normalizeAngle(0), 0);
        assertEquals(Utils.normalizeAngle(360), 0);
        assertEquals(Utils.normalizeAngle(450),90);
        assertEquals(Utils.normalizeAngle(-640),80);
    }

    @Test
    void calculateYOffset() {
        assertEquals(Utils.calculateYOffset(0, 0.8), 0);
        assertEquals(Utils.calculateYOffset(1, 1), 45);
        assertEquals(Utils.calculateYOffset(Math.sqrt(3) / 3, 1 / 3d), 60);
        assertEquals(Utils.calculateYOffset(1, 0), 90);
        assertEquals(Utils.calculateYOffset(1, -1), 135);
        assertEquals(Utils.calculateYOffset(0, -1), 180);
        assertEquals(Utils.calculateYOffset(-1, -1), 225);
        assertEquals(Utils.calculateYOffset(-0.5, 0), 270);
        assertEquals(Utils.calculateYOffset(-0.5, 0.5), 315);
    }

    @Test
    void leastResidue() {
        // assertThrows(IllegalArgumentException.class, () -> Utils.leastResidue(2, 0));
        // assertThrows(IllegalArgumentException.class, () -> Utils.leastResidue(2, -2));
        assertEquals(Utils.leastResidue(5, 2), 1);
        assertEquals(Utils.leastResidue(-5, 2), 1);
        assertEquals(Utils.leastResidue(360, 360), 0);
        assertEquals(Utils.leastResidue(400, 100), 0);
        assertEquals(Utils.leastResidue(380, 360), 20);
        assertEquals(Utils.leastResidue(-450, 360), 270);
        assertEquals(Utils.leastResidue(0.5, 360), 0.5);
        assertEquals(Utils.leastResidue(360.5, 360), 0.5);
        assertEquals(Utils.leastResidue(-0.5, 2), 1.5);
        // assertThrows(IllegalArgumentException.class, () -> Utils.leastResidue(0, 0));
    }
}