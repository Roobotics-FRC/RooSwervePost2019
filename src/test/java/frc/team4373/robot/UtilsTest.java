package frc.team4373.robot;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class UtilsTest {

    @Test
    void normalizeAngle() {

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
        assertEquals(Utils.leastResidue(0, 0), 0);
    }
}