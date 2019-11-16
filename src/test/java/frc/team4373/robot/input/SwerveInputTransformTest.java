package frc.team4373.robot.input;

import org.junit.jupiter.api.*;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SwerveInputTransformTest {

    @BeforeEach
    void setUp() {
    }

    @AfterEach
    void tearDown() {
    }

    @Test
    void process() {
        WheelVector.VectorSet set = SwerveInputTransform.processNorthUp(0, 0, 1, 0);
        assertTrue(set.equals(new WheelVector.VectorSet(
                new WheelVector(1, 0),
                new WheelVector(1, 0),
                new WheelVector(1, 0),
                new WheelVector(1, 0))));

        WheelVector.VectorSet set2 = SwerveInputTransform.processNorthUp(0, 1, 0, 0);
        assertTrue(set2.equals(new WheelVector.VectorSet(
                new WheelVector(1, 90),
                new WheelVector(1, 90),
                new WheelVector(1, 90),
                new WheelVector(1, 90))));


        WheelVector.VectorSet set3 = SwerveInputTransform.processNorthUp(0, 1, 0, -90);
        assertTrue(set3.equals(new WheelVector.VectorSet(
                new WheelVector(1, 180),
                new WheelVector(1, 180),
                new WheelVector(1, 180),
                new WheelVector(1, 180))));
    }
}