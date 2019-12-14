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

        WheelVector.VectorSet set4 = SwerveInputTransform.processTranslation(1, 1);
        assertTrue(set4.equals(new WheelVector.VectorSet(
                new WheelVector(1, 45),
                new WheelVector(1, 45),
                new WheelVector(1, 45),
                new WheelVector(1, 45))));


        WheelVector.VectorSet set5 = SwerveInputTransform.processTranslation(0.5, 1);
        assertTrue(set5.equals(new WheelVector.VectorSet(
                new WheelVector(1, 90 - Math.atan(2)),
                new WheelVector(1, 90 - Math.atan(2)),
                new WheelVector(1, 90 - Math.atan(2)),
                new WheelVector(1, 90 - Math.atan(2)))));

        WheelVector.VectorSet set6 = SwerveInputTransform.processTranslation(0.1, 0.1);
        assertTrue(set6.equals(new WheelVector.VectorSet(
                new WheelVector(Math.sqrt(0.02), 45),
                new WheelVector(Math.sqrt(0.02), 45),
                new WheelVector(Math.sqrt(0.02), 45),
                new WheelVector(Math.sqrt(0.02), 45))));

        WheelVector.VectorSet set7 = SwerveInputTransform.processTranslation(-1, 0);
        assertTrue(set7.equals(new WheelVector.VectorSet(
                new WheelVector(1,270),
                new WheelVector(1, 270),
                new WheelVector(1, 270),
                new WheelVector(1, 270))));

    }
}