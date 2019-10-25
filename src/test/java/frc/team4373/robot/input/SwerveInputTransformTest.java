package frc.team4373.robot.input;

import org.junit.jupiter.api.*;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

/**
 * A Javadoc template. TODO: Update SwerveInputTransformTest Javadoc.
 */
class SwerveInputTransformTest {

    @BeforeEach
    void setUp() {
    }

    @AfterEach
    void tearDown() {
    }

    @Test
    void process() {
        WheelVector[] arr = SwerveInputTransform.process(0, 0, 1, 0);
        double[] headings = new double[4];
        double[] speeds = new double[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = arr[i].angle;
            speeds[i] = arr[i].speed;
        }
        assertArrayEquals(new double[] {0, 0, 0, 0}, headings);
        assertArrayEquals(new double[] {1, 1, 1, 1}, speeds);


        WheelVector[] arr2 = SwerveInputTransform.process(0, 1, 0, 0);
        double[] headings2 = new double[4];
        double[] speeds2 = new double[4];
        for (int i = 0; i < 4; i++) {
            headings2[i] = arr2[i].angle;
            speeds2[i] = arr2[i].speed;
        }
        assertArrayEquals(new double[] {90, 90, 90, 90}, headings2);
        assertArrayEquals(new double[] {1, 1, 1, 1}, speeds2);


        WheelVector[] arr3 = SwerveInputTransform.process(0, 1, 0, -90);
        double[] headings3 = new double[4];
        double[] speeds3 = new double[4];
        for (int i = 0; i < 4; i++) {
            headings3[i] = arr3[i].angle;
            speeds3[i] = arr3[i].speed;
        }
        assertArrayEquals(new double[] {180, 180, 180, 180}, headings3);
        assertArrayEquals(new double[] {1, 1, 1, 1}, speeds3);
    }
}