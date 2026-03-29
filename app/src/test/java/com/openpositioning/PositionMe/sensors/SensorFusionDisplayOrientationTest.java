package com.openpositioning.PositionMe.sensors;

import android.view.Surface;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class SensorFusionDisplayOrientationTest {

    private static final float HALF_SQRT_TWO = (float) (Math.sqrt(2.0) * 0.5);

    @Test
    public void extractDisplayHeadingRadKeepsNorthWhenUprightDevicePitches() {
        float baseHeading = SensorFusion.extractDisplayHeadingRad(uprightPortraitFacingNorth(), Surface.ROTATION_0);
        float pitchedHeading = SensorFusion.extractDisplayHeadingRad(
                uprightPortraitFacingNorthWithForwardPitch(),
                Surface.ROTATION_0
        );

        assertEquals(0f, baseHeading, 1e-6f);
        assertEquals(baseHeading, pitchedHeading, 1e-6f);
    }

    @Test
    public void extractDisplayHeadingRadKeepsNorthWhenUprightDeviceRolls() {
        float baseHeading = SensorFusion.extractDisplayHeadingRad(uprightPortraitFacingNorth(), Surface.ROTATION_0);
        float rolledHeading = SensorFusion.extractDisplayHeadingRad(
                uprightPortraitFacingNorthWithSideRoll(),
                Surface.ROTATION_0
        );

        assertEquals(0f, baseHeading, 1e-6f);
        assertEquals(baseHeading, rolledHeading, 1e-6f);
    }

    @Test
    public void extractDisplayHeadingRadStillRespondsToYawChanges() {
        float northHeading = SensorFusion.extractDisplayHeadingRad(uprightPortraitFacingNorth(), Surface.ROTATION_0);
        float eastHeading = SensorFusion.extractDisplayHeadingRad(uprightPortraitFacingEast(), Surface.ROTATION_0);

        assertEquals(0f, northHeading, 1e-6f);
        assertEquals((float) (Math.PI / 2.0), eastHeading, 1e-6f);
    }

    @Test
    public void extractDisplayHeadingRadFallsBackToScreenTopWhenPhoneIsFlat() {
        float heading = SensorFusion.extractDisplayHeadingRad(flatPortraitFacingNorth(), Surface.ROTATION_0);

        assertEquals(0f, heading, 1e-6f);
    }

    private static float[] uprightPortraitFacingNorth() {
        return rotationMatrix(
                1f, 0f, 0f,
                0f, 0f, 1f,
                0f, -1f, 0f
        );
    }

    private static float[] uprightPortraitFacingNorthWithForwardPitch() {
        return rotationMatrix(
                1f, 0f, 0f,
                0f, HALF_SQRT_TWO, HALF_SQRT_TWO,
                0f, -HALF_SQRT_TWO, HALF_SQRT_TWO
        );
    }

    private static float[] uprightPortraitFacingNorthWithSideRoll() {
        return rotationMatrix(
                HALF_SQRT_TWO, 0f, -HALF_SQRT_TWO,
                HALF_SQRT_TWO, 0f, HALF_SQRT_TWO,
                0f, -1f, 0f
        );
    }

    private static float[] uprightPortraitFacingEast() {
        return rotationMatrix(
                0f, -1f, 0f,
                0f, 0f, 1f,
                -1f, 0f, 0f
        );
    }

    private static float[] flatPortraitFacingNorth() {
        return rotationMatrix(
                1f, 0f, 0f,
                0f, 1f, 0f,
                0f, 0f, 1f
        );
    }

    private static float[] rotationMatrix(
            float xEast,
            float xNorth,
            float xUp,
            float yEast,
            float yNorth,
            float yUp,
            float zEast,
            float zNorth,
            float zUp
    ) {
        return new float[]{
                xEast, yEast, zEast,
                xNorth, yNorth, zNorth,
                xUp, yUp, zUp
        };
    }
}
