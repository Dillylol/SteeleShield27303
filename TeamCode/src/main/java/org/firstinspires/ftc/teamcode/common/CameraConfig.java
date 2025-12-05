package org.firstinspires.ftc.teamcode.common;

/**
 * Centralized camera + AprilTag configuration for the 2024-2025 DECODE field.
 * Values here are placeholders that can be updated after camera calibration.
 */
public final class CameraConfig {

    // Physical camera reference
    public static final String WEBCAM_NAME = "Webcam 1";

    // Tag IDs for the season
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int OBELISK_TAG_ID_21 = 21;
    public static final int OBELISK_TAG_ID_22 = 22;
    public static final int OBELISK_TAG_ID_23 = 23;
    public static final int RED_GOAL_TAG_ID = 24;

    // Camera intrinsics (update once the camera is calibrated)
    public static final double FX = 765.0;
    public static final double FY = 765.0;
    public static final double CX = 320.0;
    public static final double CY = 240.0;

    // FTC tag size (2 inches = 50.8mm)
    public static final double TAG_SIZE_METERS = 0.165;

    public static final String CLASS_BLUE_GOAL = "blue_goal";
    public static final String CLASS_RED_GOAL = "red_goal";
    public static final String CLASS_OBELISK = "obelisk";
    public static final String CLASS_UNKNOWN = "unknown";

    private CameraConfig() {
    }

    public static String classify(int id) {
        if (id == BLUE_GOAL_TAG_ID) {
            return CLASS_BLUE_GOAL;
        }
        if (id == RED_GOAL_TAG_ID) {
            return CLASS_RED_GOAL;
        }
        if (id == OBELISK_TAG_ID_21 || id == OBELISK_TAG_ID_22 || id == OBELISK_TAG_ID_23) {
            return CLASS_OBELISK;
        }
        return CLASS_UNKNOWN;
    }

    public static boolean isGoalTag(int id) {
        return id == BLUE_GOAL_TAG_ID || id == RED_GOAL_TAG_ID;
    }
}

