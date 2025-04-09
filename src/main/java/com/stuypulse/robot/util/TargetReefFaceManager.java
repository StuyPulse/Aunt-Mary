package com.stuypulse.robot.util;

import com.stuypulse.robot.util.ReefUtil.ReefFace;

public class TargetReefFaceManager {
    private static ReefFace targetReefFace;

    static {
        targetReefFace = ReefFace.AB; // Default value
    }

    public static void setTargetReefFace(ReefFace reefFace) {
        targetReefFace = reefFace;
    }

    public static void rotateTargetReefFaceCCWBy(int rotationCCW) {
        targetReefFace = targetReefFace.rotateCCW(rotationCCW);
    }

    public static ReefFace getTargetReefFace() {
        return targetReefFace;
    }
}
