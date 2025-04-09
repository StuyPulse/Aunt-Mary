package com.stuypulse.robot.util;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;
import com.stuypulse.robot.util.ReefUtil.ReefFace;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Pose2d;

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

    public static Supplier<Pose2d> getPoseSupplierWithDriverInput(Gamepad driver, boolean isScoringFrontSide) {
        return () -> {
            if (driver.getLeftX() > Settings.Driver.BRANCH_OVERRIDE_DEADBAND) {
                return targetReefFace.getRightBranchFieldRelative().getClearancePose(isScoringFrontSide);
            }
            else if (driver.getLeftX() < -Settings.Driver.BRANCH_OVERRIDE_DEADBAND) {
                return targetReefFace.getLeftBranchFieldRelative().getClearancePose(isScoringFrontSide);
            }
            else {
                return targetReefFace.getClosestCoralBranch().getClearancePose(isScoringFrontSide);
            }
        };
    }
}
