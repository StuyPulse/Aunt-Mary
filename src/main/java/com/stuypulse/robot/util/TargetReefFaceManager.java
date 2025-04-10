package com.stuypulse.robot.util;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ReefUtil.ReefFace;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Pose2d;

public class TargetReefFaceManager {
    private static ReefFace startingTargetReefFace;
    private static int offset;

    static {
        // Default values
        startingTargetReefFace = ReefFace.AB;
        offset = 0;
    }

    public static void reset(ReefFace startingTargetReefFace) {
        TargetReefFaceManager.startingTargetReefFace = startingTargetReefFace;
        offset = 0;
    }

    public static void offsetLeft() {
        offset--;
    }

    public static void offsetRight() {
        offset++;
    }

    public static ReefFace getTargetReefFace() {
        int ccwRotation = offset;
        if (!startingTargetReefFace.isOnDriverStationSide()) {
            ccwRotation *= -1;
        }
        return startingTargetReefFace.rotateCCW(ccwRotation);
    }

    public static Supplier<Pose2d> getClearancePoseSupplierWithDriverInput(Gamepad driver, boolean isScoringFrontSide) {
        return () -> {
            if (driver.getLeftX() > Settings.Driver.BRANCH_OVERRIDE_DEADBAND) {
                return getTargetReefFace().getRightBranchFieldRelative().getClearancePose(isScoringFrontSide);
            }
            else if (driver.getLeftX() < -Settings.Driver.BRANCH_OVERRIDE_DEADBAND) {
                return getTargetReefFace().getLeftBranchFieldRelative().getClearancePose(isScoringFrontSide);
            }
            else {
                return getTargetReefFace().getClosestCoralBranch().getClearancePose(isScoringFrontSide);
            }
        };
    }
}
