package com.stuypulse.robot.util;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ReefUtil.ReefFace;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Pose2d;

public class TargetReefFaceManager {
    private static ReefFace startingTargetReefFace;
    private static Offset offset; // Field relative

    enum Offset {
        LEFT,
        CENTER,
        RIGHT
    }

    static {
        // Default values
        startingTargetReefFace = ReefFace.AB;
        offset = Offset.CENTER;
    }

    public static void reset(ReefFace startingTargetReefFace) {
        TargetReefFaceManager.startingTargetReefFace = startingTargetReefFace;
        offset = Offset.CENTER;
    }

    public static void offsetLeft() {
        int index = offset.ordinal() - 1;
        index = index < 0 ? 0 : index;
        offset = Offset.values()[index];
    }

    public static void offsetRight() {
        int index = offset.ordinal() + 1;
        index = index > 2 ? 2 : index;
        offset = Offset.values()[index];
    }

    public static ReefFace getTargetReefFace() {
        int ccwRotation = offset.ordinal() - 1;
        if (!startingTargetReefFace.isOnDriverStationSide()) {
            ccwRotation *= -1;
        }
        return startingTargetReefFace.rotateCCW(ccwRotation);
    }

    public static Supplier<Pose2d> getPoseSupplierWithDriverInput(Gamepad driver, boolean isScoringFrontSide) {
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
