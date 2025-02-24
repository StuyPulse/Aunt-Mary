package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.stuypulse.robot.util.ReefUtil.CoralBranch;

public class SwerveDrivePIDToBranchScore extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToBranchScore(Supplier<CoralBranch> branch, int level, boolean isScoringFrontSide) {
        super(() -> branch.get().getScorePose(level, isScoringFrontSide));
    }

    public SwerveDrivePIDToBranchScore(CoralBranch branch, int level, boolean isScoringFrontSide) {
        this(() -> branch, level, isScoringFrontSide);
    }
}
