package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.stuypulse.robot.util.ReefUtil.CoralBranch;

public class SwerveDrivePIDToBranchReady extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToBranchReady(Supplier<CoralBranch> branch, int level, boolean isScoringFrontSide) {
        super(() -> branch.get().getReadyPose(level, isScoringFrontSide));
    }

    public SwerveDrivePIDToBranchReady(CoralBranch branch, int level, boolean isScoringFrontSide) {
        this(() -> branch, level, isScoringFrontSide);
    }
}
