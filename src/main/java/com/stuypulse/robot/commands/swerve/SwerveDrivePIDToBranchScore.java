package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field.CoralBranch;

public class SwerveDrivePIDToBranchScore extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToBranchScore(int level, boolean isScoringFrontSide, CoralBranch nearestBranch) {
        super(() -> nearestBranch.getScorePose(level, isScoringFrontSide));
    }
}
