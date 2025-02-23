package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field.CoralBranch;

public class SwerveDrivePIDToNearestBranchScore extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToNearestBranchScore(int level, boolean isScoringFrontSide, CoralBranch nearestBranch) {
        super(() -> nearestBranch.getScorePose(level, isScoringFrontSide));
    }
}
