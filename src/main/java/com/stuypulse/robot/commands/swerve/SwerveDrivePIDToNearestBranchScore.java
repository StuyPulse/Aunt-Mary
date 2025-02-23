package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.util.ReefUtil;

public class SwerveDrivePIDToNearestBranchScore extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToNearestBranchScore(int level, boolean isScoringFrontSide) {
        super(() -> ReefUtil.getClosestBranch().getScorePose(level, isScoringFrontSide));
    }
}
