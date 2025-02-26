package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import com.stuypulse.robot.util.ReefUtil;

public class SwerveDrivePIDToNearestBranchScore extends SwerveDrivePIDToBranchScore{
    public SwerveDrivePIDToNearestBranchScore(int level, boolean isScoringFrontSide) {
        super(() -> ReefUtil.getClosestCoralBranch(), level, isScoringFrontSide);
    }
}
