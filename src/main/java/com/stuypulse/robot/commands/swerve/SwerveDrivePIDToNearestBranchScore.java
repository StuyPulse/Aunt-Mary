package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;

public class SwerveDrivePIDToNearestBranchScore extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToNearestBranchScore(int level, boolean isScoringFrontSide) {
        super(() -> Field.getClosestBranch().getScorePose(level, isScoringFrontSide));
    }
}
