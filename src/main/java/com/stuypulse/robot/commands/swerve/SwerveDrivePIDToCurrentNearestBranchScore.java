package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;

public class SwerveDrivePIDToCurrentNearestBranchScore extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToCurrentNearestBranchScore(int level, boolean isScoringFrontSide) {
        super(() -> Field.getClosestBranch().getScorePose(level, isScoringFrontSide));
    }
}
