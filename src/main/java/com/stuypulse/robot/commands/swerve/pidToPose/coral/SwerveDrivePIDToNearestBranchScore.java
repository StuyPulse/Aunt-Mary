package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ReefUtil;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class SwerveDrivePIDToNearestBranchScore extends ParallelDeadlineGroup{ 
    public SwerveDrivePIDToNearestBranchScore(int level, boolean isScoringFrontSide) {
        // super(() -> ReefUtil.getClosestCoralBranch(), level, isScoringFrontSide);
        super(
            new SwerveDrivePIDToBranchScore(() -> ReefUtil.getClosestCoralBranch(), level, isScoringFrontSide),
            new LEDApplyPattern(() -> ReefUtil.getClosestCoralBranch().isLeftPeg() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR)
        );
    }
}
