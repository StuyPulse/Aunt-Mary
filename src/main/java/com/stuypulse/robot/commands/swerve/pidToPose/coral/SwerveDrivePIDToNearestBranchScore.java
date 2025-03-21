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
            new LEDApplyPattern(() -> ReefUtil.getClosestCoralBranch().isLeftPegFieldRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR)
        );
    }
}
