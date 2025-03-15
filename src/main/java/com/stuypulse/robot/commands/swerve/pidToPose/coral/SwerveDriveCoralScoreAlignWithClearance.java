package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import java.util.function.Supplier;

import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SwerveDriveCoralScoreAlignWithClearance extends SequentialCommandGroup {
    private final SuperStructureState correspondingSuperStructureState;
    private Supplier<Boolean> shouldSkipClearance;

    public SwerveDriveCoralScoreAlignWithClearance(Supplier<CoralBranch> branch, int level, boolean isScoringFrontSide, SuperStructureState correspondingSuperStructureState, Supplier<Boolean> shouldSkipClearance) {
        this.correspondingSuperStructureState = correspondingSuperStructureState;
        this.shouldSkipClearance = shouldSkipClearance;

        addCommands(
            new WaitUntilCommand(this::isClear)
                .until(shouldSkipClearance::get)
                .deadlineFor(new SwerveDrivePIDToBranchClear(branch, isScoringFrontSide))
                .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftBranchRobotRelative() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR)),
            new SwerveDrivePIDToBranchScore(branch, level, isScoringFrontSide)
                .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftBranchRobotRelative() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR))
        );
    } 

    public SwerveDriveCoralScoreAlignWithClearance(Supplier<CoralBranch> branch, int level, boolean isScoringFrontSide, SuperStructureState correspondingSuperStructureState, boolean shouldSkipClearance) {
        this(branch, level, isScoringFrontSide, correspondingSuperStructureState, () -> shouldSkipClearance);
    }

    public SwerveDriveCoralScoreAlignWithClearance(CoralBranch branch, int level, boolean isFrontFacingReef, SuperStructureState correspondingSuperStructureState) {
        this(() -> branch, level, isFrontFacingReef, correspondingSuperStructureState, false);
    }

    private boolean isClear() {
        return SuperStructure.getInstance().getState() == correspondingSuperStructureState
            && SuperStructure.getInstance().atTarget();
    }
}
