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

    public SwerveDriveCoralScoreAlignWithClearance(Supplier<CoralBranch> branch, int level, boolean isScoringFrontSide, SuperStructureState correspondingSuperStructureState) {
        this.correspondingSuperStructureState = correspondingSuperStructureState;

        addCommands(
            new WaitUntilCommand(this::isClear)
                .deadlineFor(new SwerveDrivePIDToBranchClear(branch::get, isScoringFrontSide))
                .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftPeg() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR)),
            new SwerveDrivePIDToBranchScore(branch::get, level, isScoringFrontSide)
                .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftPeg() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR))
        );
    } 

    public SwerveDriveCoralScoreAlignWithClearance(CoralBranch branch, int level, boolean isFrontFacingReef, SuperStructureState correspondingSuperStructureState) {
        this(() -> branch, level, isFrontFacingReef, correspondingSuperStructureState);
    }

    private boolean isClear() {
        return SuperStructure.getInstance().getState() == correspondingSuperStructureState
            && SuperStructure.getInstance().atTarget();
    }
}
