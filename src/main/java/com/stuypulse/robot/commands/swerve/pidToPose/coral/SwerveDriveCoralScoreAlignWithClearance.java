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
                .deadlineFor(new SwerveDrivePIDToBranchClear(branch, isScoringFrontSide))
                .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftBranchRobotRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR)),
            new SwerveDrivePIDToBranchScore(branch, level, isScoringFrontSide)
                .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftBranchRobotRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR))
        );
    }

    public SwerveDriveCoralScoreAlignWithClearance(Supplier<CoralBranch> branch, int level, boolean isScoringFrontSide, SuperStructureState correspondingSuperStructureState, double maxVel, double maxAccel) {
        this.correspondingSuperStructureState = correspondingSuperStructureState;

        addCommands(
            new WaitUntilCommand(this::isClear)
                .deadlineFor(new SwerveDrivePIDToBranchClear(branch, isScoringFrontSide)
                    .withTranslationalConstraints(maxVel, maxAccel))
                .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftBranchRobotRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR)),
            new SwerveDrivePIDToBranchScore(branch, level, isScoringFrontSide)
                .withTranslationalConstraints(maxVel, maxAccel)
                .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftBranchRobotRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR))
        );
    } 

    public SwerveDriveCoralScoreAlignWithClearance(CoralBranch branch, int level, boolean isFrontFacingReef, SuperStructureState correspondingSuperStructureState) {
        this(() -> branch, level, isFrontFacingReef, correspondingSuperStructureState);
    }

    public SwerveDriveCoralScoreAlignWithClearance(CoralBranch branch, int level, boolean isFrontFacingReef, SuperStructureState correspondingSuperStructureState, double maxVel, double maxAccel) {
        this(() -> branch, level, isFrontFacingReef, correspondingSuperStructureState, maxVel, maxAccel);
    }

    private boolean isClear() {
        return SuperStructure.getInstance().getState() == correspondingSuperStructureState
            && SuperStructure.getInstance().canSkipClearance();
    }
}
