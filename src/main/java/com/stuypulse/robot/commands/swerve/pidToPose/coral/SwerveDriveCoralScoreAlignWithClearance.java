package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import java.util.function.Supplier;

import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SwerveDriveCoralScoreAlignWithClearance extends SequentialCommandGroup {
    private final ElevatorState correspondingElevatorState;
    private final ArmState correspondingArmState;

    public SwerveDriveCoralScoreAlignWithClearance(Supplier<CoralBranch> branch, int level, boolean isScoringFrontSide, ElevatorState correspondingElevatorState, ArmState correspondingArmState) {
        this.correspondingElevatorState = correspondingElevatorState;
        this.correspondingArmState = correspondingArmState;

        addCommands(
            new WaitUntilCommand(this::isClear)
                .deadlineFor(new SwerveDrivePIDToBranchClear(branch::get, isScoringFrontSide))
                .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftPeg() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR)),
            new SwerveDrivePIDToBranchScore(branch::get, level, isScoringFrontSide)
                .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftPeg() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR))
        );
    } 

    public SwerveDriveCoralScoreAlignWithClearance(CoralBranch branch, int level, boolean isFrontFacingReef, ElevatorState correspondingElevatorState, ArmState correspondingArmState) {
        this(() -> branch, level, isFrontFacingReef, correspondingElevatorState, correspondingArmState);
    }

    public SwerveDriveCoralScoreAlignWithClearance(int level, boolean isFrontFacingReef, ElevatorState correspondingElevatorState, ArmState correspondingArmState) {
        this(ReefUtil::getClosestCoralBranch, level, isFrontFacingReef, correspondingElevatorState, correspondingArmState);
    }

    private boolean isClear() {
        return (Elevator.getInstance().getState() == correspondingElevatorState && Elevator.getInstance().atTargetHeight() 
                && Arm.getInstance().getState() == correspondingArmState && Arm.getInstance().atTargetAngle());
                // || (correspondingArmState.getTargetAngle().getDegrees() < 90 && Arm.getInstance().getCurrentAngle().getDegrees() > Settings.Clearances.MIN_ARM_ANGLE_TO_IGNORE_CLEARANCE_FRONT.getDegrees())
                // || (correspondingArmState.getTargetAngle().getDegrees() > 90 && Arm.getInstance().getCurrentAngle().getDegrees() > Settings.Clearances.MIN_ARM_ANGLE_TO_IGNORE_CLEARANCE_BACK.getDegrees());
    }
}
