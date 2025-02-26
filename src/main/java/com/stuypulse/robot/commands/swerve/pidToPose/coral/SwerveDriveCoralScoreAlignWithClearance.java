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
    public SwerveDriveCoralScoreAlignWithClearance(Supplier<CoralBranch> branch, int level, boolean isFrontFacingReef, ElevatorState correspondingElevatorState, ArmState correspondingArmState) {
        addCommands(
            new WaitUntilCommand(() -> Elevator.getInstance().getState() == correspondingElevatorState && Elevator.getInstance().atTargetHeight() 
                && Arm.getInstance().getState() == correspondingArmState && Arm.getInstance().atTargetAngle())
                .deadlineFor(new SwerveDrivePIDToBranchClear(branch::get, isFrontFacingReef))
                .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftPeg() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR)),
            new SwerveDrivePIDToBranchScore(branch::get, level, isFrontFacingReef)
                .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftPeg() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR))
        );
    } 

    public SwerveDriveCoralScoreAlignWithClearance(CoralBranch branch, int level, boolean isFrontFacingReef, ElevatorState correspondingElevatorState, ArmState correspondingArmState) {
        this(() -> branch, level, isFrontFacingReef, correspondingElevatorState, correspondingArmState);
    }

    public SwerveDriveCoralScoreAlignWithClearance(int level, boolean isFrontFacingReef, ElevatorState correspondingElevatorState, ArmState correspondingArmState) {
        this(ReefUtil::getClosestCoralBranch, level, isFrontFacingReef, correspondingElevatorState, correspondingArmState);
    }
}
