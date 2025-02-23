package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Field.CoralBranch;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveDriveCoralScoreAlignWithClearance extends SequentialCommandGroup {    
    public SwerveDriveCoralScoreAlignWithClearance(int level, boolean isFrontFacingReef, ElevatorState correspondingElevatorState, ArmState correspondingArmState) {
        Supplier<CoralBranch> nearestBranch = () -> Field.getClosestBranch();

        addCommands(
            new SwerveDrivePIDToPose(() -> nearestBranch.get().getReadyPose(isFrontFacingReef))
                .until(() -> Elevator.getInstance().getState() == correspondingElevatorState && Elevator.getInstance().atTargetHeight() 
                        && Arm.getInstance().getState() == correspondingArmState && Arm.getInstance().atTargetAngle())
                .alongWith(new LEDApplyPattern(nearestBranch.get().isLeftPeg() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR)),
            new SwerveDrivePIDToPose(() -> nearestBranch.get().getScorePose(level, isFrontFacingReef))
        );
    }
}
