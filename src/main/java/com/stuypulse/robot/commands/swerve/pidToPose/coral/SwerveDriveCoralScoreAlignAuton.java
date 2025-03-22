
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve.pidToPose.coral;

import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.Supplier;

public class SwerveDriveCoralScoreAlignAuton extends SequentialCommandGroup {

    private final ElevatorState correspondingElevatorState;
    private final ArmState correspondingArmState;

    public SwerveDriveCoralScoreAlignAuton(Supplier<CoralBranch> branch, int level, boolean isScoringFrontSide, ElevatorState correspondingElevatorState, ArmState correspondingArmState, double timeout) {
        this.correspondingElevatorState = correspondingElevatorState;
        this.correspondingArmState = correspondingArmState;

        addCommands(
            new SwerveDrivePIDToBranchScore(branch::get, level, isScoringFrontSide)
                .withTranslationalConstraints(Settings.Swerve.Alignment.Constraints.MAX_VELOCITY_AUTON, Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION_AUTON)
                .withTimeout(timeout)
                .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftBranchRobotRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR))
        );
    } 

    public SwerveDriveCoralScoreAlignAuton(CoralBranch branch, int level, boolean isFrontFacingReef, ElevatorState correspondingElevatorState, ArmState correspondingArmState, double timeout) {
        this(() -> branch, level, isFrontFacingReef, correspondingElevatorState, correspondingArmState, timeout);
    }

}
