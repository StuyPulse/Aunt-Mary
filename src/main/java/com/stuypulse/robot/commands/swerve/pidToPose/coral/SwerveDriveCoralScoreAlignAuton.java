
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

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;

public class SwerveDriveCoralScoreAlignAuton extends SequentialCommandGroup {

    public SwerveDriveCoralScoreAlignAuton(Supplier<CoralBranch> branch, int level, boolean isScoringFrontSide, ElevatorState correspondingElevatorState, ArmState correspondingArmState, double timeout) {

        Timer timer = new Timer();

        addCommands(
            new InstantCommand(timer::reset),
            new InstantCommand(timer::start),
            new SwerveDrivePIDToBranchScore(branch::get, level, isScoringFrontSide)
                .withoutMotionProfile()
                .withTimeout(timeout)
                .deadlineFor(new LEDApplyPattern(Settings.LED.AUTON_TO_REEF_COLOR)),
            new InstantCommand(() -> {
                if (timer.hasElapsed(timeout)) {
                    CommandScheduler.getInstance().cancel(new LEDApplyPattern(Settings.LED.AUTON_TO_REEF_COLOR));
                    new LEDApplyPattern(Settings.LED.AUTON_TIMEOUT_COLOR).schedule();
                }
            })
        );
    }

    public SwerveDriveCoralScoreAlignAuton(CoralBranch branch, int level, boolean isFrontFacingReef, ElevatorState correspondingElevatorState, ArmState correspondingArmState, double timeout) {
        this(() -> branch, level, isFrontFacingReef, correspondingElevatorState, correspondingArmState, timeout);
    }
}
