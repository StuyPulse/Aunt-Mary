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

public class SwerveDriveCoralScoreAlignWithClearanceAuton extends SequentialCommandGroup {

    private final ElevatorState correspondingElevatorState;
    private final ArmState correspondingArmState;

    public SwerveDriveCoralScoreAlignWithClearanceAuton(Supplier<CoralBranch> branch, int level, boolean isScoringFrontSide, ElevatorState correspondingElevatorState, ArmState correspondingArmState, double timeout) {
        this.correspondingElevatorState = correspondingElevatorState;
        this.correspondingArmState = correspondingArmState;

        addCommands(
            // new SwerveDrivePIDToBranchClear(branch::get, isScoringFrontSide)
            //     .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftPeg() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR)),
            new SwerveDrivePIDToBranchScore(branch::get, level, isScoringFrontSide)
                .withTranslationalConstraints(Settings.Swerve.Alignment.Constraints.MAX_VELOCITY_AUTON.get(), Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION_AUTON.get())
                .withTimeout(timeout)
                .deadlineFor(new LEDApplyPattern(() -> branch.get().isLeftPeg() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR))
        );
    } 

    public SwerveDriveCoralScoreAlignWithClearanceAuton(CoralBranch branch, int level, boolean isFrontFacingReef, ElevatorState correspondingElevatorState, ArmState correspondingArmState, double timeout) {
        this(() -> branch, level, isFrontFacingReef, correspondingElevatorState, correspondingArmState, timeout);
    }

}
