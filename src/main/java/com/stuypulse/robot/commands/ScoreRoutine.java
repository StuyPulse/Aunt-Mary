package com.stuypulse.robot.commands;

import java.util.function.Supplier;

import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterShootForwards;
import com.stuypulse.robot.commands.shooter.ShooterWaitUntilHasCoral;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.robot.commands.arm.ArmSetState;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.elevator.ElevatorToHeight;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
import com.stuypulse.robot.commands.leds.LEDApplyPattern;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ScoreRoutine extends SequentialCommandGroup {
    private final Arm arm;
    private final Elevator elevator;
    private final Shooter shooter;

    public ScoreRoutine(int level, boolean isFrontFacingReef, ArmState correspondingArmState, ElevatorState correspondingElevatorState) {
        this(level, isFrontFacingReef, ReefUtil::getClosestCoralBranch, correspondingArmState, correspondingElevatorState);
    }

    public ScoreRoutine(int level, boolean isFrontFacingReef, CoralBranch targetBranch, ArmState correspondingArmState, ElevatorState correspondingElevatorState) {
        this(level, isFrontFacingReef, () -> targetBranch, correspondingArmState, correspondingElevatorState);
    }

    public ScoreRoutine(int level, boolean isFrontFacingReef, Supplier<CoralBranch> targetBranch, ArmState correspondingArmState, ElevatorState correspondingElevatorState) {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        shooter = Shooter.getInstance();

        addCommands(
            new SwerveDriveCoralScoreAlignWithClearance(targetBranch, level, isFrontFacingReef, correspondingElevatorState, correspondingArmState)
                .alongWith(
                    new WaitUntilCommand(Clearances::isArmClearFromReef)
                        .alongWith(new ShooterWaitUntilHasCoral())
                        .andThen(new ArmSetState(correspondingArmState).alongWith(new ElevatorToHeight(correspondingElevatorState)))
                        .onlyIf(() -> arm.getState() != correspondingArmState || elevator.getState() != correspondingElevatorState)
                        .andThen(new ArmWaitUntilAtTarget().alongWith(new ElevatorWaitUntilAtTargetHeight()))
                ),
            new ArmWaitUntilAtTarget().alongWith(new ElevatorWaitUntilAtTargetHeight()), // Re-check positioning
            new ConditionalCommand(
                new ShooterShootBackwards(), 
                new ShooterShootForwards(), 
                () -> shooter.shouldShootBackwards()),
            new LEDApplyPattern(Settings.LED.SCORE_COLOR)
        );
    }

    public ScoreRoutine(Gamepad driver, int level, boolean isFrontFacingReef, ArmState correspondingArmState, ElevatorState correspondingElevatorState) {
        this(level, isFrontFacingReef, getCoralBranchSupplierWithDriverInput(driver), correspondingArmState, correspondingElevatorState);
    }

    private static Supplier<CoralBranch> getCoralBranchSupplierWithDriverInput(Gamepad driver) {
        return () -> {
            if (driver.getLeftX() > Settings.Driver.BRANCH_OVERRIDE_DEADBAND) {
                return ReefUtil.getClosestReefFace().getRightBranchFieldRelative();
            }
            else if (driver.getLeftX() < -Settings.Driver.BRANCH_OVERRIDE_DEADBAND) {
                return ReefUtil.getClosestReefFace().getLeftBranchFieldRelative();
            }
            else {
                return ReefUtil.getClosestCoralBranch();
            }
        };
    }
}