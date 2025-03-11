package com.stuypulse.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterShootForwards;
import com.stuypulse.robot.commands.shooter.ShooterWaitUntilHasCoral;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ScoreRoutine extends SequentialCommandGroup {
    private final Arm arm;
    private final Elevator elevator;
    private final Shooter shooter;

    public ScoreRoutine(int level, boolean isFrontFacingReef) {
        this(level, isFrontFacingReef, Optional.empty());
    }

    public ScoreRoutine(int level, boolean isFrontFacingReef, Optional<CoralBranch> branch) {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        shooter = Shooter.getInstance();

        ElevatorState elevatorState = Elevator.getState(level, isFrontFacingReef);
        ArmState armState = Arm.getState(level, isFrontFacingReef);
        Supplier<CoralBranch> targetBranch = branch.isPresent() ? () -> branch.get() : () -> ReefUtil.getClosestCoralBranch();

        addCommands(
            new SwerveDriveCoralScoreAlignWithClearance(targetBranch, level, isFrontFacingReef, elevatorState, armState)
                .alongWith(
                    new WaitUntilCommand(Clearances::isArmClearFromReef)
                        .alongWith(new ShooterWaitUntilHasCoral())
                        .andThen(RobotContainer.getElevatorArmCommands(level, isFrontFacingReef))
                        .onlyIf(() -> elevator.getState() != elevatorState || arm.getState() != armState)
                        .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
                )
                .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget())) // Re-check positioning
                .andThen(shooter.shouldShootBackwards() ? new ShooterShootBackwards() : new ShooterShootForwards())
        );
    }
}
