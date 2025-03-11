package com.stuypulse.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterShootForwards;
import com.stuypulse.robot.commands.shooter.ShooterWaitUntilHasCoral;
import com.stuypulse.robot.commands.superStructure.SuperStructureSetState;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ScoreRoutine extends SequentialCommandGroup {
    private final SuperStructure superStructure;
    private final Shooter shooter;

    public ScoreRoutine(int level, boolean isFrontFacingReef) {
        this(level, isFrontFacingReef, Optional.empty());
    }

    public ScoreRoutine(int level, boolean isFrontFacingReef, Optional<CoralBranch> branch) {
        superStructure = SuperStructure.getInstance();
        shooter = Shooter.getInstance();

        SuperStructureState correspondingSuperStructureState = SuperStructure.getCorrespondingCoralScoreState(level, isFrontFacingReef);
        Supplier<CoralBranch> targetBranch = branch.isPresent() ? () -> branch.get() : () -> ReefUtil.getClosestCoralBranch();

        addCommands(
            new SwerveDriveCoralScoreAlignWithClearance(targetBranch, level, isFrontFacingReef, correspondingSuperStructureState)
                .alongWith(
                    new WaitUntilCommand(Clearances::isArmClearFromReef)
                        .alongWith(new ShooterWaitUntilHasCoral())
                        .andThen(new SuperStructureSetState(correspondingSuperStructureState))
                        .onlyIf(() -> superStructure.getState() != correspondingSuperStructureState)
                        .andThen(new SuperStructureWaitUntilAtTarget())
                )
                .andThen(new SuperStructureWaitUntilAtTarget()) // Re-check positioning
                .andThen(shooter.shouldShootBackwards() ? new ShooterShootBackwards() : new ShooterShootForwards())
        );
    }
}
