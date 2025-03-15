package com.stuypulse.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.shooter.ShooterWaitUntilHasCoral;
import com.stuypulse.robot.commands.superStructure.SuperStructureSetState;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ScoreRoutine extends SequentialCommandGroup {
    private final SuperStructure superStructure;
    private final Shooter shooter;

    public ScoreRoutine(int level, boolean isFrontFacingReef) {
        this(level, isFrontFacingReef, ReefUtil::getClosestCoralBranch, () -> false, 0);
    }

    public ScoreRoutine(int level, boolean isFrontFacingReef, CoralBranch targetBranch) {
        this(level, isFrontFacingReef, targetBranch, 0);
    }

    public ScoreRoutine(int level, boolean isFrontFacingReef, CoralBranch targetBranch, double shootTime) {
        this(level, isFrontFacingReef, () -> targetBranch, () -> false, shootTime);
    }

    public ScoreRoutine(int level, boolean isFrontFacingReef, CoralBranch targetBranch, Supplier<Boolean> shouldSkipClearance, double shootTime) {
        this(level, isFrontFacingReef, () -> targetBranch, shouldSkipClearance, shootTime);
    }

    public ScoreRoutine(int level, boolean isFrontFacingReef, Supplier<CoralBranch> targetBranch, Supplier<Boolean> shouldSkipClearance, double shootTime) {
        superStructure = SuperStructure.getInstance();
        shooter = Shooter.getInstance();

        SuperStructureState correspondingSuperStructureState = SuperStructure.getCorrespondingCoralScoreState(level, isFrontFacingReef);

        addCommands(
            new SwerveDriveCoralScoreAlignWithClearance(targetBranch, level, isFrontFacingReef, correspondingSuperStructureState, shouldSkipClearance)
                .alongWith(
                    new WaitUntilCommand(Clearances::isArmClearFromReef)
                        .until(shouldSkipClearance::get)
                        .alongWith(new ShooterWaitUntilHasCoral())
                        .andThen(new SuperStructureSetState(correspondingSuperStructureState))
                        .onlyIf(() -> superStructure.getState() != correspondingSuperStructureState)
                        .andThen(new SuperStructureWaitUntilAtTarget())
                )
                .andThen(new SuperStructureWaitUntilAtTarget()) // Re-check positioning
                .andThen(Shooter.getCorrespondingShootCommand(level, isFrontFacingReef))
                .andThen(new WaitCommand(shootTime)
                    .andThen(new ShooterStop()))
        );
    }

    public ScoreRoutine(Gamepad driver, int level, boolean isFrontFacingReef) {
        this(level, isFrontFacingReef, getCoralBranchSupplierWithDriverInput(driver), () -> false, 0);
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
