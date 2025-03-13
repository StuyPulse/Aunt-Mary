package com.stuypulse.robot.commands;

import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ScoreRoutine extends SequentialCommandGroup {
    private final SuperStructure superStructure;
    private final Shooter shooter;

    public ScoreRoutine(int level, boolean isFrontFacingReef) {
        this(level, isFrontFacingReef, ReefUtil::getClosestCoralBranch);
    }

    public ScoreRoutine(int level, boolean isFrontFacingReef, CoralBranch targetBranch) {
        this(level, isFrontFacingReef, () -> targetBranch);
    }

    public ScoreRoutine(int level, boolean isFrontFacingReef, Supplier<CoralBranch> targetBranch) {
        superStructure = SuperStructure.getInstance();
        shooter = Shooter.getInstance();

        SuperStructureState correspondingSuperStructureState = SuperStructure.getCorrespondingCoralScoreState(level, isFrontFacingReef);

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
                .andThen(Shooter.getCorrespondingShootCommand(level, isFrontFacingReef))
        );
    }

    public ScoreRoutine(Gamepad driver, int level, boolean isFrontFacingReef) {
        this(level, isFrontFacingReef, getCoralBranchSupplierWithDriverInput(driver));
    }

    private static Supplier<CoralBranch> getCoralBranchSupplierWithDriverInput(Gamepad driver) {
        return () -> {
            if (driver.getLeftX() > Settings.Driver.BRANCH_OVERRIDE_DEADBAND) {
                return ReefUtil.ReefFace.getClosestReefFace().getRightBranchFieldRelative();
            }
            else if (driver.getLeftX() < -Settings.Driver.BRANCH_OVERRIDE_DEADBAND) {
                return ReefUtil.ReefFace.getClosestReefFace().getLeftBranchFieldRelative();
            }
            else {
                return ReefUtil.getClosestCoralBranch();
            }
        };
    }
}
