package com.stuypulse.robot.commands;

import com.stuypulse.robot.commands.superStructure.SuperStructureFeed;
import com.stuypulse.robot.commands.swerve.pathFindToPose.SwerveDrivePathFindToPose;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.TargetReefFaceManager;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ScoreRoutineWithReefFaceSwitching extends ConditionalCommand{
    public ScoreRoutineWithReefFaceSwitching(Gamepad driver, int level) {
        super(new ConditionalCommand(
                new SwerveDrivePathFindToPose(TargetReefFaceManager.getClearancePoseSupplierWithDriverInput(driver, true))
                    .deadlineFor(new ConditionalCommand(
                        SuperStructure.getCorrespondingCoralScoreStateCommand(level, true), 
                        new SuperStructureFeed(), 
                        () -> TargetReefFaceManager.getTargetReefFace() == ReefUtil.getClosestReefFace())
                        .onlyIf(() -> Clearances.isArmClearFromReef())
                        .repeatedly())
                    .until(() -> driver.getLeftBumper().getAsBoolean() || driver.getRightBumper().getAsBoolean()), 
                new ScoreRoutine(driver, level, true).alongWith(new WaitUntilCommand(() -> false))
                    .until(() -> ReefUtil.getClosestReefFace() != TargetReefFaceManager.getTargetReefFace()), 
                () -> ReefUtil.getClosestReefFace() != TargetReefFaceManager.getTargetReefFace()).repeatedly(),
            new ConditionalCommand(
                new SwerveDrivePathFindToPose(TargetReefFaceManager.getClearancePoseSupplierWithDriverInput(driver, false))
                    .deadlineFor(new ConditionalCommand(
                        SuperStructure.getCorrespondingCoralScoreStateCommand(level, true), 
                        new SuperStructureFeed(), 
                        () -> TargetReefFaceManager.getTargetReefFace() == ReefUtil.getClosestReefFace())
                        .onlyIf(() -> Clearances.isArmClearFromReef())
                        .repeatedly())
                    .until(() -> driver.getLeftBumper().getAsBoolean() || driver.getRightBumper().getAsBoolean()), 
                new ScoreRoutine(driver, level, false).alongWith(new WaitUntilCommand(() -> false))
                    .until(() -> ReefUtil.getClosestReefFace() != TargetReefFaceManager.getTargetReefFace()), 
                () -> ReefUtil.getClosestReefFace() != TargetReefFaceManager.getTargetReefFace()).repeatedly(),
            () -> CommandSwerveDrivetrain.getInstance().isFrontFacingAllianceReef());
    }
}
