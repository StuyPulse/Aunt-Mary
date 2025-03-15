package com.stuypulse.robot.commands.autons.HAlgae;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.shooter.ShooterAcquireAlgae;
import com.stuypulse.robot.commands.shooter.ShooterHoldAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootL4Front;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeL2Front;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureCatapultReady;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureCatapultShoot;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureWaitUntilCanCatapult;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Front;
import com.stuypulse.robot.commands.swerve.SwerveDriveWaitUntilAlignedToCatapultAllianceSide;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePIDToBarge;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePidToNearestReefAlgae;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToBranchScore;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class OneHOneAlgae extends SequentialCommandGroup {
    
    public OneHOneAlgae(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on H
            new ParallelCommandGroup(
                new SwerveDrivePIDToBranchScore(CoralBranch.H, 4, true)
                    .withTranslationalConstraints(1.5, Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION_AUTON.get())
                    .withTimeout(1.75)
                    .deadlineFor(new LEDApplyPattern(CoralBranch.H.isLeftBranchRobotRelative() ? Settings.LED.DEFAULT_ALIGN_COLOR : Settings.LED.ALIGN_RIGHT_COLOR)),
                new SuperStructureCoralL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new ShooterShootL4Front(),
            new WaitCommand(0.15),
            new ShooterStop(),

            // Acquire GH Algae, Score on Barge
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new WaitUntilCommand(() -> Clearances.isArmClearFromReef())
                    .andThen(
                        new SuperStructureAlgaeL2Front()
                            .andThen(new SuperStructureWaitUntilAtTarget())
                    )
            ),
            new ParallelCommandGroup(
                new SwerveDrivePidToNearestReefAlgae(true),
                new ShooterAcquireAlgae()
            ),
            new ShooterHoldAlgae(),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
            new ParallelCommandGroup(
                new SwerveDrivePIDToBarge(),
                new SuperStructureCatapultReady()
                    .andThen(new SuperStructureWaitUntilAtTarget()
                        .alongWith(new SwerveDriveWaitUntilAlignedToCatapultAllianceSide()))
                    .andThen(new SuperStructureCatapultShoot()
                        .andThen(new SuperStructureWaitUntilCanCatapult()
                            .andThen(new ShooterShootAlgae())))
            ),
            new ShooterShootAlgae()
            
        );

    }

}
