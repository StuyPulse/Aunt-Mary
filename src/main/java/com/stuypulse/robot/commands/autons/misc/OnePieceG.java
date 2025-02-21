package com.stuypulse.robot.commands.autons.misc;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.superstructure.SuperStructureToL4Front;
import com.stuypulse.robot.commands.superstructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToNearestBranch;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class OnePieceG extends SequentialCommandGroup {
    
    public OnePieceG(PathPlannerPath... paths) {

        addCommands(

            new SwerveDriveResetPoseToStartOfPath(paths[0]),

            // Score Preload on G
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new SuperStructureToL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new SwerveDrivePIDToNearestBranch(4, true)
                .andThen(new ShooterShootBackwards()),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasCoral()),
            new ShooterStop()
            
        );

    }

}
