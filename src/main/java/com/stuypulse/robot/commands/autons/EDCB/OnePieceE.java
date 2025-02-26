package com.stuypulse.robot.commands.autons.EDCB;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.arm.coral.ArmToL4Front;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL4Front;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToNearestBranchScore;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class OnePieceE extends SequentialCommandGroup {
    
    public OnePieceE(PathPlannerPath... paths) {

        addCommands(
            
            new SwerveDriveResetPoseToStartOfPath(paths[0]),

            // Score Preload on E
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new ElevatorToL4Front().alongWith(new ArmToL4Front())
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
            ),
            new SwerveDrivePIDToNearestBranchScore(4, true)
                .andThen(new ShooterShootBackwards()),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasCoral()),
            new ShooterStop()
            
        );

    }

}
