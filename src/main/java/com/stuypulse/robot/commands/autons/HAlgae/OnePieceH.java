package com.stuypulse.robot.commands.autons.HAlgae;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.arm.coral.ArmToL4Front;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL4Front;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToBranchScore;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToNearestBranchScore;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class OnePieceH extends SequentialCommandGroup {
    
    public OnePieceH(PathPlannerPath... paths) {

        addCommands(
            new SwerveDriveResetPoseToStartOfPath(paths[0]),

            // Score Preload on H
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new ElevatorToL4Front().alongWith(new ArmToL4Front())
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
            ),
            new SwerveDrivePIDToBranchScore(ReefUtil.CoralBranch.H, 4, true)
            // new SwerveDrivePIDToNearestBranchScore(4, true)
                .andThen(new ShooterShootBackwards()),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasCoral()),
            new ShooterStop()
            
        );

    }

}
