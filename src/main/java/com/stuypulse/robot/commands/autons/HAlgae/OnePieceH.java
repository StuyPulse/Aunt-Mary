package com.stuypulse.robot.commands.autons.HAlgae;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Front;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePieceH extends SequentialCommandGroup {
    
    public OnePieceH(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on H
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new SuperStructureCoralL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new SwerveDriveCoralScoreAlignWithClearance(CoralBranch.H, 4, true, SuperStructureState.L4_FRONT),
            new ShooterShootBackwards(),
            new WaitCommand(3),
            new ShooterStop()
            
        );

    }

}
