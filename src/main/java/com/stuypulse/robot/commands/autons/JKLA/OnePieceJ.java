package com.stuypulse.robot.commands.autons.JKLA;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.arm.coral.ArmToL4Front;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL4Front;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.commands.swerve.SwerveDriveCoralScoreAlignWithClearanceToBranch;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToBranchClear;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToBranchScore;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToNearestBranchScore;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;
import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class OnePieceJ extends SequentialCommandGroup {
    
    public OnePieceJ(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on J
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new ElevatorToL4Front().alongWith(new ArmToL4Front())
                    .andThen(new ElevatorWaitUntilAtTargetHeight().alongWith(new ArmWaitUntilAtTarget()))
            ),
            new SwerveDriveCoralScoreAlignWithClearanceToBranch(CoralBranch.I, 4, true, ElevatorState.L4_FRONT, ArmState.L4_FRONT),
            new ShooterShootBackwards(),
            //new WaitUntilCommand(() -> !Shooter.getInstance().hasCoral()),
            new WaitCommand(0.5),
            new ShooterStop(),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1])
            
        );

    }

}
