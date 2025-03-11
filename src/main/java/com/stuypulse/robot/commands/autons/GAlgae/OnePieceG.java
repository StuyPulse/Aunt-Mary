package com.stuypulse.robot.commands.autons.GAlgae;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.leds.LEDApplyPattern;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.superStructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.superStructure.coral.SuperStructureCoralL4Front;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDriveCoralScoreAlignWithClearance;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToBranchScore;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.ReefUtil.CoralBranch;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.superStructure.elevator.Elevator.ElevatorState;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePieceG extends SequentialCommandGroup {
    
    public OnePieceG(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on G
            new ParallelCommandGroup(
                new SwerveDrivePIDToBranchScore(CoralBranch.G, 4, true)
                    .withTranslationalConstraints(1.5, Settings.Swerve.Alignment.Constraints.MAX_ACCELERATION_AUTON.get())
                    .withTimeout(1.75)
                    .deadlineFor(new LEDApplyPattern(CoralBranch.G.isLeftPeg() ? Settings.LED.LEFT_SIDE_COLOR : Settings.LED.RIGHT_SIDE_COLOR)),
                new SuperStructureCoralL4Front()
                    .andThen(new SuperStructureWaitUntilAtTarget())
            ),
            new ShooterShootBackwards(),
            new WaitCommand(0.15),
            new ShooterStop()
            
        );

    }

}
