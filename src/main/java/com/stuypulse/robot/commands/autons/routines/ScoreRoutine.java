package com.stuypulse.robot.commands.autons.routines;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.arm.coral.ArmToL4Front;
import com.stuypulse.robot.commands.elevator.ElevatorWaitUntilAtTargetHeight;
import com.stuypulse.robot.commands.elevator.coral.ElevatorToL4Front;
import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToCurrentNearestBranchScore;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToNearestBranchScore;
import com.stuypulse.robot.constants.Settings;

public class ScoreRoutine extends SequentialCommandGroup {

    public ScoreRoutine() {

        addCommands(

            new ElevatorToL4Front().alongWith(new ArmToL4Front())
                .andThen(new ElevatorWaitUntilAtTargetHeight()
                    .alongWith(new ArmWaitUntilAtTarget())
                    .alongWith(new SwerveDrivePIDToCurrentNearestBranchScore(4, true)))
                .andThen(new ShooterShootBackwards()),

            new WaitCommand(Settings.Auton.SHOOTER_WAIT_TIME),
            new ShooterStop()
            
        );

    }

}