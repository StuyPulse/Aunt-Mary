package com.stuypulse.robot.commands.autons.routines;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.stuypulse.robot.commands.shooter.ShooterShootBackwards;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.superstructure.SuperStructureToL4Front;
import com.stuypulse.robot.commands.superstructure.SuperStructureWaitUntilAtTarget;
import com.stuypulse.robot.commands.swerve.SwerveDrivePIDToNearestBranch;
import com.stuypulse.robot.constants.Settings;

public class ScoreRoutine extends SequentialCommandGroup {

    public ScoreRoutine() {

        addCommands(

            new SuperStructureToL4Front()
                        .andThen(new SuperStructureWaitUntilAtTarget().alongWith(new SwerveDrivePIDToNearestBranch(4, true)))
                        .andThen(new ShooterShootBackwards()),

            new WaitCommand(Settings.Auton.SHOOTER_WAIT_TIME),
            new ShooterStop()
            
        );

    }

}