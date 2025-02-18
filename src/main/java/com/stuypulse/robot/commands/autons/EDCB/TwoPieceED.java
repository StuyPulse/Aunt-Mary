package com.stuypulse.robot.commands.autons.EDCB;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.autons.routines.ScoreRoutine;
import com.stuypulse.robot.commands.superstructure.SuperStructureToFeed;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoPieceED extends SequentialCommandGroup {
    
    public TwoPieceED(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on E
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            new ScoreRoutine(),

            // To HP, Score on D
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
                new SuperStructureToFeed()
            ),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
            new ScoreRoutine()

        );

    }

}
