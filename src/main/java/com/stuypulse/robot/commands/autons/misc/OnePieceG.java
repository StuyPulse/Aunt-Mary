package com.stuypulse.robot.commands.autons.misc;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.autons.routines.ScoreRoutine;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OnePieceG extends SequentialCommandGroup {
    
    public OnePieceG(PathPlannerPath... paths) {

        addCommands(

            // Score Preload on G
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            new ScoreRoutine()
            
        );

    }

}
