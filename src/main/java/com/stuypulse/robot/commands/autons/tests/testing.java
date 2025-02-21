package com.stuypulse.robot.commands.autons.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class testing extends SequentialCommandGroup {
    
    public testing(PathPlannerPath... paths) {

        addCommands(
            
            CommandSwerveDrivetrain.followPathCommand(paths[0]),
            CommandSwerveDrivetrain.followPathCommand(paths[1]),
            CommandSwerveDrivetrain.followPathCommand(paths[2]),
            CommandSwerveDrivetrain.followPathCommand(paths[3]),
            CommandSwerveDrivetrain.followPathCommand(paths[4]),
            CommandSwerveDrivetrain.followPathCommand(paths[5]),
            CommandSwerveDrivetrain.followPathCommand(paths[6])

        ); 

    }

}
