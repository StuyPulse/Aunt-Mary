package com.stuypulse.robot.commands.autons.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SquareTest extends SequentialCommandGroup {

    public SquareTest(PathPlannerPath... paths) {
        
        addCommands(

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3])
        );

    }
    
}
