package com.stuypulse.robot.commands.autons.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StraightLineTest extends SequentialCommandGroup {

    public StraightLineTest(PathPlannerPath... paths) {
        
        addCommands(

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])

        );

    }
    
}
