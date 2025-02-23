package com.stuypulse.robot.commands.autons.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetPoseToStartOfPath;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class testing extends SequentialCommandGroup {
    
    public testing(PathPlannerPath... paths) {

        addCommands(

            new SwerveDriveResetPoseToStartOfPath(paths[0]),
            
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[5]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[6])

        ); 

    }

}
