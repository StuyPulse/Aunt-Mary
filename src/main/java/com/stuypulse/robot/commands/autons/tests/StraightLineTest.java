
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.autons.tests;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.path.PathPlannerPath;

public class StraightLineTest extends SequentialCommandGroup {

    public StraightLineTest(PathPlannerPath... paths) {
        
        addCommands(
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
        );

    }
    
}
