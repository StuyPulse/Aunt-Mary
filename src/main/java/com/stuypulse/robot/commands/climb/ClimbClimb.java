/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.climb;

import com.stuypulse.robot.subsystems.climb.Climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClimbClimb extends InstantCommand {
    private final Climb climb;

    public ClimbClimb() {
        climb = Climb.getInstance();        
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.climb();
    }
}
