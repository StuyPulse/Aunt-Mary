
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.climb;

import com.stuypulse.robot.subsystems.climb.Climb;
import com.stuypulse.robot.subsystems.climb.Climb.ClimbState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public abstract class ClimbToState extends InstantCommand{
    private final Climb climb;
    private final ClimbState state;

    public ClimbToState(ClimbState state) {
        this.climb = Climb.getInstance();
        this.state = state;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setState(state);
    }
}
