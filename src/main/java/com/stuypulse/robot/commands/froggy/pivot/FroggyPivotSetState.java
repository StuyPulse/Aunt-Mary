
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.froggy.pivot;

import com.stuypulse.robot.subsystems.froggy.*;
import com.stuypulse.robot.subsystems.froggy.Froggy.PivotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

abstract class FroggyPivotSetState extends InstantCommand {

    private final Froggy froggy;
    private final PivotState state;

    public FroggyPivotSetState(PivotState state) {
        froggy = Froggy.getInstance();
        this.state = state;
        addRequirements(froggy);
    }

    @Override
    public void initialize() {
        froggy.setPivotState(state);
    }
}
