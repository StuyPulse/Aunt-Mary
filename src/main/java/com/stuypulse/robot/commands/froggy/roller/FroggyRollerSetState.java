
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.froggy.roller;

import com.stuypulse.robot.subsystems.froggy.Froggy;
import com.stuypulse.robot.subsystems.froggy.Froggy.RollerState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FroggyRollerSetState extends InstantCommand{
    private final Froggy froggy;
    private final RollerState rollerState;

    public FroggyRollerSetState(RollerState state) {
        this.froggy = Froggy.getInstance();
        this.rollerState = state;
    }

    @Override
    public void initialize() {
        froggy.setRollerState(rollerState);
    }
}
