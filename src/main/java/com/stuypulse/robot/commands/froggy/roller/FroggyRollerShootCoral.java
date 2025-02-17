/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.froggy.roller;

import com.stuypulse.robot.subsystems.froggy.*;
import com.stuypulse.robot.subsystems.froggy.Froggy.RollerState;

import edu.wpi.first.wpilibj2.command.Command;


public class FroggyRollerShootCoral extends Command {

    protected final Froggy froggy;

    public FroggyRollerShootCoral() {
        froggy = Froggy.getInstance();
        addRequirements(froggy);
    }

    @Override
    public void initialize() {
        froggy.setRollerState(RollerState.SHOOT_ALGAE);
    }
}
