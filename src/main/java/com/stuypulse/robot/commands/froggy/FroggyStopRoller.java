/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.subsystems.froggy.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FroggyStopRoller extends InstantCommand {

    private final Froggy froggy;

    public FroggyStopRoller() {
        froggy = Froggy.getInstance();

        addRequirements(froggy);
    }

    @Override
    public void initialize() {
        froggy.stopRoller();
    }
}
