/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.commands.led.LedRainbow;
import com.stuypulse.robot.subsystems.froggy.*;

import edu.wpi.first.wpilibj2.command.Command;

public class FroggyIntakeAlgae extends Command {

    protected final Froggy froggy;

    public FroggyIntakeAlgae() {
        froggy = Froggy.getInstance();
        addRequirements(froggy);
    }

    @Override
    public void execute() {
        new LedRainbow().schedule();
        froggy.intakeAlgae();
    }

    @Override
    public void end(boolean interrupted) {
        froggy.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return froggy.hasAlgae();
    }
}
