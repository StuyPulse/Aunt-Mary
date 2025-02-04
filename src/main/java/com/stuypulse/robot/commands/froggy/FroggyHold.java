/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.subsystems.froggy.Froggy;

import edu.wpi.first.wpilibj2.command.Command;

public class FroggyHold extends Command {

    protected final Froggy froggy;

    public FroggyHold() {
        froggy = Froggy.getInstance();
        addRequirements(froggy);
    }

    @Override
    public void execute() {
        froggy.holdAlgae();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
