
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.funnel.Funnel.FunnelState;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelStop extends Command{
    private final Funnel funnel;

    public FunnelStop() {
        this.funnel = Funnel.getInstance();
        addRequirements(funnel);
    }

    @Override
    public void initialize() {
        funnel.setState(FunnelState.STOP);
    }
}
