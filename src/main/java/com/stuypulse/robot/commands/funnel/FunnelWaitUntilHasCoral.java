
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.Funnel;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FunnelWaitUntilHasCoral extends WaitUntilCommand{
    public FunnelWaitUntilHasCoral() {
        super(() -> Funnel.getInstance().hasCoral());
    }
}
