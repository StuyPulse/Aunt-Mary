
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.froggy.pivot;

import com.stuypulse.robot.subsystems.froggy.Froggy;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FroggyPivotWaitUntilAtTargetAngle extends WaitUntilCommand{
    public FroggyPivotWaitUntilAtTargetAngle() {
        super(() -> Froggy.getInstance().isAtTargetAngle());
    }
}
