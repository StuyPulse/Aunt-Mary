/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.froggy;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FroggyAlgaeGroundIntake extends SequentialCommandGroup {

    public FroggyAlgaeGroundIntake() {
        addCommands(
                new FroggySetPivot(Settings.Froggy.ALGAE_GROUND_PICKUP_ANGLE),
                new FroggyIntakeAlgae());
    }
}
