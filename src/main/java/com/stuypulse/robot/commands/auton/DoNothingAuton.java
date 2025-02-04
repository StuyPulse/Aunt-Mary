/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/*-
 * This auton does nothing... it is used as a placeholder
 *
 * @author Sam Belliveau
 */
public class DoNothingAuton extends SequentialCommandGroup {

    public DoNothingAuton() {
        addCommands(
                /** Do a whole lot of nothing */
                );
    }
}
