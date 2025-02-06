/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToLvl2Front extends ElevatorToHeight {
    public ElevatorToLvl2Front() {
        super(Elevator.L2_FRONT_HEIGHT);
    }
}
