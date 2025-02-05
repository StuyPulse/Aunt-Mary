/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Constants;

public class ElevatorToBarge extends ElevatorToHeight {
    public ElevatorToBarge() {
        super(Constants.Elevator.MAX_HEIGHT_METERS);
    }
}
