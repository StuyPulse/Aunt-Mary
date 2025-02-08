/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Constants.Elevator;

public class ElevatorToBottom extends ElevatorToHeight{
    public ElevatorToBottom() {
        super(Elevator.MIN_HEIGHT_METERS);
    }
}