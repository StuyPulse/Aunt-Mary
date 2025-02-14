/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.elevator.coral;

import com.stuypulse.robot.commands.elevator.ElevatorToHeight;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;

public class ElevatorToL4Back extends ElevatorToHeight {
    public ElevatorToL4Back() {
        super(ElevatorState.L4_BACK);
    }
}
