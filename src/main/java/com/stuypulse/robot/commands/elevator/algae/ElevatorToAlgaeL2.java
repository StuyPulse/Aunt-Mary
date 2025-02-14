/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.elevator.algae;

import com.stuypulse.robot.commands.elevator.*;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;

public class ElevatorToAlgaeL2 extends ElevatorToHeight {
    public ElevatorToAlgaeL2() {
        super(ElevatorState.ALGAE_L2);
    }
}
