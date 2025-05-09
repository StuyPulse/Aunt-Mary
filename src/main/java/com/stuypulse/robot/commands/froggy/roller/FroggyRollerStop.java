
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.froggy.roller;

import com.stuypulse.robot.subsystems.froggy.Froggy.RollerState;

public class FroggyRollerStop extends FroggyRollerSetState {
    public FroggyRollerStop() {
        super(RollerState.STOP);
    }
}
