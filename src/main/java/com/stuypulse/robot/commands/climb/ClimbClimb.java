
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.climb;

import com.stuypulse.robot.subsystems.climb.Climb.ClimbState;

public class ClimbClimb extends ClimbToState {
    public ClimbClimb() {
        super(ClimbState.CLIMBING);
    }
}
