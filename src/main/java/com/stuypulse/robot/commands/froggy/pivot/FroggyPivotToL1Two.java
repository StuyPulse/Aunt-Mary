
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.froggy.pivot;

import com.stuypulse.robot.subsystems.froggy.Froggy.PivotState;

public class FroggyPivotToL1Two extends FroggyPivotSetState {

    public FroggyPivotToL1Two() {
        super(PivotState.L1_SCORE_ANGLE_TWO);
    }
}
