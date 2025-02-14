/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.arm.coral;

import com.stuypulse.robot.commands.arm.ArmSetState;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;

public class ArmToL2Front extends ArmSetState {
    public ArmToL2Front() {
        super(ArmState.L2_FRONT);
    }
}
